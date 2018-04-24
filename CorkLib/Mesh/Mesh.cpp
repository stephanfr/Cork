
// +-------------------------------------------------------------------------
// | Mesh.cpp
// | 
// | Author: Gilbert Bernstein
// +-------------------------------------------------------------------------
// | COPYRIGHT:
// |    Copyright Gilbert Bernstein 2013
// |    See the included COPYRIGHT file for further details.
// |    
// |    This file is part of the Cork library.
// |
// |    Cork is free software: you can redistribute it and/or modify
// |    it under the terms of the GNU Lesser General Public License as
// |    published by the Free Software Foundation, either version 3 of
// |    the License, or (at your option) any later version.
// |
// |    Cork is distributed in the hope that it will be useful,
// |    but WITHOUT ANY WARRANTY; without even the implied warranty of
// |    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// |    GNU Lesser General Public License for more details.
// |
// |    You should have received a copy 
// |    of the GNU Lesser General Public License
// |    along with Cork.  If not, see <http://www.gnu.org/licenses/>.
// +-------------------------------------------------------------------------


#include "..\Intersection\gmpext4.h"

#include <sstream>

#include "boost\container\small_vector.hpp"

#include "tbb/tbb.h"

#include "..\cork.h"

#include "..\Mesh\TopoCache.h"
#include "..\Mesh\IntersectionProblem.h"
#include "..\Mesh\EGraphCache.h"

#include "..\Intersection\unsafeRayTriIsct.h"

#include "..\Util\unionFind.h"
#include "..\Util\SystemStats.h"

#include "..\Util\CachingFactory.h"

#include <boost\chrono\chrono.hpp>			//	Had to include when I upgraded to boost 65 - on windows is griped about a missing library
#include <boost\timer\timer.hpp>



CachingFactory<Cork::TopoCacheWorkspace>::CacheType		CachingFactory<Cork::TopoCacheWorkspace>::m_cache;




namespace Cork
{

	using namespace Intersection;


	
	inline
	double triArea( const Cork::Math::Vector3D&		a,
					const Cork::Math::Vector3D&		b,
					const Cork::Math::Vector3D&		c )
	{
		return( len( cross( b-a, c-a )));
	}


		
	//
	//	The Mesh class brings together the functionality needed for the boolean operations
	//

	class Mesh : public MeshBase, public CorkMesh 
	{
	public:


		Mesh()
		{}

		Mesh( const Mesh&					src,
			  const SolverControlBlock		controlBlock )
			: MeshBase( src, controlBlock )
		{};

		explicit
		Mesh( const Cork::TriangleMesh&	inputMesh );

		virtual ~Mesh();
    

		void operator=(Mesh&&	src);
    

		//	Validity check:
		//		- all numbers are well-defined and finite
		//		- all triangle vertex indices are in the right range
 
		bool valid() const;
    

		// form the disjoint union of two meshes

		void							DisjointUnion( const Mesh&					meshToMerge );

		//	Intersection module

//		SelfIntersectionStatistics		ComputeSelfIntersectionStatistics();							//	Is the mesh self-intersecting?
    
		//	Boolean operations

		BooleanOperationResult			Union( const CorkMesh&								rhs,
											   const SolverControlBlock&					solverControlBlock = GetDefaultControlBlock() ) const;
		BooleanOperationResult			Difference( const CorkMesh&							rhs,
													const SolverControlBlock&				solverControlBlock = GetDefaultControlBlock() ) const;
		BooleanOperationResult			Intersection( const CorkMesh&						rhs,
													  const SolverControlBlock&				solverControlBlock = GetDefaultControlBlock() ) const;
		BooleanOperationResult			SymmetricDifference( const CorkMesh&				rhs,
															 const SolverControlBlock&		solverControlBlock = GetDefaultControlBlock() ) const;
    
		std::unique_ptr<TriangleMesh>	ToTriangleMesh() const;

		const SolverPerformanceStatisticsIfx&	GetPerformanceStats() const
		{
			return( m_performanceStats );
		}


	private:


		enum class TriCode { KEEP_TRI, DELETE_TRI, FLIP_TRI };


		enum class SetupBooleanProblemResultCodes { SUCCESS = 0,
													TOO_MANY_TRIANGLES_IN_DISJOINT_UNION,
													INSUFFICIENT_PERTURBATION_RANGE,
													FIND_INTERSECTIONS_FAILED,
													RESOLVE_INTERSECTIONS_FAILED,
													POPULATE_EDGE_GRAPH_CACHE_FAILED,
													SELF_INTERSECTING_MESH };

		typedef SEFUtility::Result<SetupBooleanProblemResultCodes>		SetupBooleanProblemResult;


		enum class PopulateEGraphCacheResultCodes { SUCCESS = 0,
													OUT_OF_MEMORY };

		typedef SEFUtility::Result<PopulateEGraphCacheResultCodes>		PopulateEGraphCacheResult;



		SetupBooleanProblemResult		SetupBooleanProblem( const Mesh&										rhs );
	
		void							doDeleteAndFlip( std::function<TriCode(byte bool_alg_data)>				classify );

		PopulateEGraphCacheResult		populateEGraphCache( EGraphCache&										ecache);
	
		void							for_ecache( EGraphCache&												ecache,
													std::function<void(const EGraphEntryTIDVector&	tids)>		action );
	
		bool							isInside( IndexType		tid,
												  byte			operand );

		void							IsInsideCheck( CorkTriangle&				tri,
													   Cork::Math::Ray3D&			r,
													   std::atomic<int>&			winding );
	};





	inline void Mesh::for_ecache( EGraphCache&												ecache,
								  std::function<void(const EGraphEntryTIDVector&	tids)>	action )
	{
		for( auto& column : ecache.columns() )
		{
			column.for_each( [this,&action](EGraphEntry&	entry)
			{
				if( entry.isIsct() )
				{
					EGraphEntryTIDVector		tid0s;
					EGraphEntryTIDVector		tid1s;
            
					for( IndexType tid : entry.tids() )
					{
						if( m_tris[tid].boolAlgData() & 1 )
						{
							tid1s.push_back( tid );
						}
						else
						{
							tid0s.push_back( tid );
						}
					}

					action( tid1s );
					action( tid0s );
				}
				else
				{
					action( entry.tids() );
				}
			});
		}
	}


	inline
	bool	Mesh::isInside( IndexType tid, byte operand)
	{
		// find the point to trace outward from...
    
		Cork::Math::Vector3D		p(m_verts[m_tris[tid].a()]);

		p += m_verts[m_tris[tid].b()];
		p += m_verts[m_tris[tid].c()];
		p /= 3.0;
        
		// ok, we've got the point, now let's pick a direction
        
		Cork::Math::Ray3D		r( p, Cork::Math::Vector3D::randomVector( 0.5, 1.5 ) );
        
		std::atomic<int> winding = 0;

		// pass all triangles over ray
        
		if( solverControlBlock().useMultipleThreads() )
		{
			tbb::parallel_for( tbb::blocked_range<std::vector<Cork::CorkTriangle>::iterator>( m_tris.begin(), m_tris.end(), m_tris.size() / 4 ),
			[&]( tbb::blocked_range<std::vector<Cork::CorkTriangle>::iterator> triangles )
			{
				for(CorkTriangle &tri : triangles )
				{
					// ignore triangles from the same operand surface
					if ((tri.boolAlgData() & 1) == operand)
					{
						continue;
					}

					IsInsideCheck( tri, r, winding );
				}
			}, tbb::simple_partitioner() );
		}
		else
		{
			for( CorkTriangle &tri : m_tris )
			{
				// ignore triangles from the same operand surface
				if( ( tri.boolAlgData() & 1 ) == operand )
				{
					continue;
				}

				IsInsideCheck( tri, r, winding );
			}
		}
        
		// now, we've got a winding number to work with...
		return( winding > 0 );
	}


	inline
	void	Mesh::IsInsideCheck( CorkTriangle&				tri,
								 Cork::Math::Ray3D&			r,
								 std::atomic<int>&			winding )
	{
		NUMERIC_PRECISION flip = 1.0;

		IndexType   a = tri.a();
		IndexType   b = tri.b();
		IndexType   c = tri.c();

		Cork::Math::Vector3D		va = m_verts[a];
		Cork::Math::Vector3D		vb = m_verts[b];
		Cork::Math::Vector3D		vc = m_verts[c];

		// normalize vertex order (to prevent leaks)

		if( a > b )
		{
			std::swap( a, b ); std::swap( va, vb ); flip = -flip;
		}

		if( b > c )
		{
			std::swap( b, c ); std::swap( vb, vc ); flip = -flip;
		}

		if( a > b )
		{
			std::swap( a, b ); std::swap( va, vb ); flip = -flip;
		}

		NUMERIC_PRECISION			t;
		Cork::Math::Vector3D		bary;

		if( isct_ray_triangle( r, va, vb, vc, &t, &bary ) )
		{
			Cork::Math::Vector3D	normal = flip * cross( vb - va, vc - va );

			if( dot( normal, r.direction() ) > 0.0 )
			{
				winding++;
			}
			else
			{
				winding--;
			}
		}
	}


	//
	//	Constructors and assignment operators
	//


	Mesh::Mesh( const Cork::TriangleMesh&		inputMesh )
	{
		m_tris.reserve( inputMesh.numTriangles() );
		m_verts.reserve( inputMesh.numVertices() );
	
		//	Start by copying the vertices.  We need to cast them from floats to doubles.

		for ( Cork::TriangleMesh::Vertex currentVertex : inputMesh.vertices() )
		{
			m_verts.emplace_back( currentVertex.x(), currentVertex.y(), currentVertex.z() );
		}

		//	Fill the triangles

		for ( uint i = 0; i < inputMesh.triangles().size(); i++ )
		{
			m_tris.emplace_back( inputMesh.triangles()[i], 0 );
		}

		m_boundingBox = make_aligned<Cork::Math::BBox3D>( inputMesh.boundingBox() );
	}



	Mesh::~Mesh()
	{}




	void Mesh::operator=(Mesh &&src)
	{
		m_tris = src.m_tris;
		m_verts = src.m_verts;
	}


	bool Mesh::valid() const
	{
		for( unsigned int i=0; i< m_verts.size(); i++)
		{
			if(!std::isfinite(m_verts[i].x()) ||
			   !std::isfinite(m_verts[i].y()) ||
			   !std::isfinite(m_verts[i].z()))
			{
				std::ostringstream message;
				message << "vertex #" << i << " has non-finite coordinates: " << m_verts[i];
				CORK_ERROR(message.str());
				return false;
			}
		}
    
		for( unsigned int i=0; i<m_tris.size(); i++)
		{
			if(m_tris[i].a() >= m_verts.size() ||
			   m_tris[i].b() >= m_verts.size() ||
			   m_tris[i].c() >= m_verts.size())
			{
				std::ostringstream message;
				message << "triangle #" << i << " should have indices in "
						<< "the range 0 to " << (m_verts.size()-1)
						<< ", but it has invalid indices: "
						<< m_tris[i].a() << ", " << m_tris[i].b() << ", " << m_tris[i].c();
				CORK_ERROR(message.str());
				return false;
			}
		}
    
		return( true );
	}



	void Mesh::DisjointUnion( const Mesh&		meshToMerge )
	{
		//	Reset the labels on this mesh's collection of triangles

		for( auto& t : triangles() )
		{
			t.boolAlgData() = 0;
		}
	
		size_t		oldVsize = m_verts.size();
		size_t		oldTsize = m_tris.size();
		size_t		cpVsize  = meshToMerge.m_verts.size();
		size_t		cpTsize  = meshToMerge.m_tris.size();
		size_t		newVsize = oldVsize + cpVsize;
		size_t		newTsize = oldTsize + cpTsize;
    
		m_verts.resize( newVsize );
		m_tris.resize( newTsize );
    
		for( unsigned int i = 0; i < cpVsize; i++ )
		{
			m_verts[oldVsize + i] = meshToMerge.m_verts[i];
		}

		for( unsigned int i = 0; i < cpTsize; i++ )
		{
			auto &tri = m_tris[oldTsize + i];

			tri = meshToMerge.m_tris[i];
			tri.boolAlgData() = 1;					//	These triangles are part of the RHS so label them as such
			tri.offsetIndices( oldVsize );
		}
	}



    
	Mesh::PopulateEGraphCacheResult  Mesh::populateEGraphCache( EGraphCache&		ecache )
	{
		try
		{
			ecache.resize(m_verts.size());
		}
		catch (std::bad_alloc&		ex)
		{
			return( PopulateEGraphCacheResult::Failure( PopulateEGraphCacheResultCodes::OUT_OF_MEMORY, "Out of Memory resizing the edge cache" ));
		}
    
		for(uint tid = 0; tid < m_tris.size(); tid++)
		{
			const CorkTriangle&		tri = m_tris[tid];

			ecache[tri.a()].find_or_add(tri.b()).tids().push_back(tid);
			ecache[tri.a()].find_or_add(tri.c()).tids().push_back(tid);

			ecache[tri.b()].find_or_add(tri.a()).tids().push_back(tid);
			ecache[tri.b()].find_or_add(tri.c()).tids().push_back(tid);

			ecache[tri.c()].find_or_add(tri.a()).tids().push_back(tid);
			ecache[tri.c()].find_or_add(tri.b()).tids().push_back(tid);
		}
        
		//	Label some of the edges as intersection edges and others as not

		for( auto& column : ecache.columns() )
		{
			column.for_each([this](EGraphEntry &entry)
			{
				entry.setIsIsct( false );
				byte operand = m_tris[entry.tids()[0]].boolAlgData();

				for(uint k=1; k<entry.tids().size(); k++)
				{
					if( m_tris[entry.tids()[k]].boolAlgData() != operand)
					{
						entry.setIsIsct( true );
						break;
					}
				}
			});
		}

		//	Finished with success

		return(PopulateEGraphCacheResult::Success());
	}




	Mesh::SetupBooleanProblemResult		Mesh::SetupBooleanProblem( const Mesh&		rhs )
	{
		auto		intersectionBBox = m_boundingBox->intersection( rhs.boundingBox() );
	
		//	Form the disjoint union of this mesh and the second operand mesh
		
		DisjointUnion( rhs );

		m_performanceStats.setNumberOfTrianglesInDisjointUnion( (unsigned long)this->m_tris.size() );
		m_controlBlock->setNumTriangles( (unsigned long)this->m_tris.size() );

		if( this->m_tris.size() >= MAX_TRIANGLES_IN_DISJOINT_UNION )
		{
			return( SetupBooleanProblemResult::Failure( SetupBooleanProblemResultCodes::TOO_MANY_TRIANGLES_IN_DISJOINT_UNION, "Too many triangles in disjoint union, possible out of memory exception if the operation is attenmpted." ));
		}

		//	Start by finding the intersections

		int		tries = 5;
		
		Quantization::Quantizer			quantizer = getQuantizer();

		if (!quantizer.sufficientPerturbationRange())
		{
			return(SetupBooleanProblemResult::Failure(SetupBooleanProblemResultCodes::INSUFFICIENT_PERTURBATION_RANGE, "Insufficient Dynamic Range left in model for perturbation."));
		}

		//	Find intersections and then resolve them.  We might have to repurturb if finding and resolving fails.
		//		We can repurturb until we run out of perturbation resolution.

		std::unique_ptr<IntersectionProblemIfx>					iproblem( IntersectionProblemIfx::GetProblem( *this, quantizer, intersectionBBox ) );

		while( true )
		{
			IntersectionProblemIfx::IntersectionProblemResult		findResult = iproblem->FindIntersections();

			if( !findResult.Succeeded() )
			{
				//	If we failed here - not mush to do but return a failed result

				return( SetupBooleanProblemResult::Failure( SetupBooleanProblemResultCodes::FIND_INTERSECTIONS_FAILED, "FindIntersections failed.", findResult ));
			}

			//	Next, resolve them

			IntersectionProblemIfx::IntersectionProblemResult		resolveResult = iproblem->ResolveAllIntersections();

			if( !resolveResult.Succeeded() )
			{
				//	Resolve failed, check the error code to see if this is a recoverable error or not

				//	If we failed due to a self-intersection, then one of the meshes is bad so no amount of repurturbation will work.

				if( resolveResult.errorCode() == IntersectionProblemIfx::IntersectionProblemResultCodes::SELF_INTERSECTING_MESH )
				{
					return( SetupBooleanProblemResult::Failure( SetupBooleanProblemResultCodes::SELF_INTERSECTING_MESH, "One of the two meshes self intersects", resolveResult ) );
				}

				//	Resolve failed for some other reason.

				return( SetupBooleanProblemResult::Failure( SetupBooleanProblemResultCodes::RESOLVE_INTERSECTIONS_FAILED, "ResolveIntersections failed and exhuasted perturbations.", resolveResult ) );
			}
    
			iproblem->commit();
			break;
		}

		//	Create and populate the EGraph Cache

		EGraphCache				ecache;

		auto popEGraphCacheResult = populateEGraphCache( ecache );

		if (!popEGraphCacheResult.Succeeded())
		{
			return(SetupBooleanProblemResult::Failure(SetupBooleanProblemResultCodes::POPULATE_EDGE_GRAPH_CACHE_FAILED, "Edge Cache population failed.", popEGraphCacheResult));
		}
    
		// form connected components;
		// we get one component for each connected component in one
		// of the two input meshes.
		// These components are not necessarily uniformly inside or outside
		// of the other operand mesh.

		UnionFind uf(m_tris.size());
    
		for_ecache(ecache, [&uf](const EGraphEntryTIDVector&	tids)
		{
			size_t		tid0 = tids[0];

			for ( size_t k = 1; k < tids.size(); k++)
			{
				uf.unionIds(tid0, tids[k]);
			}
		});
    
		// we re-organize the results of the union find as follows:
    
		std::vector<long>						uq_ids(m_tris.size(), long(-1));
		std::vector< std::vector<size_t> >		components;

		components.reserve( m_tris.size() );

		for( size_t i = 0; i < m_tris.size(); i++)
		{
			size_t		ufid = uf.find(i);
        
			if( uq_ids[ufid] == long(-1) )
			{ // unassigned
				size_t N = components.size();
				components.emplace_back();
				components.back().reserve( 24 );
            
				uq_ids[ufid] = uq_ids[i] = (long)N;
				components[N].push_back(i);
			}
			else
			{ // assigned already
				uq_ids[i] = uq_ids[ufid]; // propagate assignment
				components[uq_ids[i]].push_back(i);
			}
		}
    
		std::vector<bool> visited( m_tris.size(), false );
    
		// find the "best" triangle in each component,
		// and ray cast to determine inside-ness vs. outside-ness

		for( auto&	comp : components )
		{
			// find max according to score

			size_t			best_tid = comp[0];
			double			best_area = 0.0;

			// SEARCH

			for( size_t tid : comp )
			{
				const Cork::Math::Vector3D&		va = m_verts[m_tris[tid].a()];
				const Cork::Math::Vector3D&		vb = m_verts[m_tris[tid].b()];
				const Cork::Math::Vector3D&		vc = m_verts[m_tris[tid].c()];
            
				double area = triArea( va, vb, vc );

				if( area > best_area )
				{
					best_area = area;
					best_tid = tid;
				}
			}
        
			byte operand = m_tris[best_tid].boolAlgData();
			bool inside = isInside(best_tid, operand);
        
			//	Do a breadth first propagation of classification throughout the component.

			std::vector<size_t>	work;
		
			work.reserve( 1024 );

			// begin by tagging the first triangle

			m_tris[best_tid].boolAlgData() |= (inside) ? 2 : 0;
			visited[best_tid] = true;
			work.push_back(best_tid);
        
			while( !work.empty() )
			{
				size_t		curr_tid = work.back();
				work.pop_back();
            
				for(uint k=0; k<3; k++)
				{
					size_t		a = m_tris[curr_tid][k];
					size_t		b = m_tris[curr_tid][(k+1)%3];

					auto&	entry = ecache[a][b];

					byte inside_sig = m_tris[curr_tid].boolAlgData() & 2;
                
					if( entry.isIsct() )
					{
						inside_sig ^= 2;
					}

					for( size_t		tid : entry.tids() )
					{
						if (visited[tid])
						{
							continue;
						}

						if (( m_tris[tid].boolAlgData() & 1) != operand)
						{
							continue;
						}
                    
						m_tris[tid].boolAlgData() |= inside_sig;
						visited[tid] = true;
						work.push_back(tid);
					}
				}
			}
		}

		//	Finished with Success


		return( SetupBooleanProblemResult::Success() );
	}





	void Mesh::doDeleteAndFlip( std::function<TriCode(byte bool_alg_data)> classify )
	{
		CachingFactory<TopoCacheWorkspace>::UniquePtr			topoCacheWorkspace( CachingFactory<TopoCacheWorkspace>::GetInstance());
		
		TopoCache				topocache( *this, *topoCacheWorkspace );
    
		std::vector<Tptr>		toDelete;
    
		toDelete.reserve( topocache.triangles().size() / 2 );


		for( auto& currentTriangle : topocache.triangles() )
		{
			TriCode code = classify( m_tris[currentTriangle.ref()].boolAlgData() );

			switch(code)
			{
				case TriCode::DELETE_TRI:
					toDelete.push_back( &currentTriangle );
					break;
        
				case TriCode::FLIP_TRI:
					topocache.flipTri(  &currentTriangle );
					break;
        
				case TriCode::KEEP_TRI:
        
				default:
					break;
			}
		}
    
		for(Tptr tptr : toDelete)
		{
			topocache.deleteTri(tptr);
		}
    
		topocache.commit();
	}




	Mesh::BooleanOperationResult	Mesh::Union( const CorkMesh&					rhs,
												 const SolverControlBlock&			solverControlBlock ) const
	{
		//	Collect some starting statistics
		
		unsigned long				startingVirtualMemory = GetConsumedVirtualMemory();
		
		boost::timer::cpu_timer		elapsedTime;

		elapsedTime.start();

		//	Duplicate the mesh

		std::unique_ptr<Mesh>			resultMesh( new Mesh( *this, solverControlBlock ));

		resultMesh->m_performanceStats.setStartingVirtualMemorySizeInMB( startingVirtualMemory );

		SetupBooleanProblemResult		result = resultMesh->SetupBooleanProblem( dynamic_cast<const Cork::Mesh&>(rhs) );

		if( !result.Succeeded() )
		{
			return( BooleanOperationResult::Failure( BooleanOperationResultCodes::ERROR_DURING_BOOLEAN_PROBLEM_SETUP, "Error Occurred During Boolean Problem Setup Phase.", result ) );
		}

		resultMesh->doDeleteAndFlip([](byte data) -> TriCode
		{
			if((data & 2) == 2)     // part of op 0/1 INSIDE op 1/0
			{
				return TriCode::DELETE_TRI;
			}
			else                    // part of op 0/1 OUTSIDE op 1/0
			{
				return TriCode::KEEP_TRI;
			}
		});

		//	Collect the ending statistics

		elapsedTime.stop();
		resultMesh->m_performanceStats.setElapsedCPUTimeInNanoSeconds( elapsedTime.elapsed().system + elapsedTime.elapsed().user );
		resultMesh->m_performanceStats.setElapsedWallTimeInNanoSeconds( elapsedTime.elapsed().wall + elapsedTime.elapsed().wall );

		resultMesh->m_performanceStats.setNumberOfTrianglesInFinalMesh( (unsigned long)resultMesh->triangles().size() );
		resultMesh->m_performanceStats.setEndingVirtualMemorySizeInMB( GetConsumedVirtualMemory() );

		//	Finished with success

		return( BooleanOperationResult::Success( resultMesh.release() ));
	}



	Mesh::BooleanOperationResult	Mesh::Difference( const CorkMesh&				rhs,
													  const SolverControlBlock&		solverControlBlock ) const
	{
		//	Collect some starting statistics
		
		unsigned long		startingVirtualMemory = GetConsumedVirtualMemory();
		
		boost::timer::cpu_timer		elapsedTime;

		elapsedTime.start();

		//	Duplicate the mesh

		std::unique_ptr<Mesh>			resultMesh( new Mesh( *this, solverControlBlock ) );

		resultMesh->m_performanceStats.setStartingVirtualMemorySizeInMB( startingVirtualMemory );

		SetupBooleanProblemResult		result = resultMesh->SetupBooleanProblem( dynamic_cast<const Cork::Mesh&>(rhs) );

		if( !result.Succeeded() )
		{
			return( BooleanOperationResult::Failure( BooleanOperationResultCodes::ERROR_DURING_BOOLEAN_PROBLEM_SETUP, "Error Occurred During Boolean Problem Setup Phase.", result ) );
		}
    
		resultMesh->doDeleteAndFlip([](byte data) -> TriCode
		{
			if(data == 2 || data == 1)      // part of op 0 INSIDE op 1, part of op 1 OUTSIDE op 0
			{
				return TriCode::DELETE_TRI;
			}
			else if(data == 3)      // part of op 1 INSIDE op 1
			{
				return TriCode::FLIP_TRI;
			}
			else                    // part of op 0 OUTSIDE op 1
			{
				return TriCode::KEEP_TRI;
			}
		});

		//	Collect the ending statistics

		elapsedTime.stop();
		resultMesh->m_performanceStats.setElapsedCPUTimeInNanoSeconds( elapsedTime.elapsed().system + elapsedTime.elapsed().user );
		resultMesh->m_performanceStats.setElapsedWallTimeInNanoSeconds( elapsedTime.elapsed().wall + elapsedTime.elapsed().wall );

		resultMesh->m_performanceStats.setNumberOfTrianglesInFinalMesh( (unsigned long)resultMesh->triangles().size() );
		resultMesh->m_performanceStats.setEndingVirtualMemorySizeInMB( GetConsumedVirtualMemory() );

		//	Finished with success

		return( BooleanOperationResult::Success( resultMesh.release() ));
	}



	Mesh::BooleanOperationResult	Mesh::Intersection( const CorkMesh&					rhs,
														const SolverControlBlock&		solverControlBlock ) const
	{
		//	Collect some starting statistics
		
		unsigned long			startingVirtualMemory = GetConsumedVirtualMemory();
		
		boost::timer::cpu_timer		elapsedTime;

		elapsedTime.start();

		//	Duplicate the mesh

		std::unique_ptr<Mesh>			resultMesh( new Mesh( *this, solverControlBlock ) );

		resultMesh->m_performanceStats.setStartingVirtualMemorySizeInMB( startingVirtualMemory );

		SetupBooleanProblemResult		result = resultMesh->SetupBooleanProblem( dynamic_cast<const Cork::Mesh&>(rhs) );

		if( !result.Succeeded() )
		{
			return( BooleanOperationResult::Failure( BooleanOperationResultCodes::ERROR_DURING_BOOLEAN_PROBLEM_SETUP, "Error Occurred During Boolean Problem Setup Phase.", result ) );
		}

		//	Don't let the returns below confuse you - the code is a lambda

		resultMesh->doDeleteAndFlip([](byte data) -> TriCode
		{
			if((data & 2) == 0)     // part of op 0/1 OUTSIDE op 1/0
			{
				return( TriCode::DELETE_TRI );
			}
			else                    // part of op 0/1 INSIDE op 1/0
			{
				return( TriCode::KEEP_TRI );
			}
		});

		//	Collect the ending statistics

		elapsedTime.stop();
		resultMesh->m_performanceStats.setElapsedCPUTimeInNanoSeconds( elapsedTime.elapsed().system + elapsedTime.elapsed().user );
		resultMesh->m_performanceStats.setElapsedWallTimeInNanoSeconds( elapsedTime.elapsed().wall + elapsedTime.elapsed().wall );

		resultMesh->m_performanceStats.setNumberOfTrianglesInFinalMesh( (unsigned long)resultMesh->triangles().size() );
		resultMesh->m_performanceStats.setEndingVirtualMemorySizeInMB( GetConsumedVirtualMemory() );

		//	Finished with success

		return( BooleanOperationResult::Success( resultMesh.release() ));
	}



	Mesh::BooleanOperationResult	Mesh::SymmetricDifference( const CorkMesh&					rhs,
															   const SolverControlBlock&		solverControlBlock ) const
	{
		//	Collect some starting statistics
		
		unsigned long			startingVirtualMemory = GetConsumedVirtualMemory();
		
		boost::timer::cpu_timer		elapsedTime;

		elapsedTime.start();

		//	Duplicate the mesh

		std::unique_ptr<Mesh>			resultMesh( new Mesh( *this, solverControlBlock ) );

		resultMesh->m_performanceStats.setStartingVirtualMemorySizeInMB( startingVirtualMemory );

		SetupBooleanProblemResult		result = resultMesh->SetupBooleanProblem( dynamic_cast<const Cork::Mesh&>(rhs) );

		if( !result.Succeeded() )
		{
			return( BooleanOperationResult::Failure( BooleanOperationResultCodes::ERROR_DURING_BOOLEAN_PROBLEM_SETUP, "Error Occurred During Boolean Problem Setup Phase.", result ) );
		}
    
		//	Don't let the returns below confuse you - the code is a lambda

		resultMesh->doDeleteAndFlip([](byte data) -> TriCode
		{
			if((data & 2) == 0 )     // part of op 0/1 OUTSIDE op 1/0
			{
				return( TriCode::KEEP_TRI );
			}
			else if((data & 2) == 2 )
			{
				return( TriCode::DELETE_TRI );
			}
			else
			{
				return( TriCode::FLIP_TRI );
			}
		});

		//	Collect the ending statistics

		elapsedTime.stop();
		resultMesh->m_performanceStats.setElapsedCPUTimeInNanoSeconds( elapsedTime.elapsed().system + elapsedTime.elapsed().user );
		resultMesh->m_performanceStats.setElapsedWallTimeInNanoSeconds( elapsedTime.elapsed().wall + elapsedTime.elapsed().wall );

		resultMesh->m_performanceStats.setNumberOfTrianglesInFinalMesh( (unsigned long)resultMesh->triangles().size() );
		resultMesh->m_performanceStats.setEndingVirtualMemorySizeInMB( GetConsumedVirtualMemory() );

		//	Finished with success

		return( Mesh::BooleanOperationResult::Success( resultMesh.release() ));
	}

	
	std::unique_ptr<TriangleMesh>				Mesh::ToTriangleMesh() const
	{
		std::unique_ptr<Cork::IncrementalVertexIndexTriangleMeshBuilder>		triangleMeshBuilder( Cork::IncrementalVertexIndexTriangleMeshBuilder::GetBuilder( vertices().size(), triangles().size() ) );

		for( auto& currentVertex : vertices() )
		{
			triangleMeshBuilder->AddVertex(Cork::TriangleMesh::Vertex((NUMERIC_PRECISION)currentVertex.x(), (NUMERIC_PRECISION)currentVertex.y(), (NUMERIC_PRECISION)currentVertex.z()));
		}

		for_raw_tris( [&]( IndexType a, IndexType b, IndexType c )
		{
			triangleMeshBuilder->AddTriangle( TriangleMesh::TriangleByIndices( a, b, c ) );
		} );

		return(triangleMeshBuilder->Mesh());
	}


	
	std::unique_ptr<CorkMesh>			CorkMesh::FromTriangleMesh( const TriangleMesh&		triangleMesh )
	{
		return(std::unique_ptr<CorkMesh>( new Mesh( triangleMesh ) ));
	}


	const SolverControlBlock&					CorkMesh::GetDefaultControlBlock()
	{
		static	SolverControlBlock		defaultBlock( true, (long)100000, true );
		
		return( defaultBlock );
	}


//	SelfIntersectionStatistics			ComputeTriangleMeshSelfIntersectionStatistics( const Cork::TriangleMesh&		triangleMeshToTest )
//	{
//		Mesh			meshForTesting( triangleMeshToTest );
//
//		return( meshForTesting.ComputeSelfIntersectionStatistics() );
//	}

}	//	Namespace Cork
