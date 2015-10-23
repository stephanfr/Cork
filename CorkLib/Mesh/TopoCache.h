// +-------------------------------------------------------------------------
// | TopoCache.h
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
#pragma once




#include "MeshBase.h"

#include <array>

#include <boost\dynamic_bitset.hpp>
#include <boost\optional.hpp>
#include <boost\container\static_vector.hpp>
#include <boost\container\small_vector.hpp>

#include <Utility\AlignedUniquePtr.h>
#include <Utility\SparseVector.h>

#include "..\Util\ManagedIntrusiveList.h"

#include "..\Intersection\empty3d.h"
#include "..\Intersection\quantization.h"

#include <tbb\spin_mutex.h>



namespace Cork
{


	/*
	 *  Allows for topological algorithms to manipulate
	 *  a more familiar pointer data structure based on a simplicial complex.
	 *  This structure can be regenerated from the more basic
	 *  vertex/triangle arrays using
	 *      createTopoCache()
	 *  Once manipulations have been satisfactorily performed,
	 *  the underlying vertex/triangle arrays can be cleaned up for
	 *  further use by topologically insensitive algorithms by
	 *      commitTopoCache()
	 */



	#define INVALID_ID uint(-1)


	class TopoVert;
	typedef TopoVert*		Vptr;

	class TopoTri;
	typedef TopoTri*		Tptr;

	class TopoEdge;
	typedef TopoEdge*		Eptr;




	typedef SEFUtility::SearchablePointerList<TopoTri,10>			TopoTrianglePointerList;
	typedef SEFUtility::SearchablePointerList<TopoEdge,10>			TopoEdgePointerList;




	class TopoVert final : public boost::noncopyable, public IntrusiveListHookNoDestructorOnElements
	{
	public :

		TopoVert()
			: m_ref( INVALID_ID ),
			  m_data( nullptr )
		{}

		explicit
		TopoVert( IndexType	ref )
			: m_ref( ref ),
			  m_data( nullptr )
		{}

		~TopoVert()
		{}


		IndexType								ref() const
		{
			return( m_ref );
		}

		void									setRef( IndexType		newValue )
		{
			m_ref = newValue;
		}


		const Cork::Math::Vector3D*				quantizedValue() const
		{
			return( m_data );
		}

		void									setQuantizedValue( Cork::Math::Vector3D*		newValue )
		{
			m_data = newValue;
		}


		void									addTriangle( TopoTri&		triangle )
		{
			m_tris.insert( &triangle );
		}

		const TopoTrianglePointerList&			triangles() const
		{
			return( m_tris );
		}

		TopoTrianglePointerList&				triangles()
		{
			return( m_tris );
		}


		const TopoEdgePointerList&				edges() const
		{
			return( m_edges );
		}

		TopoEdgePointerList&					edges()
		{
			return( m_edges );
		}



	private :

		IndexType						m_ref;        // index to actual data
		Cork::Math::Vector3D*			m_data;       // algorithm specific handle
    
		TopoTrianglePointerList	        m_tris;       // triangles this vertex is incident on
		TopoEdgePointerList				m_edges;      // edges this vertex is incident on
	};


	typedef ManagedIntrusiveValueList<TopoVert>			TopoVertexList;




	class TopoEdge final : public boost::noncopyable, public IntrusiveListHookNoDestructorOnElements
	{
	public :

		TopoEdge()
		{}

		TopoEdge( TopoVert*		vertex0,
				  TopoVert*		vertex1 )
			: m_verts( {{ vertex0, vertex1 }} )
		{
			vertex0->edges().insert(this);
			vertex1->edges().insert(this);
		}




		~TopoEdge()
		{}



		void*								data() const
		{
			return( m_data );
		}

		void								setData( void*	newValue)
		{
			m_data = newValue;
		}


		byte								boolAlgData() const
		{
			return( m_boolAlgData );
		}

		void								setBoolAlgData( byte		newValue )
		{
			m_boolAlgData = newValue;
		}


		const std::array<Vptr,2>&			verts() const
		{
			return( m_verts );
		}

		std::array<Vptr,2>&					verts()
		{
			return( m_verts );
		}


		const TopoTrianglePointerList&		triangles() const
		{
			return( m_tris );
		}

		TopoTrianglePointerList&			triangles()
		{
			return( m_tris );
		}

		Cork::Math::BBox3D					boundingBox() const
		{
			const Cork::Math::Vector3D& p0 = *(m_verts[0]->quantizedValue());
			const Cork::Math::Vector3D& p1 = *(m_verts[1]->quantizedValue());

			return Cork::Math::BBox3D( min(p0, p1), max(p0, p1) );
		}


		operator Empty3d::EdgeIn() const
		{
			return( Empty3d::EdgeIn( *( m_verts[0]->quantizedValue() ), *( m_verts[1]->quantizedValue() ) ));
		}


		GMPExt4::GmpExt4_2		edgeExactCoordinates() const
		{
			GMPExt4::GmpExt4_1		ep[2];

			Empty3d::toGmpExt( ep[0], *( m_verts[0]->quantizedValue() ) );
			Empty3d::toGmpExt( ep[1], *( m_verts[1]->quantizedValue() ) );

			// construct geometry

			GMPExt4::GmpExt4_2		value;

			join( value, ep[0], ep[1] );

			return( value );
		}


	private :

		void*										m_data;						// algorithm specific handle

		byte										m_boolAlgData;

		std::array<Vptr,2>							m_verts;					// endpoint vertices
		TopoTrianglePointerList						m_tris;						// incident triangles
	};


	typedef ManagedIntrusiveValueList<TopoEdge>							TopoEdgeList;

	typedef boost::container::small_vector<const TopoEdge*,24>			TopoEdgePointerVector;




	class TopoTri final : public boost::noncopyable, public IntrusiveListHookNoDestructorOnElements
	{
	public :

		TopoTri()
		{}

		explicit
		TopoTri( IndexType		ref )
			: m_ref( ref )
		{}


		TopoTri( IndexType		ref,
				 TopoVert&		vertex0,
				 TopoVert&		vertex1,
				 TopoVert&		vertex2 )
			: m_ref( ref )
		{
			m_verts[0] = &vertex0;
			m_verts[1] = &vertex1;
			m_verts[2] = &vertex2;

			vertex0.triangles().insert( this );
			vertex1.triangles().insert( this );
			vertex2.triangles().insert( this );
		}


		TopoTri(const TopoTri&) = delete;

		~TopoTri()
		{}


		IndexType					ref() const
		{
			return( m_ref );
		}

		void						setRef( IndexType		newValue )
		{
			m_ref = newValue;
		}


		void*						data() const
		{
			return( m_data );
		}

		void						setData( void*		newValue )
		{
			m_data = newValue;
		}

#ifndef CORK_SSE
		//	Without SSE, the min/max computations as slow enough that caching the computed value is most efficient

		const Cork::Math::BBox3D&	boundingBox() const
		{
			if( m_boundingBox.is_initialized() )
			{
				return( m_boundingBox.get() );
			}

			const Cork::Math::Vector3D& p0 = *( m_verts[0]->quantizedValue() );
			const Cork::Math::Vector3D& p1 = *( m_verts[1]->quantizedValue() );
			const Cork::Math::Vector3D& p2 = *( m_verts[2]->quantizedValue() );

			const_cast<boost::optional<Cork::Math::BBox3D>&>(m_boundingBox).emplace( min( p0, p1, p2 ), max( p0, p1, p2 ) );

			return( m_boundingBox.get() );
		}
#else
		//	With SSE, the min/max functions and bounding box computation is quick enough that computing the
		//		value every time is actually most efficient.

		const Cork::Math::BBox3D	boundingBox() const
		{
			const Cork::Math::Vector3D& p0 = *( m_verts[0]->quantizedValue() );
			const Cork::Math::Vector3D& p1 = *( m_verts[1]->quantizedValue() );
			const Cork::Math::Vector3D& p2 = *( m_verts[2]->quantizedValue() );

			return( Cork::Math::BBox3D( min( p0, p1, p2 ), max( p0, p1, p2 ) ));
		}
#endif

		const GMPExt4::GmpExt4_3		triangleExactCoordinates() const
		{
			GMPExt4::GmpExt4_3		value;

			GMPExt4::GmpExt4_1		p[3];
		
			Empty3d::toGmpExt( p[0], *( m_verts[0]->quantizedValue() ) );
			Empty3d::toGmpExt( p[1], *( m_verts[1]->quantizedValue() ) );
			Empty3d::toGmpExt( p[2], *( m_verts[2]->quantizedValue() ) );

			GMPExt4::GmpExt4_2		temp;

			join( temp, p[0], p[1] );
			join( value, temp, p[2] );

			return( value );
		}



		byte						boolAlgData() const
		{
			return( m_boolAlgData );
		}

		void						setBoolAlgData( byte		newValue )
		{
			m_boolAlgData = newValue;
		}

	
		void						setVertices( std::array<TopoVert*,3>&		vertices )
		{
			memcpy( &m_verts, &vertices, sizeof( std::array<TopoVert*,3> ));

			m_verts[0]->triangles().insert( this );
			m_verts[1]->triangles().insert( this );
			m_verts[2]->triangles().insert( this );

#ifndef CORK_SSE
			m_boundingBox.reset();
#endif
		}

		const std::array<Vptr,3>&	verts() const
		{
			return( m_verts );
		}

		void						setEdges( std::array<TopoEdge*,3>&		edges )
		{
			memcpy( &m_edges, &edges, sizeof( std::array<TopoEdge*,3> ));

			edges[0]->triangles().insert( this );
			edges[1]->triangles().insert( this );
			edges[2]->triangles().insert( this );
		}
	
		const std::array<Eptr,3>&	edges() const
		{
			return( m_edges );
		}


		void						flip()
		{
			std::swap( m_verts[0], m_verts[1] );
			std::swap( m_edges[0], m_edges[1]);
		}

		void						AssignEdges( TopoVert*			v0,
												 TopoVert*			v1,
												 TopoVert*			v2,
												 TopoEdge*			edge01,
												 TopoEdge*			edge02,
												 TopoEdge*			edge12 )
		{

			if(( v0 != m_verts[0] ) && ( v1 != m_verts[0] ))
			{
				m_edges[0] = edge01;

				if(( v0 != m_verts[1] ) && ( v2 != m_verts[1] ))
				{
					m_edges[1] = edge02;
					m_edges[2] = edge12;
				}
				else
				{
					m_edges[2] = edge02;
					m_edges[1] = edge12;
				}

			}
			else if(( v0 != m_verts[1] ) && ( v1 != m_verts[1] ))
			{
				m_edges[1] = edge01;
				
				if(( v0 != m_verts[0] ) && ( v2 != m_verts[0] ))
				{
					m_edges[0] = edge02;
					m_edges[2] = edge12;
				}
				else
				{
					m_edges[2] = edge02;
					m_edges[0] = edge12;
				}
			}
			else if(( v0 != m_verts[2] ) && ( v1 != m_verts[2] ))
			{
				m_edges[2] = edge01;
			
				if(( v0 != m_verts[0] ) && ( v2 != m_verts[0] ))
				{
					m_edges[0] = edge02;
					m_edges[1] = edge12;
				}
				else
				{
					m_edges[1] = edge02;
					m_edges[0] = edge12;
				}
			}
		}


		bool		hasCommonVertex( const TopoTri&		triToCheck ) const
		{
			return(( m_verts[0] == triToCheck.m_verts[0] ) ||
				   ( m_verts[0] == triToCheck.m_verts[1] ) ||
				   ( m_verts[0] == triToCheck.m_verts[2] ) ||
				   ( m_verts[1] == triToCheck.m_verts[0] ) ||
				   ( m_verts[1] == triToCheck.m_verts[1] ) ||
				   ( m_verts[1] == triToCheck.m_verts[2] ) ||
				   ( m_verts[2] == triToCheck.m_verts[0] ) ||
				   ( m_verts[2] == triToCheck.m_verts[1] ) ||
				   ( m_verts[2] == triToCheck.m_verts[2] ));
		}



		bool	findCommonVertex( const TopoTri&		triToCheck,
								  TopoVert*&			commonVertex ) const
		{
			for (uint i = 0; i<3; i++)
			{
				for (uint j = 0; j<3; j++)
				{
					if (m_verts[i] == triToCheck.m_verts[j])
					{
						commonVertex = m_verts[i];
						return( true );
					}
				}
			}

			commonVertex = nullptr;

			return( false );
		}


		bool		hasCommonVertex( const TopoEdge&		edgeToCheck ) const
		{
			return (( m_verts[0] == edgeToCheck.verts()[0] ) ||
					( m_verts[1] == edgeToCheck.verts()[0] ) ||
					( m_verts[2] == edgeToCheck.verts()[0] ) ||
					( m_verts[0] == edgeToCheck.verts()[1] ) ||
					( m_verts[1] == edgeToCheck.verts()[1] ) ||
					( m_verts[2] == edgeToCheck.verts()[1] ));
		}


		operator Empty3d::TriIn() const
		{
			return( Empty3d::TriIn( *( m_verts[0]->quantizedValue() ), *( m_verts[1]->quantizedValue() ), *( m_verts[2]->quantizedValue() ) ) );
		}


		bool intersectsEdge( const TopoEdge&						edgeToCheck,
							 Empty3d::ExactArithmeticContext&		arithContext ) const
		{
			// must check whether the edge and triangle share a vertex
			// if so, then trivially we know they intersect in exactly that vertex
			// so we discard this case from consideration.

			if (hasCommonVertex(edgeToCheck))
			{
				return(false);
			}

			Empty3d::TriEdgeIn		input( this->operator Empty3d::TriIn() , edgeToCheck.operator Empty3d::EdgeIn() );

			return( !input.emptyExact( arithContext ) );
		}



	private :

		IndexType									m_ref;				// index to actual data
		void*										m_data;				// algorithm specific handle

		byte										m_boolAlgData;

		std::array<Vptr,3>							m_verts;			// vertices of this triangle
		std::array<Eptr,3>							m_edges;			// edges of this triangle opposite to the given vertex

#ifndef CORK_SSE
		boost::optional<Cork::Math::BBox3D>			m_boundingBox;
#endif
	};



	typedef ManagedIntrusiveValueList<TopoTri>			TopoTriList;







	class TopoCacheWorkspace
	{
	public :

		typedef std::array<tbb::spin_mutex,MAX_TRIANGLES_IN_DISJOINT_UNION * 3>			MutexArray;

		TopoCacheWorkspace()
		{
			m_vertexListPool.reserve(100000);
			m_edgeListPool.reserve(100000);
			m_triListPool.reserve(100000);
		}

		virtual ~TopoCacheWorkspace()
		{}



		void	reset()
		{
			m_vertexListPool.clear();
			m_edgeListPool.clear();
			m_triListPool.clear();
		}


		operator			TopoVertexList::PoolType&()
		{
			return( m_vertexListPool );
		}

		operator			TopoEdgeList::PoolType&()
		{
			return( m_edgeListPool );
		}

		operator			TopoTriList::PoolType&()
		{
			return( m_triListPool );
		}


		MutexArray&			getVertexMutexes()
		{
			return( m_vertexMutexes );
		}

		MutexArray&			getEdgeAcceleratorMutexes()
		{
			return( m_edgeAcceleratorMutexes );
		}


	private :

		TopoVertexList::PoolType		m_vertexListPool;
		TopoEdgeList::PoolType			m_edgeListPool;
		TopoTriList::PoolType			m_triListPool;

		MutexArray						m_vertexMutexes;
		MutexArray						m_edgeAcceleratorMutexes;
	};








	class TopoCache : public boost::noncopyable
	{
	public :

		TopoCache( MeshBase&					owner,
				   TopoCacheWorkspace&			workspace );

		virtual ~TopoCache()
		{}
    
    
		// until commit() is called, the Mesh::verts and Mesh::tris
		// arrays will still contain garbage entries
    
		void commit();
    
		bool isValid();
		void print();



		MeshBase&						ownerMesh()
		{
			return(m_mesh);
		}

		const MeshBase&					ownerMesh() const
		{
			return(m_mesh);
		}


		const TopoTriList&				triangles() const
		{
			return( m_topoTriList );
		}

		TopoTriList&					triangles()
		{
			return( m_topoTriList );
		}

		const TopoEdgeList&				edges() const
		{
			return( m_topoEdgeList );
		}
    
		TopoEdgeList&					edges()
		{
			return( m_topoEdgeList );
		}

		const TopoVertexList&			vertices() const
		{
			return( m_topoVertexList );
		}

		TopoVertexList&					vertices()
		{
			return( m_topoVertexList );
		}


		// helpers to create bits and pieces

		Vptr TopoCache::newVert()
		{
			size_t        ref = m_meshVertices.size();
                
			m_meshVertices.emplace_back();

			return( m_topoVertexList.emplace_back( ref ) );
		}


		Eptr TopoCache::newEdge()
		{
			return( m_topoEdgeList.emplace_back() );
		}


		Tptr TopoCache::newTri()
		{
			IndexType        ref = m_meshTriangles.size();
    
			m_meshTriangles.push_back( CorkTriangle() );

			return( m_topoTriList.emplace_back( ref ) );
		}


		 // helpers to release bits and pieces

		void TopoCache::freeVert(Vptr v)
		{
			m_topoVertexList.free( v );
		}

		void TopoCache::freeEdge(Eptr e)
		{
			m_topoEdgeList.free( e );
		}

		void TopoCache::freeTri(Tptr t)
		{
			m_topoTriList.free( t );
		}



		// helper to delete geometry in a structured way


		void TopoCache::deleteTri(Tptr tri)
		{
			// first, unhook the triangle from its faces
    
			for(uint k=0; k<3; k++)
			{
				tri->verts()[k]->triangles().erase( tri );

				tri->edges()[k]->triangles().erase( tri );
			}

			// now, let's check for any edges which no longer border triangles
    
			for(uint k=0; k<3; k++)
			{
				Eptr e = tri->edges()[k];

				if(e->triangles().empty())
				{
					//	Unhook the edge from its vertices and delete it

					e->verts()[0]->edges().erase( e );
					e->verts()[1]->edges().erase( e );

					freeEdge(e);
				}
			}

			// now, let's check for any vertices which no longer border triangles

			for(uint k=0; k<3; k++)
			{
				Vptr            v  = tri->verts()[k];

				if(v->triangles().empty())
				{
					freeVert(v);
				}
			}

			// finally, release the triangle
    
			freeTri(tri);
		}


		// helper to flip triangle orientation

		void TopoCache::flipTri(Tptr t)
		{
			t->flip();
			m_meshTriangles[t->ref()].flip();
		}

    

	private:

		//	Data Members

		MeshBase&							m_mesh;
		TopoCacheWorkspace&					m_workspace;

		MeshBase::TriangleVector&			m_meshTriangles;
		MeshBase::VertexVector&				m_meshVertices;

		TopoVertexList						m_topoVertexList;
		TopoEdgeList						m_topoEdgeList;
		TopoTriList							m_topoTriList;

		//	Methods

		void								init();
	};







	// support structure for cache construction

	

	class TopoEdgePrototype : public SEFUtility::SparseVectorEntry
	{
	public :

		TopoEdgePrototype( IndexType	v )
			: SparseVectorEntry( v ),
			  m_edge( nullptr )
		{}


		IndexType			vid() const
		{
			return( index() );
		}


		TopoEdge*		edge()
		{
			return( m_edge );
		}

		TopoEdge*			setEdge( TopoEdge*		edge )
		{
			m_edge = edge;

			return( m_edge );
		}

	private :

		TopoEdge*		m_edge;
	};



	typedef SEFUtility::SparseVector<TopoEdgePrototype,10>		TopoEdgePrototypeVector;




	std::ostream& operator<<(std::ostream &out, const TopoVert& vertex);
	std::ostream& operator<<(std::ostream &out, const TopoEdge& edge);
	std::ostream& operator<<(std::ostream &out, const TopoTri& tri);

}





