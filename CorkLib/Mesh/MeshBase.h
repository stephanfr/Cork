// +-------------------------------------------------------------------------
// | MeshBase.h
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


#include <functional>
#include <vector>

#include <boost/align/aligned_allocator.hpp>
#include <boost/optional.hpp>

#include "../Util/AlignedUniquePtr.h"

#include "../Math/Primitives.h"

#include "../Intersection/quantization.h"

#include "../TriangleMesh.h"

#include "../cork.h"



namespace Cork
{
	
	class SolverPerfStats : public SolverPerformanceStatisticsIfx
	{
	public :

		SolverPerfStats()
			: m_numberOfTrianglesInDisjointUnion(0),
			  m_numberOfTrianglesInFinalMesh(0),
			  m_elapsedCPUTimeInNanoSeconds(0),
			  m_elapsedWallTimeInNanoSeconds(0),
			  m_startingVirtualMemorySizeInMB(0),
			  m_endingVirtualMemorySizeInMB(0)
		{}


		unsigned long			numberOfTrianglesInDisjointUnion() const
		{
			return( m_numberOfTrianglesInDisjointUnion );
		}

		unsigned long			numberOfTrianglesInFinalMesh() const
		{
			return( m_numberOfTrianglesInFinalMesh );
		}

		unsigned long long		elapsedCPUTimeInNanoSeconds() const
		{
			return( m_elapsedCPUTimeInNanoSeconds );
		}

		unsigned long long		elapsedWallTimeInNanoSeconds() const
		{
			return( m_elapsedWallTimeInNanoSeconds );
		}

		unsigned long			startingVirtualMemorySizeInMB() const
		{
			return( m_startingVirtualMemorySizeInMB );
		}

		unsigned long			endingVirtualMemorySizeInMB() const
		{
			return( m_endingVirtualMemorySizeInMB );
		}

		void		setNumberOfTrianglesInDisjointUnion( unsigned long			numberOfTrianglesInDisjointUnion )
		{
			m_numberOfTrianglesInDisjointUnion = numberOfTrianglesInDisjointUnion;
		}

		void		setNumberOfTrianglesInFinalMesh( unsigned long				numberOfTrianglesInFinalMesh ) 
		{
			m_numberOfTrianglesInFinalMesh = numberOfTrianglesInFinalMesh;
		}

		void		setElapsedCPUTimeInNanoSeconds( unsigned long long				elapsedCPUTimeInNanoSeconds )
		{
			m_elapsedCPUTimeInNanoSeconds = elapsedCPUTimeInNanoSeconds;
		}

		void		setElapsedWallTimeInNanoSeconds( unsigned long long				elapsedWallTimeInNanoSeconds )
		{
			m_elapsedWallTimeInNanoSeconds = elapsedWallTimeInNanoSeconds;
		}

		void		setStartingVirtualMemorySizeInMB( unsigned long				startingVirtualMemorySizeInMB ) 
		{
			m_startingVirtualMemorySizeInMB = startingVirtualMemorySizeInMB;
		}

		void		setEndingVirtualMemorySizeInMB( unsigned long				endingVirtualMemorySizeInMB ) 
		{
			m_endingVirtualMemorySizeInMB = endingVirtualMemorySizeInMB;
		}


	private :

		unsigned long			m_numberOfTrianglesInDisjointUnion;
		unsigned long			m_numberOfTrianglesInFinalMesh;

		unsigned long long		m_elapsedCPUTimeInNanoSeconds;
		unsigned long long		m_elapsedWallTimeInNanoSeconds;

		unsigned long			m_startingVirtualMemorySizeInMB;
		unsigned long			m_endingVirtualMemorySizeInMB;

	};


	class CorkTriangle : public Cork::Math::TriangleByIndicesBase
	{
	public :

		CorkTriangle()
		{}
/*
		CorkTriangle( const CorkTriangle&		triangleToCopy )
			: Cork::Math::TriangleByIndicesBase( (const Cork::Math::TriangleByIndicesBase&)triangleToCopy ),
			  m_boolAlgData( triangleToCopy.m_boolAlgData )
		{
			memcpy( &m_a, &triangleToCopy.m_a, sizeof( unsigned int ) * 3 );
		}
*/
		CorkTriangle( const Cork::TriangleMesh::TriangleByIndices&		triangleToCopy,
					  byte												boolAlgData	)
			: Cork::Math::TriangleByIndicesBase( triangleToCopy ),
			  m_boolAlgData( boolAlgData )
		{}




		const byte			boolAlgData() const
		{
			return( m_boolAlgData );
		}

		byte&				boolAlgData()
		{
			return( m_boolAlgData );
		}

		void				flip()
		{
			std::swap( m_a, m_b );
		}

		void				offsetIndices( size_t		offsetValue )
		{
			m_a += offsetValue;
			m_b += offsetValue;
			m_c += offsetValue;
		}


	private :

		byte					m_boolAlgData; // internal use by algorithm - value must be copied when the triangle is subdivided
	};


	typedef Cork::Math::Vector3D				CorkVertex;


	



	class MeshBase
	{
	public :

		typedef std::vector<CorkTriangle>					TriangleVector;
		typedef Cork::Math::Vertex3DVector					VertexVector;



		MeshBase()
		{}

		MeshBase( const MeshBase&				meshBaseToCopy,
				  const SolverControlBlock&		controlBlock )
			: m_boundingBox( make_aligned<Cork::Math::BBox3D>( *(meshBaseToCopy.m_boundingBox) )),
			  m_tris( meshBaseToCopy.m_tris ),
			  m_verts( meshBaseToCopy.m_verts ),
			  m_controlBlock( controlBlock )
		{}

		virtual ~MeshBase()
		{}


		const SolverControlBlock&		solverControlBlock() const
		{
			return( m_controlBlock.get() );
		}

		TriangleVector&				triangles()
		{
			return( m_tris );
		}

		const TriangleVector&		triangles() const
		{
			return( m_tris );
		}

		VertexVector&				vertices()
		{
			return( m_verts );
		}

		const VertexVector&			vertices() const
		{
			return( m_verts );
		}

		const Cork::Math::BBox3D&	boundingBox() const
		{
			return( *m_boundingBox );
		}

		const Cork::Quantization::Quantizer		getQuantizer() const
		{
			//	Calibrate the quantization unit...

			NUMERIC_PRECISION		maxMag = NUMERIC_PRECISION_MIN;

			for (const CorkVertex &v : vertices())
			{
				maxMag = std::max(maxMag, v.abs().max() );
			}

			//	Find the minimum edge length across all the triangles

			NUMERIC_PRECISION		minEdgeLengthSquared = NUMERIC_PRECISION_MAX;
			NUMERIC_PRECISION		maxEdgeLengthSquared = NUMERIC_PRECISION_MIN;

			for (auto& currentTriangle : triangles())
			{
				const Cork::Math::Vector3D&	vert0(vertices()[currentTriangle.a()]);
				const Cork::Math::Vector3D&	vert1(vertices()[currentTriangle.b()]);
				const Cork::Math::Vector3D&	vert2(vertices()[currentTriangle.c()]);

				minEdgeLengthSquared = std::min(minEdgeLengthSquared, std::min((vert0 - vert1).len_squared(), std::min((vert0 - vert2).len_squared(), (vert1 - vert2).len_squared())));
				maxEdgeLengthSquared = std::max(maxEdgeLengthSquared, std::max((vert0 - vert1).len_squared(), std::max((vert0 - vert2).len_squared(), (vert1 - vert2).len_squared())));
			}

			return( Quantization::Quantizer(maxMag, sqrt( minEdgeLengthSquared )));
		}

		void for_raw_tris( std::function<void( IndexType, IndexType, IndexType )> func )
		{
			for ( auto& tri : m_tris )
			{
				func( tri.a(), tri.b(), tri.c() );
			}
		}

		void for_raw_tris( std::function<void( IndexType, IndexType, IndexType )> func ) const
		{
			for ( auto& tri : m_tris )
			{
				func( tri.a(), tri.b(), tri.c() );
			}
		}


	protected :

		TriangleVector								m_tris;
		VertexVector								m_verts;

		aligned_unique_ptr<Cork::Math::BBox3D>		m_boundingBox;

		boost::optional<SolverControlBlock>			m_controlBlock;

		SolverPerfStats								m_performanceStats;
	};

}	//	namesapce Cork