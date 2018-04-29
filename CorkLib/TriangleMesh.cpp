// +-------------------------------------------------------------------------
// | TriangleMesh.cpp
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



#include "TriangleMesh.h"

#include <unordered_map>
#include <map>

#include <boost\align\aligned_allocator.hpp>

#include ".\Statistics\StatsImpl.h"



namespace Cork
{
	

	//
	//	TriangleMeshImpl is a straightforward implementation of a triangularized solid expressed as a set
	//		of vertices and triangles defined as 3-tuples of indices into the vertex set.
	//

	class TriangleMeshImpl : public TriangleMesh
	{
	public:


		TriangleMeshImpl( std::shared_ptr<const std::vector<TriangleByIndices>>&		triangles,
						  std::shared_ptr<VertexVector>&								vertices,
						  const Cork::Math::BBox3D&										boundingBox )
			: m_triangles( triangles ),
			  m_vertices( vertices ),
			  m_boundingBox( make_aligned<Cork::Math::BBox3D>( boundingBox ))
		{}


		size_t			numTriangles() const
		{
			return(m_triangles->size());
		}

		size_t			numVertices() const
		{
			return(m_vertices->size());
		}



		const VertexVector&								vertices() const
		{
			return(*m_vertices);
		}

		const std::vector<TriangleByIndices>&			triangles() const
		{
			return(*m_triangles);
		}

		TriangleByVertices								triangleByVertices( const TriangleByIndices&		triangleByIndices ) const
		{
			return(TriangleByVertices( (*m_vertices)[triangleByIndices[0]],
									   (*m_vertices)[triangleByIndices[1]],
									   (*m_vertices)[triangleByIndices[2]] ));
		}


		const Cork::Math::BBox3D&						boundingBox() const
		{
			return( *m_boundingBox );
		}

		
		Statistics::GeometricStatistics					ComputeGeometricStatistics() const
		{
			Statistics::GeometricStatisticsEngine		geometricStatsEngine;
			
			for( auto& currentTriangle : *m_triangles )
			{
				geometricStatsEngine.AddTriangle( triangleByVertices( currentTriangle ) );
			}

			return( Statistics::GeometricStatistics( m_vertices->size(),
													 geometricStatsEngine.numTriangles(),
													 geometricStatsEngine.area(),
													 geometricStatsEngine.volume(),
													 geometricStatsEngine.minEdgeLength(),
													 geometricStatsEngine.maxEdgeLength(),
													 geometricStatsEngine.boundingBox() ));
		}


		Statistics::TopologicalStatistics			ComputeTopologicalStatistics() const
		{
			Statistics::TopologicalStatisticsEngine			statsEngine( m_triangles->size() );


			for( auto& currentTriangle : *m_triangles )
			{
				statsEngine.AddTriangle( currentTriangle );
			}

			return( statsEngine.Analyze() );
		}



	private:

		std::shared_ptr<const std::vector<TriangleByIndices>>		m_triangles;
		std::shared_ptr<VertexVector>								m_vertices;

		aligned_unique_ptr<Cork::Math::BBox3D>						m_boundingBox;
	};



	//
	//	IncrementalVertexIndexTriangleMeshBuilderImpl implements an incremental triangle mesh builder
	//		which can be used to construct a triangle mesh from a list of vertices and 
	//		triangles assembled from those vertices.
	//
	//	This class also uses copy-on-write semantics for the internal dtata structures that are eventually
	//		shared with the TriangleMeshImpl class.  If additional vertices or triangles are added after
	//		generating a mesh, then the internal data structures are cloned such that the TriangleMeshImpl
	//		will then have a std::shared_ptr to the original data structures and this class will have new
	//		data structures that it uniquely owns.  This permits the TriangleMesh instances to be const without
	//		incurring the overhead of copying the data structures for the majority of use cases.
	//

	class IncrementalVertexIndexTriangleMeshBuilderImpl : public IncrementalVertexIndexTriangleMeshBuilder
	{
	public:

		IncrementalVertexIndexTriangleMeshBuilderImpl( size_t		numVertices,
													   size_t		numTriangles )
			: m_vertices(),
			  m_indexedVertices( new TriangleMesh::VertexVector() ),
			  m_triangles( new std::vector<TriangleMesh::TriangleByIndices>() ),
			  m_boundingBox( make_aligned<Cork::Math::BBox3D>( Cork::Math::Vector3D( NUMERIC_PRECISION_MAX, NUMERIC_PRECISION_MAX, NUMERIC_PRECISION_MAX ), Cork::Math::Vector3D( NUMERIC_PRECISION_MIN, NUMERIC_PRECISION_MIN, NUMERIC_PRECISION_MIN )  ))
		{
			if ( numVertices > 0 )
			{
				m_indexedVertices->reserve( numVertices );
				m_vertexIndexRemapper.reserve(numVertices);
			}

			if ( numTriangles > 0 )
			{
				m_triangles->reserve( numTriangles );
			}
		};

		~IncrementalVertexIndexTriangleMeshBuilderImpl() {};



		const Cork::Math::BBox3D&			boundingBox() const
		{
			return( *m_boundingBox );
		}

		TriangleMesh::VertexIndexType		AddVertex( const TriangleMesh::Vertex&					vertexToAdd )
		{
			//	Copy on write for the vertex structure.  We need to duplicate the vector if we no longer hold the pointer uniquely.

			if ( !m_indexedVertices.unique() )
			{
				m_indexedVertices = std::shared_ptr<TriangleMesh::VertexVector>( new TriangleMesh::VertexVector( *m_indexedVertices ) );
			}

			//	Add the vertex, de-duplicate on the fly.

			TriangleMesh::VertexMap::const_iterator	vertexLoc = m_vertices.find(vertexToAdd);
			
			if (vertexLoc == m_vertices.end())
			{
				//	Vertex is new, update all data structures

				m_vertices[vertexToAdd] = m_indexedVertices->size();
				m_vertexIndexRemapper.push_back(m_indexedVertices->size());
				m_indexedVertices->push_back(vertexToAdd);
			}
			else
			{
				//	Vertex is a duplicate, so remap to it

				m_vertexIndexRemapper.push_back(vertexLoc->second);
			}

			//	The index we return should always be the remapper size minus 1
			
			return(m_vertexIndexRemapper.size() - 1);
		}

		TriangleMeshBuilderResultCodes		AddTriangle( const TriangleMesh::TriangleByIndices&		triangleToAdd )
		{
			//	Insure the indices are in bounds

			if ((triangleToAdd[0] >= m_vertexIndexRemapper.size()) ||
				(triangleToAdd[1] >= m_vertexIndexRemapper.size()) ||
				(triangleToAdd[2] >= m_vertexIndexRemapper.size()))
			{
				return(TriangleMeshBuilderResultCodes::VERTEX_INDEX_OUT_OF_BOUNDS);
			}

			//	Copy on write for the triangle and edge incidence structures.  We need to duplicate them if we no longer hold the pointers uniquely.

			if ( !m_triangles.unique() )
			{
				m_triangles = std::shared_ptr<std::vector<TriangleMesh::TriangleByIndices>>( new std::vector<TriangleMesh::TriangleByIndices>( *m_triangles ) );
			}

			//	Remap the triangle indices

			TriangleMesh::TriangleByIndices		remappedTriangle( m_vertexIndexRemapper[triangleToAdd[0]],
																  m_vertexIndexRemapper[triangleToAdd[1]],
																  m_vertexIndexRemapper[triangleToAdd[2]] );

			//	Add the triangle to the vector

			m_triangles->push_back( remappedTriangle );

			//	Update the bounding box

			TriangleMesh::TriangleByVertices		triByVerts( (*m_indexedVertices)[remappedTriangle[0]],
																(*m_indexedVertices)[remappedTriangle[1]],
																(*m_indexedVertices)[remappedTriangle[2]] );

			NUMERIC_PRECISION	minX = std::min( (NUMERIC_PRECISION)std::min(triByVerts[0].x(), std::min(triByVerts[1].x(), triByVerts[2].x())), boundingBox().minima().x() );
			NUMERIC_PRECISION	minY = std::min( (NUMERIC_PRECISION)std::min(triByVerts[0].y(), std::min(triByVerts[1].y(), triByVerts[2].y())), boundingBox().minima().y() );
			NUMERIC_PRECISION	minZ = std::min( (NUMERIC_PRECISION)std::min(triByVerts[0].z(), std::min(triByVerts[1].z(), triByVerts[2].z())), boundingBox().minima().z() );

			NUMERIC_PRECISION	maxX = std::max( (NUMERIC_PRECISION)std::max(triByVerts[0].x(), std::max(triByVerts[1].x(), triByVerts[2].x())), boundingBox().maxima().x() );
			NUMERIC_PRECISION	maxY = std::max( (NUMERIC_PRECISION)std::max(triByVerts[0].y(), std::max(triByVerts[1].y(), triByVerts[2].y())), boundingBox().maxima().y() );
			NUMERIC_PRECISION	maxZ = std::max( (NUMERIC_PRECISION)std::max(triByVerts[0].z(), std::max(triByVerts[1].z(), triByVerts[2].z())), boundingBox().maxima().z() );

			m_boundingBox = make_aligned<Cork::Math::BBox3D>( Cork::Math::Vector3D( minX, minY, minZ ), Cork::Math::Vector3D( maxX, maxY, maxZ ) );

			//	All is well if we made it here

			return(TriangleMeshBuilderResultCodes::SUCCESS);
		}


		std::unique_ptr<TriangleMesh>		Mesh()
		{
			return(std::unique_ptr<TriangleMesh>( new  TriangleMeshImpl( std::const_pointer_cast<const std::vector<TriangleMesh::TriangleByIndices>>(m_triangles),
																		 m_indexedVertices,
																		 boundingBox() ) ));
		}


	private:

		TriangleMesh::VertexMap													m_vertices;
		std::shared_ptr<TriangleMesh::VertexVector>								m_indexedVertices;
		std::vector<TriangleMesh::VertexIndexType>								m_vertexIndexRemapper;

		std::shared_ptr<std::vector<TriangleMesh::TriangleByIndices>>			m_triangles;

		aligned_unique_ptr<Cork::Math::BBox3D>									m_boundingBox;
	};


	//
	//	Factory method to return an incremental triangle mesh builder
	//

	std::unique_ptr<IncrementalVertexIndexTriangleMeshBuilder>		IncrementalVertexIndexTriangleMeshBuilder::GetBuilder( size_t		numVertices,
																														   size_t		numTriangles )
	{
		return(std::unique_ptr<IncrementalVertexIndexTriangleMeshBuilder>( new IncrementalVertexIndexTriangleMeshBuilderImpl( numVertices, numTriangles ) ));
	}


}	//	namespace Cork

