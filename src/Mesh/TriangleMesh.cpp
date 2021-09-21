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

#include <boost/align/aligned_allocator.hpp>
#include <map>
#include <unordered_map>

#include "mesh/triangle_mesh.h"
#include "statistics/statistics_impl.h"

namespace Cork
{
    //
    //	TriangleMeshImpl is a straightforward implementation of a triangularized solid expressed as a set
    //		of vertices and triangles defined as 3-tuples of indices into the vertex set.
    //

    class TriangleMeshImpl : public TriangleMesh
    {
       public:
        TriangleMeshImpl(std::shared_ptr<const std::vector<TriangleByIndices>>& triangles,
                         std::shared_ptr<VertexVector>& vertices, const Cork::Math::BBox3D& boundingBox)
            : m_triangles(triangles),
              m_vertices(vertices),
              m_boundingBox(std::make_unique<Cork::Math::BBox3D>(boundingBox))
        {
        }

        [[nodiscard]] size_t numTriangles() const final { return (m_triangles->size()); }

        [[nodiscard]] size_t numVertices() const final { return (m_vertices->size()); }

        [[nodiscard]] const VertexVector& vertices() const final { return (*m_vertices); }

        [[nodiscard]] const std::vector<TriangleByIndices>& triangles() const final { return (*m_triangles); }

        [[nodiscard]] TriangleByVertices triangleByVertices(const TriangleByIndices& triangleByIndices) const final
        {
            return (TriangleByVertices((*m_vertices)[triangleByIndices[0]], (*m_vertices)[triangleByIndices[1]],
                                       (*m_vertices)[triangleByIndices[2]]));
        }

        [[nodiscard]] const Cork::Math::BBox3D& boundingBox() const final { return (*m_boundingBox); }

        [[nodiscard]] Statistics::GeometricStatistics ComputeGeometricStatistics() const final
        {
            Statistics::GeometricStatisticsEngine geometricStatsEngine;

            for (const auto& currentTriangle : *m_triangles)
            {
                geometricStatsEngine.AddTriangle(triangleByVertices(currentTriangle));
            }

            return (Statistics::GeometricStatistics(
                m_vertices->size(), geometricStatsEngine.numTriangles(), geometricStatsEngine.area(),
                geometricStatsEngine.volume(), geometricStatsEngine.minEdgeLength(),
                geometricStatsEngine.maxEdgeLength(), geometricStatsEngine.boundingBox()));
        }

        [[nodiscard]] Statistics::TopologicalStatistics ComputeTopologicalStatistics() const final
        {
            Statistics::TopologicalStatisticsEngine statsEngine(m_triangles->size());

            for (const auto& currentTriangle : *m_triangles)
            {
                statsEngine.AddTriangle(currentTriangle);
            }

            return (statsEngine.Analyze());
        }

       private:
        std::shared_ptr<const std::vector<TriangleByIndices>> m_triangles;
        std::shared_ptr<VertexVector> m_vertices;

        std::unique_ptr<Cork::Math::BBox3D> m_boundingBox;
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
        IncrementalVertexIndexTriangleMeshBuilderImpl(size_t numVertices, size_t numTriangles)
            : m_indexedVertices(new VertexVector()),
              m_triangles(new std::vector<TriangleByIndices>()),
              m_boundingBox(std::make_unique<Cork::Math::BBox3D>(
                  Cork::Math::Vector3D(NUMERIC_PRECISION_MAX, NUMERIC_PRECISION_MAX, NUMERIC_PRECISION_MAX),
                  Cork::Math::Vector3D(NUMERIC_PRECISION_MIN, NUMERIC_PRECISION_MIN, NUMERIC_PRECISION_MIN)))
        {
            if (numVertices > 0)
            {
                m_indexedVertices->reserve(numVertices);
                m_vertexIndexRemapper.reserve(numVertices);
            }

            if (numTriangles > 0)
            {
                m_triangles->reserve(numTriangles);
            }
        };

        ~IncrementalVertexIndexTriangleMeshBuilderImpl() = default;		//	NOLINT

        IncrementalVertexIndexTriangleMeshBuilderImpl() = delete;
        IncrementalVertexIndexTriangleMeshBuilderImpl(const IncrementalVertexIndexTriangleMeshBuilderImpl&) = delete;
        IncrementalVertexIndexTriangleMeshBuilderImpl(const IncrementalVertexIndexTriangleMeshBuilderImpl&&) = delete;

        IncrementalVertexIndexTriangleMeshBuilderImpl& operator=(const IncrementalVertexIndexTriangleMeshBuilderImpl&) =
            delete;
        IncrementalVertexIndexTriangleMeshBuilderImpl& operator=(
            const IncrementalVertexIndexTriangleMeshBuilderImpl&&) = delete;

        [[nodiscard]] const Cork::Math::BBox3D& boundingBox() const { return (*m_boundingBox); }

        [[nodiscard]] size_t num_vertices() const final { return m_indexedVertices->size(); }

        VertexIndexType AddVertex(const Vertex& vertexToAdd) final
        {
            //	Copy on write for the vertex structure.  We need to duplicate the vector if we no longer hold the
            //pointer uniquely.

            if (!m_indexedVertices.unique())
            {
                m_indexedVertices = std::make_shared<VertexVector>(*m_indexedVertices);
            }

            //	Add the vertex, de-duplicate on the fly.

            VertexIndexLookupMap::const_iterator vertexLoc = m_vertexIndices.find(vertexToAdd);  //	NOLINT

            if (vertexLoc == m_vertexIndices.end())
            {
                //	Vertex is new, update all data structures

                m_vertexIndices[vertexToAdd] = m_indexedVertices->size();
                m_vertexIndexRemapper.push_back(m_indexedVertices->size());
                m_indexedVertices->push_back(vertexToAdd);
            }
            else
            {
                //	Vertex is a duplicate, so remap to it

                m_vertexIndexRemapper.push_back(vertexLoc->second);
            }

            //	The index we return should always be the remapper size minus 1

            return (m_vertexIndexRemapper.size() - 1);
        }

        TriangleMeshBuilderResultCodes AddTriangle(const TriangleByIndices& triangleToAdd) final
        {
            //	Insure the indices are in bounds

            if ((triangleToAdd[0] >= m_vertexIndexRemapper.size()) ||
                (triangleToAdd[1] >= m_vertexIndexRemapper.size()) ||
                (triangleToAdd[2] >= m_vertexIndexRemapper.size()))
            {
                return (TriangleMeshBuilderResultCodes::VERTEX_INDEX_OUT_OF_BOUNDS);
            }

            //	Copy on write for the triangle and edge incidence structures.  We need to duplicate them if we no longer
            //hold the pointers uniquely.

            if (!m_triangles.unique())
            {
                m_triangles = std::make_shared<std::vector<TriangleByIndices>>(*m_triangles);
            }

            //	Remap the triangle indices

            TriangleByIndices remappedTriangle(m_vertexIndexRemapper[triangleToAdd[0]],
                                               m_vertexIndexRemapper[triangleToAdd[1]],
                                               m_vertexIndexRemapper[triangleToAdd[2]]);

            //	Add the triangle to the vector

            m_triangles->push_back(remappedTriangle);

            //	Update the bounding box

            TriangleByVertices triByVerts((*m_indexedVertices)[remappedTriangle[0]],
                                          (*m_indexedVertices)[remappedTriangle[1]],
                                          (*m_indexedVertices)[remappedTriangle[2]]);

            NUMERIC_PRECISION minX = std::min(static_cast<NUMERIC_PRECISION>(std::min(
                                                  triByVerts[0].x(), std::min(triByVerts[1].x(), triByVerts[2].x()))),
                                              boundingBox().minima().x());
            NUMERIC_PRECISION minY =
                std::min((NUMERIC_PRECISION)std::min(triByVerts[0].y(), std::min(triByVerts[1].y(), triByVerts[2].y())),
                         boundingBox().minima().y());
            NUMERIC_PRECISION minZ =
                std::min((NUMERIC_PRECISION)std::min(triByVerts[0].z(), std::min(triByVerts[1].z(), triByVerts[2].z())),
                         boundingBox().minima().z());

            NUMERIC_PRECISION maxX =
                std::max((NUMERIC_PRECISION)std::max(triByVerts[0].x(), std::max(triByVerts[1].x(), triByVerts[2].x())),
                         boundingBox().maxima().x());
            NUMERIC_PRECISION maxY =
                std::max((NUMERIC_PRECISION)std::max(triByVerts[0].y(), std::max(triByVerts[1].y(), triByVerts[2].y())),
                         boundingBox().maxima().y());
            NUMERIC_PRECISION maxZ =
                std::max((NUMERIC_PRECISION)std::max(triByVerts[0].z(), std::max(triByVerts[1].z(), triByVerts[2].z())),
                         boundingBox().maxima().z());

            m_boundingBox = std::make_unique<Cork::Math::BBox3D>(Cork::Math::Vector3D(minX, minY, minZ),
                                                                 Cork::Math::Vector3D(maxX, maxY, maxZ));

            //	All is well if we made it here

            return (TriangleMeshBuilderResultCodes::SUCCESS);
        }

        std::unique_ptr<TriangleMesh> Mesh() final
        {
            std::shared_ptr<const std::vector<TriangleByIndices>> triangles = m_triangles;

            return (std::unique_ptr<TriangleMesh>(new TriangleMeshImpl(triangles, m_indexedVertices, boundingBox())));
        }

       private:
        typedef std::map<Cork::Math::Vertex3D, IndexType, Cork::Math::Vertex3DMapCompare,
                         boost::alignment::aligned_allocator<Cork::Math::Vertex3D>>
            VertexIndexLookupMap;

        VertexIndexLookupMap m_vertexIndices;
        std::shared_ptr<VertexVector> m_indexedVertices;
        std::vector<VertexIndexType> m_vertexIndexRemapper;

        std::shared_ptr<std::vector<TriangleByIndices>> m_triangles;

        std::unique_ptr<Cork::Math::BBox3D> m_boundingBox;
    };

    //
    //	Factory method to return an incremental triangle mesh builder
    //

    std::unique_ptr<IncrementalVertexIndexTriangleMeshBuilder> IncrementalVertexIndexTriangleMeshBuilder::GetBuilder(
        size_t numVertices, size_t numTriangles)
    {
        return (std::unique_ptr<IncrementalVertexIndexTriangleMeshBuilder>(
            new IncrementalVertexIndexTriangleMeshBuilderImpl(numVertices, numTriangles)));
    }

}  // namespace Cork
