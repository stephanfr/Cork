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

#include <map>
#include <unordered_map>

#include "mesh/triangle_mesh.h"
#include "statistics/statistics_engines.h"

namespace Cork
{
    //
    //	TriangleMeshImpl is a straightforward implementation of a triangularized solid expressed as a set
    //		of vertices and triangles defined as 3-tuples of indices into the vertex set.
    //

    class TriangleMeshImpl : public TriangleMesh
    {
       public:
        TriangleMeshImpl(std::shared_ptr<std::vector<TriangleByIndices>>& triangles,
                         std::shared_ptr<VertexVector>& vertices, const Math::BBox3D& boundingBox,
                         const Math::MinAndMaxEdgeLengths min_and_max_edge_lengths, double max_vertex_magnitude)
            : m_triangles(triangles),
              m_vertices(vertices),
              m_boundingBox(boundingBox),
              min_and_max_edge_lengths_(min_and_max_edge_lengths),
              max_vertex_magnitude_(max_vertex_magnitude)
        {
        }

        [[nodiscard]] size_t numTriangles() const final { return (m_triangles->size()); }

        [[nodiscard]] size_t numVertices() const final { return (m_vertices->size()); }

        [[nodiscard]] const VertexVector& vertices() const final { return (*m_vertices); }

        [[nodiscard]] const std::vector<TriangleByIndices>& triangles() const final { return (*m_triangles); }

        [[nodiscard]] TriangleByVertices triangleByVertices(const TriangleByIndices& triangleByIndices) const final
        {
            return (TriangleByVertices((*m_vertices)[triangleByIndices.a()],
                                       (*m_vertices)[triangleByIndices.b()],
                                       (*m_vertices)[triangleByIndices.c()]));
        }

        void remove_triangle(size_t triangle_index) { m_triangles->erase(m_triangles->begin() + triangle_index); }

        [[nodiscard]] const Math::BBox3D& boundingBox() const final { return m_boundingBox; }
        [[nodiscard]] Math::MinAndMaxEdgeLengths min_and_max_edge_lengths() const final
        {
            return min_and_max_edge_lengths_;
        }
        [[nodiscard]] double max_vertex_magnitude() const final { return max_vertex_magnitude_; }

        [[nodiscard]] Statistics::GeometricStatistics ComputeGeometricStatistics() const final
        {
            Statistics::GeometricStatisticsEngine geometricStatsEngine(*this);

            return (Statistics::GeometricStatistics(
                m_vertices->size(), geometricStatsEngine.numTriangles(), geometricStatsEngine.area(),
                geometricStatsEngine.volume(), geometricStatsEngine.minEdgeLength(),
                geometricStatsEngine.maxEdgeLength(), geometricStatsEngine.boundingBox()));
        }

        [[nodiscard]] Statistics::TopologicalStatistics ComputeTopologicalStatistics() const final
        {
            Statistics::TopologicalStatisticsEngine statsEngine(*this);

            return (statsEngine.Analyze());
        }

       private:
        std::shared_ptr<std::vector<TriangleByIndices>> m_triangles;
        std::shared_ptr<VertexVector> m_vertices;

        const Math::BBox3D m_boundingBox;
        const Math::MinAndMaxEdgeLengths min_and_max_edge_lengths_;
        double max_vertex_magnitude_;
    };

    //
    //	IncrementalVertexIndexTriangleMeshBuilderImpl implements an incremental triangle mesh builder
    //		which can be used to construct a triangle mesh from a list of vertices and
    //		triangles assembled from those vertices.
    //
    //	This class also uses copy-on-write semantics for the internal data structures that are eventually
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
              m_boundingBox(Math::Vector3D(NUMERIC_PRECISION_MAX, NUMERIC_PRECISION_MAX, NUMERIC_PRECISION_MAX),
                            Math::Vector3D(NUMERIC_PRECISION_MIN, NUMERIC_PRECISION_MIN, NUMERIC_PRECISION_MIN)),
              max_vertex_magnitude_(NUMERIC_PRECISION_MIN)
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

        ~IncrementalVertexIndexTriangleMeshBuilderImpl() = default;  //	NOLINT

        IncrementalVertexIndexTriangleMeshBuilderImpl() = delete;
        IncrementalVertexIndexTriangleMeshBuilderImpl(const IncrementalVertexIndexTriangleMeshBuilderImpl&) = delete;
        IncrementalVertexIndexTriangleMeshBuilderImpl(const IncrementalVertexIndexTriangleMeshBuilderImpl&&) = delete;

        IncrementalVertexIndexTriangleMeshBuilderImpl& operator=(const IncrementalVertexIndexTriangleMeshBuilderImpl&) =
            delete;
        IncrementalVertexIndexTriangleMeshBuilderImpl& operator=(
            const IncrementalVertexIndexTriangleMeshBuilderImpl&&) = delete;

        [[nodiscard]] size_t num_vertices() const final { return m_indexedVertices->size(); }

        [[nodiscard]] const Math::BBox3D& boundingBox() const { return m_boundingBox; }

        [[nodiscard]] double max_vertex_magnitude() const { return max_vertex_magnitude_; }
        [[nodiscard]] Math::MinAndMaxEdgeLengths min_and_max_edge_lengths() const { return min_and_max_edge_lengths_; }

        VertexIndex AddVertex(const Vertex& vertexToAdd) final
        {
            //	Copy on write for the vertex structure.  We need to duplicate the vector if we no longer hold the
            // pointer uniquely.

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

            //	Copy on write for the triangle and edge incidence structures.
            //      We need to duplicate them if we no longer hold the pointers uniquely.

            if (!m_triangles.unique())
            {
                m_triangles = std::make_shared<std::vector<TriangleByIndices>>(*m_triangles);
            }

            //	Remap the triangle indices

            TriangleByIndices remappedTriangle(m_vertexIndexRemapper[VertexIndex::integer_type(triangleToAdd.a())],
                                               m_vertexIndexRemapper[VertexIndex::integer_type(triangleToAdd.b())],
                                               m_vertexIndexRemapper[VertexIndex::integer_type(triangleToAdd.c())]);

            //	Add the triangle to the vector

            m_triangles->push_back(remappedTriangle);

            //  Compute a few metrics

            TriangleByVertices tri_by_verts((*m_indexedVertices)[remappedTriangle[0]],
                                            (*m_indexedVertices)[remappedTriangle[1]],
                                            (*m_indexedVertices)[remappedTriangle[2]]);

            m_boundingBox.convex(tri_by_verts.bounding_box());

            max_vertex_magnitude_ = std::max(max_vertex_magnitude_, tri_by_verts.max_magnitude_vertex());

            min_and_max_edge_lengths_.update(tri_by_verts.min_and_max_edge_lengths());

            //	All is well if we made it here

            return (TriangleMeshBuilderResultCodes::SUCCESS);
        }

        std::unique_ptr<TriangleMesh> Mesh() final
        {
            std::shared_ptr<std::vector<TriangleByIndices>> triangles = m_triangles;

            return (std::unique_ptr<TriangleMesh>(new TriangleMeshImpl(
                triangles, m_indexedVertices, boundingBox(), min_and_max_edge_lengths(), max_vertex_magnitude())));
        }

       private:
        typedef std::map<Math::Vertex3D, IndexType, Math::Vertex3DMapCompare> VertexIndexLookupMap;

        VertexIndexLookupMap m_vertexIndices;
        std::shared_ptr<VertexVector> m_indexedVertices;
        std::vector<VertexIndex> m_vertexIndexRemapper;

        std::shared_ptr<std::vector<TriangleByIndices>> m_triangles;

        Math::BBox3D m_boundingBox;
        Math::MinAndMaxEdgeLengths min_and_max_edge_lengths_;
        double max_vertex_magnitude_;
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
