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
#include <set>
#include <unordered_map>

#include "intersection/triangulator.hpp"
#include "mesh/triangle_mesh.h"
#include "statistics/statistics_engines.h"

namespace Cork::Meshes
{
    using Vertex3DVector = Primitives::Vertex3DVector;

    using IndexType = Primitives::IndexType;
    using VertexIndex = Primitives::VertexIndex;

    using EdgeByVertices = Primitives::EdgeByVertices;

    using TriangleByIndices = Primitives::TriangleByIndices;
    using TriangleByIndicesVector = Primitives::TriangleByIndicesVector;
    using TriangleByIndicesIndex = Primitives::TriangleByIndicesIndex;

    using TriangleByVertices = Primitives::TriangleByVertices;

    //
    //	TriangleMeshImpl is a straightforward implementation of a triangularized solid expressed as a set
    //		of vertices and triangles defined as 3-tuples of indices into the vertex set.
    //

    class TriangleMeshImpl : public TriangleMesh
    {
       public:
        TriangleMeshImpl(std::shared_ptr<TriangleByIndicesVector>& triangles, std::shared_ptr<Vertex3DVector>& vertices,
                         const Primitives::BBox3D& boundingBox,
                         const Primitives::MinAndMaxEdgeLengths min_and_max_edge_lengths, double max_vertex_magnitude)
            : m_triangles(triangles),
              m_vertices(vertices),
              m_boundingBox(boundingBox),
              min_and_max_edge_lengths_(min_and_max_edge_lengths),
              max_vertex_magnitude_(max_vertex_magnitude)
        {
        }

        [[nodiscard]] size_t numTriangles() const final { return (m_triangles->size()); }

        [[nodiscard]] size_t numVertices() const final { return (m_vertices->size()); }

        [[nodiscard]] const Vertex3DVector& vertices() const final { return (*m_vertices); }

        [[nodiscard]] const TriangleByIndicesVector& triangles() const final { return (*m_triangles); }

        [[nodiscard]] TriangleByVertices triangleByVertices(const TriangleByIndices& triangleByIndices) const final
        {
            return (TriangleByVertices((*m_vertices)[triangleByIndices.a()], (*m_vertices)[triangleByIndices.b()],
                                       (*m_vertices)[triangleByIndices.c()]));
        }

        void AddTriangle(const TriangleByIndices& triangle_to_add) final { m_triangles->emplace_back(triangle_to_add); }

        void remove_triangle(TriangleByIndicesIndex triangle_index)
        {
            m_triangles->erase(m_triangles->begin() + TriangleByIndicesIndex::integer_type(triangle_index));
        }

        [[nodiscard]] const Primitives::BBox3D& boundingBox() const final { return m_boundingBox; }
        [[nodiscard]] Primitives::MinAndMaxEdgeLengths min_and_max_edge_lengths() const final
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

        [[nodiscard]] TopologicalStatisticsResult ComputeTopologicalStatistics() const final
        {
            Statistics::TopologicalStatisticsEngine statsEngine(*this);

            auto result = statsEngine.Analyze();

            if( result.failed() )
            {
                return TopologicalStatisticsResult::failure( result, TopologicalStatisticsResultCodes::ANALYSIS_FAILED, "Topological Analysis Failed" );
            }

            return TopologicalStatisticsResult::success( result.return_value() );
        }

        void remove_self_intersections(const Statistics::TopologicalStatistics& topo_stats)
        {
            //  Use a set below, as this will deduplicate and the ordering will insure we remove triangles
            //      from the end of the vector moving forward.  This prevents corruption of the vector
            //      by having the vector compress with removals and then having the wrong element removed by index.

            std::set<TriangleByIndicesIndex, std::greater<TriangleByIndicesIndex>> triangles_to_remove;

            for (auto& record : topo_stats.self_intersections())
            {
                //                for (auto triangle_id : record.triangles_sharing_edge() )
                for (auto triangle_id : record.triangles_touching_triangles_sharing_edge())
                {
                    triangles_to_remove.insert(triangle_id);
                }
            }

            for (auto tri_to_remove_index : triangles_to_remove)
            {
                remove_triangle(tri_to_remove_index);
            }
        }

        HoleClosingResult close_holes(const Statistics::TopologicalStatistics& topo_stats)
        {
            //  There may be multiple holes

            for (auto hole : topo_stats.holes())
            {
                //  Determine the projection needed to turn this into a 2D triangulation problem

                Cork::Triangulator::NormalProjector projector(
                    vertices()[hole.vertices()[0]], vertices()[hole.vertices()[1]], vertices()[hole.vertices()[2]]);

                //  Get the triangulator and add the points on the hole edge and the segments joining them.
                //      This is trivial as the vertices are ordered so segments are just one after the next.
                //      Holes must be closed, thus the last segment from the last vertex to the first.

                Cork::Triangulator::Triangulator triangulator;

                for (auto vertex_index : hole.vertices())
                {
                    triangulator.add_point(vertices()[vertex_index], true, projector);
                }

                for (int i = 0; i < hole.vertices().size() - 1; i++)
                {
                    triangulator.add_segment(i, i + 1, true);
                }

                triangulator.add_segment(hole.vertices().size() - 1, 0, true);

                //  Compute the triangulation - I suppose some really messed up geometries might fail here.

                auto result = triangulator.compute_triangulation();

                if (result.failed())
                {
                    return HoleClosingResult::failure(result, HoleClosingResultCodes::TRIANGULATION_FAILED,
                                                      "Triangulation failed for hole");
                }

                //  Add the new triangles which close the hole.  There will neve be new vertices to add based
                //      on the settings of the triangulator - thus this operation is simple.

                for (auto triangle_to_add : *(result.return_ptr()))
                {
                    AddTriangle(Cork::Primitives::TriangleByIndices(hole.vertices()[triangle_to_add.v2()],
                                                                    hole.vertices()[triangle_to_add.v1()],
                                                                    hole.vertices()[triangle_to_add.v0()]));
                }
            }

            //  Finished with success

            return HoleClosingResult::success();
        }

       private:
        std::shared_ptr<TriangleByIndicesVector> m_triangles;
        std::shared_ptr<Vertex3DVector> m_vertices;

        const Primitives::BBox3D m_boundingBox;
        const Primitives::MinAndMaxEdgeLengths min_and_max_edge_lengths_;
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
            : m_indexedVertices(new Vertex3DVector()),
              m_triangles(new TriangleByIndicesVector()),
              m_boundingBox(Primitives::Vector3D(NUMERIC_PRECISION_MAX, NUMERIC_PRECISION_MAX, NUMERIC_PRECISION_MAX),
                            Primitives::Vector3D(NUMERIC_PRECISION_MIN, NUMERIC_PRECISION_MIN, NUMERIC_PRECISION_MIN)),
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

        [[nodiscard]] const Primitives::BBox3D& boundingBox() const { return m_boundingBox; }

        [[nodiscard]] double max_vertex_magnitude() const { return max_vertex_magnitude_; }
        [[nodiscard]] Primitives::MinAndMaxEdgeLengths min_and_max_edge_lengths() const
        {
            return min_and_max_edge_lengths_;
        }

        VertexIndex AddVertex(const Primitives::Vertex3D& vertexToAdd) final
        {
            //	Copy on write for the vertex structure.  We need to duplicate the vector if we no longer hold the
            // pointer uniquely.

            if (!m_indexedVertices.unique())
            {
                m_indexedVertices = std::make_shared<Vertex3DVector>(*m_indexedVertices);
            }

            //	Add the vertex, de-duplicate on the fly.

            VertexIndexLookupMap::const_iterator vertexLoc = m_vertexIndices.find(vertexToAdd);  //	NOLINT

            if (vertexLoc == m_vertexIndices.end())
            {
                //	Vertex is new, update all data structures

                m_vertexIndices[vertexToAdd] = m_indexedVertices->size();
                m_vertexIndexRemapper.push_back(VertexIndex::integer_type(m_indexedVertices->size()));
                m_indexedVertices->push_back(vertexToAdd);
            }
            else
            {
                //	Vertex is a duplicate, so remap to it

                m_vertexIndexRemapper.push_back(vertexLoc->second);
            }

            //	The index we return should always be the remapper size minus 1

            return (VertexIndex::integer_type(m_vertexIndexRemapper.size()) - 1u);
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
                m_triangles = std::make_shared<TriangleByIndicesVector>(*m_triangles);
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
            std::shared_ptr<TriangleByIndicesVector> triangles = m_triangles;

            return (std::unique_ptr<TriangleMesh>(new TriangleMeshImpl(
                triangles, m_indexedVertices, boundingBox(), min_and_max_edge_lengths(), max_vertex_magnitude())));
        }

       private:
        typedef std::map<Primitives::Vertex3D, IndexType, Primitives::Vertex3DMapCompare> VertexIndexLookupMap;

        VertexIndexLookupMap m_vertexIndices;
        std::shared_ptr<Vertex3DVector> m_indexedVertices;
        std::vector<VertexIndex> m_vertexIndexRemapper;

        std::shared_ptr<TriangleByIndicesVector> m_triangles;

        Primitives::BBox3D m_boundingBox;
        Primitives::MinAndMaxEdgeLengths min_and_max_edge_lengths_;
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

}  // namespace Cork::Meshes
