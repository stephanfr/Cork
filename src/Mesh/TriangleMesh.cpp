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

#include "mesh/triangle_mesh_with_topo_cache.hpp"

#include <map>
#include <set>

#include "intersection/self_intersection_finder.hpp"
#include "intersection/triangulator.hpp"

#include "primitives/index_remapper.hpp"

#include "statistics/statistics_engines.h"

namespace Cork::Meshes
{
    //
    //	TriangleMeshImpl is a straightforward implementation of a triangularized solid expressed as a set
    //		of vertices and triangles defined as 3-tuples of indices into the vertex set.
    //

    class TriangleMeshImpl : public TriangleMeshWithTopoCache
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

        [[nodiscard]] Meshes::TriangleByIndicesVectorTopoCache& topo_cache() const
        {
            if (!topo_cache_)
            {
                auto get_quantizer_result =
                    Math::Quantizer::get_quantizer(max_vertex_magnitude(), min_and_max_edge_lengths().min());

                const_cast<std::unique_ptr<Meshes::TriangleByIndicesVectorTopoCache>&>(topo_cache_)
                    .reset(new Meshes::TriangleByIndicesVectorTopoCache(
                        *m_triangles, *m_vertices, m_triangles->size() * 4, get_quantizer_result.return_value()));
            }

            return *topo_cache_;
        }

        [[nodiscard]] Statistics::GeometricStatistics ComputeGeometricStatistics(
            Statistics::GeometricProperties props_to_compute) const final
        {
            Statistics::GeometricStatisticsEngine geometricStatsEngine(*this, props_to_compute);

            return Statistics::GeometricStatistics(geometricStatsEngine.statistics());
        }

        [[nodiscard]] TopologicalStatisticsResult ComputeTopologicalStatistics(
            Statistics::TopologicalProperties props_to_compute) const final
        {
            Statistics::TopologicalStatisticsEngine statsEngine(*this);

            auto result = statsEngine.Analyze(props_to_compute);

            if (result.failed())
            {
                return TopologicalStatisticsResult::failure(result, TopologicalStatisticsResultCodes::ANALYSIS_FAILED,
                                                            "Topological Analysis Failed");
            }

            return TopologicalStatisticsResult::success(result.return_value());
        }

        void remove_self_intersections(const Statistics::TopologicalStatistics& topo_stats)
        {
            //  Use a set below, as this will deduplicate and the ordering will insure we remove triangles
            //      from the end of the vector moving forward.  This prevents corruption of the vector
            //      by having the vector compress with removals and then having the wrong element removed by index.

            std::set<TriangleByIndicesIndex, std::greater<TriangleByIndicesIndex>> all_triangles_to_remove;
            std::vector<TriangleByIndices> all_triangles_to_add;

            all_triangles_to_add.reserve(1000);

            for (auto& record : topo_stats.self_intersections())
            {
                std::set<TriangleByIndicesIndex> triangles_to_remove;

                for (auto triangle_id : record.triangles_including_se_vertex())
                {
                    triangles_to_remove.insert(triangle_id);
                }

                std::vector<Hole> holes_for_se = get_hole_for_self_intersection(triangles_to_remove);

                auto get_hole_closing_triangles_result = get_hole_closing_triangles(holes_for_se[0]);

                if (!get_hole_closing_triangles_result.succeeded())
                {
                    std::cout << "closing hole failed" << std::endl;

                    auto next_ring_of_tris = find_enclosing_triangles(triangles_to_remove);
                    continue;
                }

                Meshes::TriangleByIndicesVectorTopoCache minimal_cache(
                    *(get_hole_closing_triangles_result.return_ptr()),
                    const_cast<Primitives::Vertex3DVector&>(vertices()),
                    get_hole_closing_triangles_result.return_ptr()->size() * 4, topo_cache().quantizer());

                Intersection::SelfIntersectionFinder se_finder(minimal_cache);

                auto si_stats = se_finder.CheckSelfIntersection();

                if (si_stats.size() == 0)
                {
                    for (auto tri_to_remove_index : triangles_to_remove)
                    {
                        all_triangles_to_remove.emplace(tri_to_remove_index);
                    }

                    for (auto tri_to_add : *(get_hole_closing_triangles_result.return_ptr()))
                    {
                        all_triangles_to_add.emplace_back(tri_to_add);
                    }
                }
                else
                {
                    std::cout << "Found self-intersection trying to close hole" << std::endl;
                }
            }

            for (auto tri_to_remove_index : all_triangles_to_remove)
            {
                remove_triangle(tri_to_remove_index);
            }

            for (auto& tri_to_add : all_triangles_to_add)
            {
                AddTriangle(tri_to_add);
            }

            topo_cache_.release();
        }

        std::vector<Hole> get_hole_for_self_intersection(
            const std::set<TriangleByIndicesIndex>& triangles_to_remove) const
        {
            std::map<EdgeByIndices, uint32_t, Primitives::EdgeByIndicesMapCompare> edge_counts;

            for (auto tri_to_remove_index : triangles_to_remove)
            {
                auto edge_ab = edge_counts.find(triangles()[tri_to_remove_index].edge(TriangleEdgeId::AB));
                auto edge_bc = edge_counts.find(triangles()[tri_to_remove_index].edge(TriangleEdgeId::BC));
                auto edge_ca = edge_counts.find(triangles()[tri_to_remove_index].edge(TriangleEdgeId::CA));

                if (edge_ab == edge_counts.end())
                {
                    edge_counts.insert(std::make_pair(triangles()[tri_to_remove_index].edge(TriangleEdgeId::AB), 1));
                }
                else
                {
                    edge_ab->second++;
                }

                if (edge_bc == edge_counts.end())
                {
                    edge_counts.insert(std::make_pair(triangles()[tri_to_remove_index].edge(TriangleEdgeId::BC), 1));
                }
                else
                {
                    edge_bc->second++;
                }

                if (edge_ca == edge_counts.end())
                {
                    edge_counts.insert(std::make_pair(triangles()[tri_to_remove_index].edge(TriangleEdgeId::CA), 1));
                }
                else
                {
                    edge_ca->second++;
                }
            }

            EdgeByIndicesVector hole_edges;

            for (auto edge : edge_counts)
            {
                if (edge.second == 1)
                {
                    hole_edges.emplace_back(edge.first);
                }
            }

            return HoleBuilder::extract_holes(hole_edges);
        }

        using GetHoleClosingTrianglesResult =
            SEFUtility::ResultWithReturnUniquePtr<HoleClosingResultCodes, TriangleByIndicesVector>;

        GetHoleClosingTrianglesResult get_hole_closing_triangles(const Hole& hole)
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
                return GetHoleClosingTrianglesResult::failure(result, HoleClosingResultCodes::TRIANGULATION_FAILED,
                                                              "Triangulation failed for hole");
            }

            //  Add the new triangles which close the hole.  There will never be new vertices to add based
            //      on the settings of the triangulator - thus this operation is simple.

            std::unique_ptr<TriangleByIndicesVector> hole_closing_triangles =
                std::make_unique<TriangleByIndicesVector>();

            hole_closing_triangles->reserve(result.return_ptr()->size() + 2);

            for (auto triangle_to_add : *(result.return_ptr()))
            {
                hole_closing_triangles->emplace_back(
                    TriangleByIndices(Primitives::UNINTIALIZED_INDEX, hole.vertices()[triangle_to_add.v0()],
                                      hole.vertices()[triangle_to_add.v2()], hole.vertices()[triangle_to_add.v1()]));
            }

            //  Return the triangles to close the hole

            return GetHoleClosingTrianglesResult::success(std::move(hole_closing_triangles));
        }

        HoleClosingResult close_holes(const Statistics::TopologicalStatistics& topo_stats)
        {
            //  There may be multiple holes

            for (auto hole : topo_stats.holes())
            {
                auto result = get_hole_closing_triangles(hole);

                if (!result.succeeded())
                {
                    return HoleClosingResult::failure(result.error_code(), result.message());
                }

                for (auto tri_to_add : *(result.return_ptr()))
                {
                    AddTriangle(tri_to_add);
                }
            }

            //  Finished with success

            return HoleClosingResult::success();
        }

        std::set<TriangleByIndicesIndex> find_enclosing_triangles(
            const std::set<TriangleByIndicesIndex>& triangles_patch)
        {
            std::map<TriangleByIndicesIndex, const TopoTri*> topo_tris_by_index;

            for (auto& next_topo_tri : topo_cache_->triangles())
            {
                topo_tris_by_index.insert(std::make_pair(next_topo_tri.source_triangle_id(), &next_topo_tri));
            }

            std::set<const TopoTri*> topo_tris_in_patch;

            for (auto& next_triangle : triangles_patch)
            {
                topo_tris_in_patch.emplace(topo_tris_by_index[next_triangle]);
            }

            std::set<TriangleByIndicesIndex> enclosing_triangles;

            for (auto next_topo_tri : topo_tris_in_patch)
            {
                for (auto touching_edge : next_topo_tri->edges())
                {
                    for (auto triangle_touching : touching_edge->triangles())
                    {
                        if (!triangles_patch.contains(triangle_touching->source_triangle_id()))
                        {
                            enclosing_triangles.emplace(triangle_touching->source_triangle_id());
                        }
                    }
                }
            }

            return enclosing_triangles;
        }

       private:
        std::shared_ptr<TriangleByIndicesVector> m_triangles;
        std::shared_ptr<Vertex3DVector> m_vertices;

        const Primitives::BBox3D m_boundingBox;
        const Primitives::MinAndMaxEdgeLengths min_and_max_edge_lengths_;
        double max_vertex_magnitude_;

        std::unique_ptr<Meshes::TriangleByIndicesVectorTopoCache> topo_cache_;
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

        TriangleMeshBuilderResultCodes AddTriangle(VertexIndex a, VertexIndex b, VertexIndex c) final
        {
            //	Insure the indices are in bounds

            if ((a >= m_vertexIndexRemapper.size()) || (b >= m_vertexIndexRemapper.size()) ||
                (c >= m_vertexIndexRemapper.size()))
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

            TriangleByIndices remappedTriangle(m_triangles->size(), m_vertexIndexRemapper[a], m_vertexIndexRemapper[b],
                                               m_vertexIndexRemapper[c]);

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
        Primitives::IndexRemapper<VertexIndex> m_vertexIndexRemapper;

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
