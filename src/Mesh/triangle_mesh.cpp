// +-------------------------------------------------------------------------
// | triangle_mesh.cpp
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

#include "file_formats/files.hpp"
#include "intersection/self_intersection_finder.hpp"
#include "intersection/triangulator.hpp"
#include "mesh/surface_mesh.hpp"
#include "mesh/triangle_mesh_builder.hpp"
#include "primitives/index_remapper.hpp"
#include "statistics/statistics_engines.hpp"

namespace Cork::Meshes
{
    class SelfIntersectionResolutionPlan
    {
       public:
        SelfIntersectionResolutionPlan() = default;

        std::vector<TriangleByIndicesIndexSet> resolutions() { return resolutions_; }

        void add_resolution(const TriangleByIndicesIndexSet& new_resolution)
        {
            //  Merge any new resolution with an existing resolution it intersects

            TriangleByIndicesIndexSet resolution{new_resolution};
            bool finished = false;

            while (!finished)
            {
                bool merged = false;

                for (auto itr = resolutions_.begin(); itr != resolutions_.end(); itr++)
                {
                    if (itr->intersects(resolution))
                    {
                        resolution.merge(*itr);
                        resolutions_.erase(itr);
                        merged = true;
                        break;
                    }
                }

                finished = !merged;
            }

            resolutions_.emplace_back(resolution);
        }

       private:
        std::vector<TriangleByIndicesIndexSet> resolutions_;
    };

    //
    //	TriangleMeshImpl is a straightforward implementation of a triangularized solid expressed as a set
    //		of vertices and triangles defined as 3-tuples of indices into the vertex set.
    //

    class TriangleMeshImpl : public MeshBase
    {
       public:
        TriangleMeshImpl(MeshBase&& mesh_base) : MeshBase(std::move(mesh_base)) {}

        void add_triangle(const TriangleByIndices& triangle_to_add) { tris_->emplace_back(triangle_to_add); }

        void remove_triangle(TriangleByIndicesIndex triangle_index)
        {
            tris_->erase(tris_->begin() + TriangleByIndicesIndex::integer_type(triangle_index));
        }

        [[nodiscard]] Statistics::GeometricStatistics ComputeGeometricStatistics(
            Statistics::GeometricProperties props_to_compute) const
        {
            Statistics::GeometricStatisticsEngine geometricStatsEngine(*this, props_to_compute);

            return Statistics::GeometricStatistics(geometricStatsEngine.statistics());
        }

        [[nodiscard]] TopologicalStatisticsResult ComputeTopologicalStatistics(
            Statistics::TopologicalProperties props_to_compute) const
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

        SelfIntersectionResolutionResults remove_self_intersections(const Statistics::TopologicalStatistics& topo_stats)
        {
            uint32_t        sis_resolved = 0;
            uint32_t        sis_abdondoned = 0;
            uint32_t        failures_on_final_resolution = 0;
            
            SelfIntersectionResolutionPlan resolution_plan;

            for (auto& intersecting_edge : topo_stats.self_intersecting_edges())
            {
                auto& triangle_containing_edge = triangles()[intersecting_edge.edge_triangle_id()];

                VertexIndex vertex_1 = triangle_containing_edge.edge(intersecting_edge.edge_index()).first();
                VertexIndex vertex_2 = triangle_containing_edge.edge(intersecting_edge.edge_index()).second();

                //  Try to fix the self intersection by removing different sets of triangles:
                //
                //  1)  First SE vertex only
                //  2)  Second SE vertex only
                //  3)  First and Second vertices together
                //  4)  First and Second vertices with the enclosing ring

                TriangleByIndicesIndexSet triangles_containing_vertex_1{
                    std::move(find_triangles_containing_vertex(vertex_1))};

                if (resolves_self_intersection(intersecting_edge.triangle_instersected_id(),
                                               triangles_containing_vertex_1))
                {
                    sis_resolved++;
                    resolution_plan.add_resolution(triangles_containing_vertex_1);
                    continue;
                }

                TriangleByIndicesIndexSet triangles_containing_vertex_2{
                    std::move(find_triangles_containing_vertex(vertex_2))};

                if (resolves_self_intersection(intersecting_edge.triangle_instersected_id(),
                                               triangles_containing_vertex_2))
                {
                    sis_resolved++;
                    resolution_plan.add_resolution(triangles_containing_vertex_2);
                    continue;
                }

                TriangleByIndicesIndexSet triangles_containing_both_vertices(triangles_containing_vertex_1,
                                                                             triangles_containing_vertex_2);

                if (resolves_self_intersection(intersecting_edge.triangle_instersected_id(),
                                               triangles_containing_both_vertices))
                {
                    sis_resolved++;
                    resolution_plan.add_resolution(triangles_containing_both_vertices);
                    continue;
                }

                TriangleByIndicesIndexSet both_vertices_and_ring(
                    triangles_containing_both_vertices, find_enclosing_triangles(triangles_containing_both_vertices));

                if (resolves_self_intersection(intersecting_edge.triangle_instersected_id(), both_vertices_and_ring))
                {
                    sis_resolved++;
                    resolution_plan.add_resolution(both_vertices_and_ring);
                    continue;
                }

                sis_abdondoned++;
            }

            //  Use a set below, as this will deduplicate and the ordering will insure we remove triangles
            //      from the end of the vector moving forward.  This prevents corruption of the vector
            //      by having the vector compress with removals and then having the wrong element removed by index.

            std::set<TriangleByIndicesIndex, std::greater<TriangleByIndicesIndex>> all_triangles_to_remove;
            std::vector<TriangleByIndices> all_triangles_to_add;

            all_triangles_to_add.reserve(1000);

            for (const auto& resolution : resolution_plan.resolutions())
            {
                std::vector<Hole> holes_for_se = get_hole_for_self_intersection(resolution);

                if (holes_for_se.size() != 1)
                {
                    failures_on_final_resolution++;
                    continue;
                }

                GetHoleClosingTrianglesResult get_hole_closing_triangles_result =
                    get_hole_closing_triangles(holes_for_se[0]);

                if (!get_hole_closing_triangles_result.succeeded())
                {
                    failures_on_final_resolution++;
                    continue;
                }

                for (auto const& tri : resolution)
                {
                    all_triangles_to_remove.emplace(tri);
                }

                for (const auto& tri : *(get_hole_closing_triangles_result.return_ptr()))
                {
                    all_triangles_to_add.emplace_back(tri);
                }
            }

            for (auto tri_to_remove_index : all_triangles_to_remove)
            {
                remove_triangle(tri_to_remove_index);
            }

            for (auto& tri_to_add : all_triangles_to_add)
            {
                add_triangle(tri_to_add);
            }

            //  Compact the vertices and remap the triangles

            compact();

            //  Return our result stats

            return SelfIntersectionResolutionResults( sis_resolved, sis_abdondoned, failures_on_final_resolution );
        }

        bool resolves_self_intersection(const TriangleByIndicesIndex intersected_triangle,
                                        const TriangleByIndicesIndexSet& tris_to_remove)
        {
            std::vector<Hole> holes_for_se = get_hole_for_self_intersection(tris_to_remove);

            if (holes_for_se.size() != 1)
            {
                return false;
            }

            GetHoleClosingTrianglesResult get_hole_closing_triangles_result =
                get_hole_closing_triangles(holes_for_se[0]);

            if (!get_hole_closing_triangles_result.succeeded())
            {
                return false;
            }

            TriangleByIndicesIndexSet full_patch{tris_to_remove};

            auto first_ring_of_tris = find_enclosing_triangles(holes_for_se[0], tris_to_remove);

            full_patch.merge(first_ring_of_tris);

            auto second_ring_of_tris = find_enclosing_triangles(full_patch);

            full_patch.merge(second_ring_of_tris);

            auto third_ring_of_tris = find_enclosing_triangles(full_patch);

            SurfaceMesh patch(vertices());

            patch.add(*(get_hole_closing_triangles_result.return_ptr()))
                .add(as_triangles(first_ring_of_tris))
                .add(as_triangles(second_ring_of_tris))
                .add(as_triangles(third_ring_of_tris));

            Meshes::MeshTopoCache minimal_cache(patch, topo_cache().quantizer());

            Intersection::SelfIntersectionFinder se_finder(minimal_cache);

            auto si_stats = se_finder.CheckSelfIntersection();

            if (!si_stats.empty())
            {
                return false;
            }

            return true;
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

            Triangulator::Triangulator triangulator;

            for (auto vertex_index : hole.vertices())
            {
                triangulator.add_point(vertices()[vertex_index], true, projector);
            }

            for (int i = 0; i < hole.vertices().size() - 1; i++)
            {
                triangulator.add_segment(i, i + 1, true);
            }

            //            for( int i = 0; i < hole.vertices().size(); i++ )
            //            {
            //                std::cout << triangulator.points()[i].first << "  " << triangulator.points()[i].second <<
            //                std::endl;
            //            }

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
                    add_triangle(tri_to_add);
                }
            }

            //  Finished with success

            return HoleClosingResult::success();
        }

        TriangleByIndicesIndexSet find_triangles_containing_vertex(VertexIndex vertex_index)
        {
            TriangleByIndicesIndexSet triangles_including_vertex;

            for (const auto& triangle_to_add : topo_cache().vertices().getPool()[vertex_index].triangles())
            {
                triangles_including_vertex.insert(triangle_to_add->source_triangle_id());
            }

            return (triangles_including_vertex);
        }

        TriangleByIndicesIndexSet find_enclosing_triangles(const TriangleByIndicesVector& triangles)
        {
            TriangleByIndicesIndexSet triangle_set;

            for (auto triangle : triangles)
            {
                triangle_set.insert(triangle.uid());
            }

            return find_enclosing_triangles(triangle_set);
        }

        TriangleByIndicesIndexSet find_enclosing_triangles(const TriangleByIndicesIndexSet& interior_triangles)
        {
            std::vector<Hole> holes_for_se = get_hole_for_self_intersection(interior_triangles);

            //  Occasionally we may end up with multiple holes in the ring - we will just find all the
            //      enclosing triangles for however many holes we have and press on.

            if (holes_for_se.size() == 1)
            {
                return find_enclosing_triangles(holes_for_se[0], interior_triangles);
            }

            TriangleByIndicesIndexSet union_around_all_holes;

            for (const Hole& current_hole : holes_for_se)
            {
                union_around_all_holes.merge(find_enclosing_triangles(current_hole, interior_triangles));
            }

            return union_around_all_holes;
        }

        TriangleByIndicesIndexSet find_enclosing_triangles(const Hole& hole,
                                                           const TriangleByIndicesIndexSet& interior_triangles)
        {
            TriangleByIndicesIndexSet enclosing_triangles;

            for (auto vertex_index : hole.vertices())
            {
                for (auto edge : topo_cache().vertices().getPool()[vertex_index].edges())
                {
                    for (auto next_tri : edge->triangles())
                    {
                        if (!interior_triangles.contains(next_tri->source_triangle_id()))
                        {
                            enclosing_triangles.insert(next_tri->source_triangle_id());
                        }
                    }
                }
            }

            return enclosing_triangles;
        }

        TriangleByIndicesVector as_triangles(const TriangleByIndicesIndexSet& indices)
        {
            TriangleByIndicesVector result;

            for (TriangleByIndicesIndex current_index : indices)
            {
                result.emplace_back((*tris_)[current_index]);
            }

            return result;
        }

       private:
        //        std::unique_ptr<Meshes::TriangleByIndicesVectorTopoCache> topo_cache_;
    };

    class TriangleMeshWrapper : public TriangleMesh
    {
       public:
        using GeometricStatistics = Statistics::GeometricStatistics;
        using GeometricProperties = Statistics::GeometricProperties;
        using TopologicalStatistics = Statistics::TopologicalStatistics;
        using TopologicalProperties = Statistics::TopologicalProperties;

        //  Constructor/Destructor

        TriangleMeshWrapper(MeshBase&& mesh_base) : mesh_(new TriangleMeshImpl(std::move(mesh_base))) {}

        ~TriangleMeshWrapper(){};

        //	Methods follow

        size_t num_triangles() const { return mesh_->num_triangles(); }
        size_t num_vertices() const { return mesh_->num_vertices(); }

        const Vertex3DVector& vertices() const { return mesh_->vertices(); }
        const TriangleByIndicesVector& triangles() const { return mesh_->triangles(); }

        TriangleByVertices triangle_by_vertices(const TriangleByIndices& triangle_by_indices) const
        {
            return mesh_->triangle_by_vertices(triangle_by_indices);
        }

        const BBox3D& bounding_box() const { return mesh_->bounding_box(); }
        MinAndMaxEdgeLengths min_and_max_edge_lengths() const { return mesh_->min_and_max_edge_lengths(); }
        double max_vertex_magnitude() const { return mesh_->max_vertex_magnitude(); }

        GeometricStatistics ComputeGeometricStatistics(GeometricProperties props_to_compute) const
        {
            return mesh_->ComputeGeometricStatistics(props_to_compute);
        }

        TopologicalStatisticsResult ComputeTopologicalStatistics(TopologicalProperties props_to_compute) const
        {
            return mesh_->ComputeTopologicalStatistics(props_to_compute);
        }

        HoleClosingResult close_holes(const TopologicalStatistics& topo_stats)
        {
            return mesh_->close_holes(topo_stats);
        }

        SelfIntersectionResolutionResults remove_self_intersections(const TopologicalStatistics& topo_stats)
        {
            return mesh_->remove_self_intersections(topo_stats);
        }

       private:
        std::unique_ptr<TriangleMeshImpl> mesh_;
    };

    //
    //	IncrementalVertexIndexTriangleMeshBuilderImpl implements an incremental triangle mesh builder
    //		which can be used to construct a triangle mesh from a list of vertices and
    //		triangles assembled from those vertices.
    //

    class IncrementalVertexIndexTriangleMeshBuilderImpl : public IncrementalVertexIndexTriangleMeshBuilder
    {
       public:
        IncrementalVertexIndexTriangleMeshBuilderImpl(size_t num_vertices, size_t num_triangles)
            : mesh_(num_vertices, num_triangles)
        {
            vertex_index_remapper_.reserve(num_vertices);
        };

        ~IncrementalVertexIndexTriangleMeshBuilderImpl() = default;

        IncrementalVertexIndexTriangleMeshBuilderImpl() = delete;
        IncrementalVertexIndexTriangleMeshBuilderImpl(const IncrementalVertexIndexTriangleMeshBuilderImpl&) = delete;
        IncrementalVertexIndexTriangleMeshBuilderImpl(const IncrementalVertexIndexTriangleMeshBuilderImpl&&) = delete;

        IncrementalVertexIndexTriangleMeshBuilderImpl& operator=(const IncrementalVertexIndexTriangleMeshBuilderImpl&) =
            delete;
        IncrementalVertexIndexTriangleMeshBuilderImpl& operator=(
            const IncrementalVertexIndexTriangleMeshBuilderImpl&&) = delete;

        [[nodiscard]] size_t num_vertices() const final { return mesh_.num_vertices(); }

        [[nodiscard]] const Primitives::BBox3D& boundingBox() const { return mesh_.bounding_box(); }

        [[nodiscard]] double max_vertex_magnitude() const { return mesh_.max_vertex_magnitude(); }
        [[nodiscard]] Primitives::MinAndMaxEdgeLengths min_and_max_edge_lengths() const
        {
            return mesh_.min_and_max_edge_lengths();
        }

        VertexIndex add_vertex(const Primitives::Vertex3D& vertexToAdd) final
        {
            //	Add the vertex, de-duplicate on the fly.

            VertexIndexLookupMap::const_iterator vertexLoc = vertex_indices_.find(vertexToAdd);  //	NOLINT

            if (vertexLoc == vertex_indices_.end())
            {
                //	Vertex is new, update all data structures

                vertex_indices_[vertexToAdd] = mesh_.vertices().size();
                vertex_index_remapper_.push_back(VertexIndex::integer_type(mesh_.vertices().size()));
                mesh_.vertices().push_back(vertexToAdd);
            }
            else
            {
                //	Vertex is a duplicate, so remap to it

                vertex_index_remapper_.push_back(vertexLoc->second);
            }

            //	The index we return should always be the remapper size minus 1

            return (VertexIndex::integer_type(vertex_index_remapper_.size()) - 1u);
        }

        TriangleMeshBuilderResultCodes add_triangle(VertexIndex a, VertexIndex b, VertexIndex c) final
        {
            //	Insure the indices are in bounds

            if ((a >= vertex_index_remapper_.size()) || (b >= vertex_index_remapper_.size()) ||
                (c >= vertex_index_remapper_.size()))
            {
                return (TriangleMeshBuilderResultCodes::VERTEX_INDEX_OUT_OF_BOUNDS);
            }

            //	Remap the triangle indices

            TriangleByIndices remappedTriangle(mesh_.triangles().size(), vertex_index_remapper_[a],
                                               vertex_index_remapper_[b], vertex_index_remapper_[c]);

            //	Add the triangle to the vector

            mesh_.add_triangle_and_update_metrics(remappedTriangle);

            //	All is well if we made it here

            return (TriangleMeshBuilderResultCodes::SUCCESS);
        }

        std::unique_ptr<TriangleMesh> mesh() final
        {
            std::unique_ptr<TriangleMesh> return_value =
                std::unique_ptr<TriangleMesh>(new TriangleMeshWrapper(std::move(mesh_)));

            mesh_.clear();

            return return_value;
        }

       private:
        using VertexIndexLookupMap = std::map<Primitives::Vertex3D, IndexType, Primitives::Vertex3DMapCompare>;

        MeshBase mesh_;

        VertexIndexLookupMap vertex_indices_;
        Primitives::IndexRemapper<VertexIndex> vertex_index_remapper_;
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
