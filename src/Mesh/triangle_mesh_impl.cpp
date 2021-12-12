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

#include <algorithm>

#include "mesh/triangle_mesh_impl.hpp"

#include "file_formats/files.hpp"
#include "intersection/self_intersection_finder.hpp"
#include "intersection/triangulator.hpp"
#include "mesh/boundary_edge_builder.hpp"
#include "statistics/statistics_engines.hpp"
#include "mesh/surface_mesh.hpp"

namespace Cork::Meshes
{
    using GeometricStatisticsEngine = Statistics::GeometricStatisticsEngine;
    using GeometricProperties = Statistics::GeometricProperties;
    using GeometricStatistics = Statistics::GeometricStatistics;

    using TopologicalStatisticsEngine = Statistics::TopologicalStatisticsEngine;
    using TopologicalProperties = Statistics::TopologicalProperties;
    using TopologicalStatisticsResultCodes = Statistics::TopologicalStatisticsResultCodes;
    using TopologicalStatisticsResult = Statistics::TopologicalStatisticsResult;

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

    [[nodiscard]] GeometricStatistics TriangleMeshImpl::ComputeGeometricStatistics(
        GeometricProperties props_to_compute) const
    {
        Statistics::GeometricStatisticsEngine geometricStatsEngine(*this, props_to_compute);

        return GeometricStatistics(geometricStatsEngine.statistics());
    }

    [[nodiscard]] TopologicalStatisticsResult TriangleMeshImpl::ComputeTopologicalStatistics(
        TopologicalProperties props_to_compute) const
    {
        TopologicalStatisticsEngine statsEngine(*this);

        auto result = statsEngine.Analyze(props_to_compute);

        if (result.failed())
        {
            return TopologicalStatisticsResult::failure(result, TopologicalStatisticsResultCodes::ANALYSIS_FAILED,
                                                        "Topological Analysis Failed");
        }

        return TopologicalStatisticsResult::success(result.return_value());
    }

    void TriangleMeshImpl::remove_non_manifold_edges(const Statistics::TopologicalStatistics& topo_stats)
    {
        //  Use a set below, as this will deduplicate and the ordering will insure we remove triangles
        //      from the end of the vector moving forward.  This prevents corruption of the vector
        //      by having the vector compress with removals and then having the wrong element removed by index.

        std::set<TriangleByIndicesIndex, std::greater<TriangleByIndicesIndex>> all_triangles_to_remove;
        std::vector<TriangleByIndices> all_triangles_to_add;

        for (auto& non_manifold_edge : topo_stats.non_manifold_edges())
        {
            auto& triangle_containing_edge = triangles()[non_manifold_edge.triangle_id_];

            VertexIndex vertex_1 = triangle_containing_edge.edge(non_manifold_edge.edge_id_).first();
            VertexIndex vertex_2 = triangle_containing_edge.edge(non_manifold_edge.edge_id_).second();

            //  Simpler than self intersections, we are just going to delete the edge and all
            //      triangles including either vertex.

            TriangleByIndicesIndexSet triangles_to_remove{std::move(find_triangles_containing_vertex(vertex_1)),
                                                          std::move(find_triangles_containing_vertex(vertex_2))};

            std::vector<BoundaryEdge> holes_for_nme = get_boundary_edge(triangles_to_remove);

            auto surface = extract_surface(find_enclosing_triangles(triangles_to_remove));

            Cork::Files::writeOFF("../../UnitTest/Test Results/nm_edge.off", *surface);

            TriangleByIndicesVector nme_closing_tris;

            bool hole_closing_failed = false;

            for (auto const& hole : holes_for_nme)
            {
                GetHoleClosingTrianglesResult get_hole_closing_triangles_result = get_hole_closing_triangles(hole);

                if (!get_hole_closing_triangles_result.succeeded())
                {
                    std::cout << "Get hole closing triangles failed" << std::endl;
                    hole_closing_failed = true;
                    break;
                }

                for (const auto& tri : *(get_hole_closing_triangles_result.return_ptr()))
                {
                    nme_closing_tris.emplace_back(tri);
                }
            }

            if (hole_closing_failed)
            {
                continue;
            }

            for (auto const& tri : triangles_to_remove)
            {
                all_triangles_to_remove.emplace(tri);
            }

            for (const auto& tri : nme_closing_tris)
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
    }

    SelfIntersectionResolutionResults TriangleMeshImpl::remove_self_intersections(
        const Statistics::TopologicalStatistics& topo_stats)
    {
        uint32_t sis_resolved = 0;
        uint32_t sis_abdondoned = 0;
        uint32_t failures_on_final_resolution = 0;

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

            if (resolves_self_intersection(triangles_containing_vertex_1))
            {
                sis_resolved++;
                resolution_plan.add_resolution(triangles_containing_vertex_1);
                continue;
            }

            TriangleByIndicesIndexSet triangles_containing_vertex_2{
                std::move(find_triangles_containing_vertex(vertex_2))};

            if (resolves_self_intersection(triangles_containing_vertex_2))
            {
                sis_resolved++;
                resolution_plan.add_resolution(triangles_containing_vertex_2);
                continue;
            }

            TriangleByIndicesIndexSet triangles_containing_both_vertices(triangles_containing_vertex_1,
                                                                         triangles_containing_vertex_2);

            if (resolves_self_intersection(triangles_containing_both_vertices))
            {
                sis_resolved++;
                resolution_plan.add_resolution(triangles_containing_both_vertices);
                continue;
            }

            TriangleByIndicesIndexSet both_vertices_and_ring(
                triangles_containing_both_vertices, find_enclosing_triangles(triangles_containing_both_vertices));

            if (resolves_self_intersection(both_vertices_and_ring))
            {
                sis_resolved++;
                resolution_plan.add_resolution(both_vertices_and_ring);
                continue;
            }

            sis_abdondoned++;

            {
                std::unique_ptr<MeshBase> patch(this->extract_surface(both_vertices_and_ring));

                auto write_result = Cork::Files::writeOFF("../../UnitTest/Test Results/patch.off", *patch);
            }
        }

        //  Use a set below, as this will deduplicate and the ordering will insure we remove triangles
        //      from the end of the vector moving forward.  This prevents corruption of the vector
        //      by having the vector compress with removals and then having the wrong element removed by index.

        std::set<TriangleByIndicesIndex, std::greater<TriangleByIndicesIndex>> all_triangles_to_remove;
        std::vector<TriangleByIndices> all_triangles_to_add;

        all_triangles_to_add.reserve(1000);

        for (const auto& resolution : resolution_plan.resolutions())
        {
            std::vector<BoundaryEdge> holes_for_se = get_boundary_edge(resolution);

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

        return SelfIntersectionResolutionResults(sis_resolved, sis_abdondoned, failures_on_final_resolution);
    }

    bool TriangleMeshImpl::resolves_self_intersection(const TriangleByIndicesIndexSet& tris_to_remove)
    {
        std::vector<BoundaryEdge> holes_for_se = get_boundary_edge(tris_to_remove);

        if (holes_for_se.size() != 1)
        {
            return false;
        }

        GetHoleClosingTrianglesResult get_hole_closing_triangles_result = get_hole_closing_triangles(holes_for_se[0]);

        if (!get_hole_closing_triangles_result.succeeded())
        {
            return false;
        }

        auto ring_of_tris = find_enclosing_triangles(holes_for_se[0], tris_to_remove, 3, true);

        TriangleByIndicesVector full_patch{*(get_hole_closing_triangles_result.return_ptr())};

        for (const auto& tri_to_add : as_triangles(ring_of_tris))
        {
            full_patch.emplace_back(tri_to_add);
        }

        std::unique_ptr<MeshBase> patch = this->extract_surface(full_patch);

        Meshes::MeshTopoCache minimal_cache(*patch, topo_cache().quantizer());

        Intersection::SelfIntersectionFinder se_finder(minimal_cache);

        auto si_stats = se_finder.CheckSelfIntersection();

        if (!si_stats.empty())
        {
            return false;
        }

        //  Check to insure that there are no non 2 manifold edges added as a result of the fixes

        if( TriangleByIndicesIndex::integer_type( *(tris_to_remove.begin()) ) > 400000 )
        {
            EdgeIncidenceCounter        edge_counts( *patch );

            for( const auto& edge_and_count : edge_counts.edges_and_incidences() )
            {
                if( edge_and_count.numIncidences() > 2 )
                {
                    return false;
                }
            }
        }

        //  Finished with success

        return true;
    }

    std::vector<BoundaryEdge> TriangleMeshImpl::get_boundary_edge(
        const TriangleByIndicesIndexSet& tris_to_outline) const
    {
        return BoundaryEdgeBuilder().extract_boundaries(*this, tris_to_outline);
    }

    TriangleMeshImpl::GetHoleClosingTrianglesResult TriangleMeshImpl::get_hole_closing_triangles(
        const BoundaryEdge& hole)
    {
        //  Determine the projection needed to turn this into a 2D triangulation problem

        Cork::Triangulator::NormalProjector projector(vertices()[hole.vertices()[0]], vertices()[hole.vertices()[1]],
                                                      vertices()[hole.vertices()[2]]);

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

        std::unique_ptr<TriangleByIndicesVector> hole_closing_triangles = std::make_unique<TriangleByIndicesVector>();

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

    HoleClosingResult TriangleMeshImpl::close_holes(const Statistics::TopologicalStatistics& topo_stats)
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

    TriangleByIndicesIndexSet TriangleMeshImpl::find_enclosing_triangles(const TriangleByIndicesVector& triangles,
                                                                         uint32_t num_layers, bool smooth_boundary)
    {
        TriangleByIndicesIndexSet triangle_set;

        for (auto triangle : triangles)
        {
            triangle_set.insert(triangle.uid());
        }

        return find_enclosing_triangles(triangle_set, smooth_boundary);
    }

    TriangleByIndicesIndexSet TriangleMeshImpl::find_enclosing_triangles(
        const TriangleByIndicesIndexSet& interior_triangles, uint32_t num_layers, bool smooth_boundary)
    {
        std::vector<BoundaryEdge> boundaries = get_boundary_edge(interior_triangles);

        //  Occasionally we may end up with multiple boundaries - we will just find all the
        //      enclosing triangles for however many boundaries we have and press on.

        if (boundaries.size() == 1)
        {
            return find_enclosing_triangles(boundaries[0], interior_triangles, num_layers, smooth_boundary);
        }

        TriangleByIndicesIndexSet union_of_all_boundaries;

        for (const BoundaryEdge& current_boundary : boundaries)
        {
            union_of_all_boundaries.merge(find_enclosing_triangles(current_boundary, interior_triangles));
        }

        return union_of_all_boundaries;
    }

    TriangleByIndicesIndexSet TriangleMeshImpl::find_enclosing_triangles(
        const BoundaryEdge& boundary, const TriangleByIndicesIndexSet& interior_triangles, uint32_t num_layers,
                                                           bool smooth_boundary )
    {
        std::vector<BoundaryEdge> current_boundaries{boundary};
        TriangleByIndicesIndexSet all_enclosing_triangles;

        for (int i = 0; i < num_layers; i++)
        {
            TriangleByIndicesIndexSet enclosing_triangles;

            for (const auto& boundary : current_boundaries)
            {
                for (auto vertex_index : boundary.vertices())
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
            }

            //  Merge all the triangles into one master set

            all_enclosing_triangles.merge(enclosing_triangles);

            //  Compute the new boundary, then smooth it

            if( smooth_boundary )
            {

                TriangleByIndicesIndexSet all_interior_triangles(interior_triangles, all_enclosing_triangles);
                current_boundaries = get_boundary_edge(all_interior_triangles);

                //  One special case here - look for triangles whose 3 vertices appear in order on the
                //      boundary edge that are not already in the interior triangle set.  Add these triangles
                //      to the set and this will smooth out the boundary.

                bool recompute_boundary = false;

                for (const auto& boundary : current_boundaries)
                {
                    for (uint32_t i = 0; i < boundary.vertices().size() - 2; i++)
                    {
                        std::optional<TriangleByIndicesIndex> all_three = tri_containing_all_three_vertices( boundary.vertices()[i], boundary.vertices()[(i + 1)], boundary.vertices()[(i + 2)] );

                        if (all_three)
                        {
                            //  We found a triangle to smooth the edge, add it.  We will also need to recompute the boundary.

                            all_enclosing_triangles.emplace(all_three.value());
                            all_interior_triangles.emplace(all_three.value());

                            recompute_boundary = true;
                        }
                    }
                }

                if (recompute_boundary)
                {
                    current_boundaries = get_boundary_edge(all_interior_triangles);
                }
            }
            else if( i < num_layers - 1 )
            {
                TriangleByIndicesIndexSet all_interior_triangles(interior_triangles, all_enclosing_triangles);
                current_boundaries = get_boundary_edge(all_interior_triangles);
            }
        }

        return all_enclosing_triangles;
    }

    inline std::optional<TriangleByIndicesIndex> TriangleMeshImpl::tri_containing_all_three_vertices(VertexIndex vert1,
                                                                                                     VertexIndex vert2,
                                                                                                     VertexIndex vert3)
    {
        std::vector<TopoTri*> tris_containing_1_and_2;

        std::set_intersection(topo_cache().vertices().getPool()[vert1].triangles().begin(),
                              topo_cache().vertices().getPool()[vert1].triangles().end(),
                              topo_cache().vertices().getPool()[vert2].triangles().begin(),
                              topo_cache().vertices().getPool()[vert2].triangles().end(),
                              std::back_inserter(tris_containing_1_and_2));

        std::vector<TopoTri*> tris_containing_all_3;

        std::set_intersection(tris_containing_1_and_2.begin(), tris_containing_1_and_2.end(),
                              topo_cache().vertices().getPool()[vert3].triangles().begin(),
                              topo_cache().vertices().getPool()[vert3].triangles().end(),
                              std::back_inserter(tris_containing_all_3));

        std::optional<TriangleByIndicesIndex>   result;

        if (!tris_containing_all_3.empty())
        {
            //  There should only ever be one ....
            result = (*(tris_containing_all_3.begin()))->source_triangle_id();
        }

        return result;
    }

}  // namespace Cork::Meshes