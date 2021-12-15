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

#include "mesh/triangle_mesh_impl.hpp"

#include <algorithm>

#include "file_formats/files.hpp"
#include "intersection/self_intersection_finder.hpp"
#include "intersection/triangulator.hpp"
#include "mesh/boundary_edge_builder.hpp"
#include "statistics/statistics_engines.hpp"

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
            auto& triangle_containing_edge = triangles()[non_manifold_edge.triangle_id()];

            VertexIndex vertex_1 = triangle_containing_edge.edge(non_manifold_edge.edge_id()).first();
            VertexIndex vertex_2 = triangle_containing_edge.edge(non_manifold_edge.edge_id()).second();

            //  Simpler than self intersections, we are just going to delete the edge and all
            //      triangles including either vertex.

            TriangleByIndicesIndexSet triangles_to_remove{std::move(find_triangles_containing_vertex(vertex_1)),
                                                          std::move(find_triangles_containing_vertex(vertex_2))};

            std::vector<BoundaryEdge> holes_for_nme = get_boundary_edge(triangles_to_remove);

            TriangleRemapper remapper(*this);

            auto surface = extract_surface(remapper, find_enclosing_triangles(triangles_to_remove));

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
//        find_self_intersecting_regions(topo_stats);

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

            TriangleByIndicesIndexSet both_vertices_and_ring_2(
                triangles_containing_both_vertices,
                find_enclosing_triangles(triangles_containing_both_vertices, 2, true));

            if (resolves_self_intersection(both_vertices_and_ring_2))
            {
                sis_resolved++;
                resolution_plan.add_resolution(both_vertices_and_ring_2);
                continue;
            }

            sis_abdondoned++;

            {
                TriangleRemapper remapper(*this);

                std::unique_ptr<MeshBase> patch(extract_surface(remapper, both_vertices_and_ring_2));

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
            std::vector<BoundaryEdge> holes_for_si = get_boundary_edge(resolution);

            if (holes_for_si.size() != 1)
            {
                failures_on_final_resolution++;
                continue;
            }

            GetHoleClosingTrianglesResult get_hole_closing_triangles_result =
                get_hole_closing_triangles(holes_for_si[0]);

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

        TriangleRemapper remapper(*this);

        std::unique_ptr<MeshBase> patch = extract_surface(remapper, full_patch);

        Meshes::MeshTopoCache minimal_cache(*patch, topo_cache().quantizer());

        Intersection::SelfIntersectionFinder si_finder(minimal_cache);

        auto si_stats = si_finder.CheckSelfIntersection();

        if (!si_stats.empty())
        {
            return false;
        }

        //  Check to insure that there are no non 2 manifold edges added as a result of the fixes

        EdgeIncidenceCounter edge_counts(*patch);

        for (const auto& edge_and_count : edge_counts.edges_and_incidences())
        {
            if (edge_and_count.numIncidences() > 2)
            {
                return false;
            }
        }

        //  Finished with success

        return true;
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

/*
    void TriangleMeshImpl::find_self_intersecting_regions(const Statistics::TopologicalStatistics& topo_stats)
    {
        std::vector<TriangleByIndicesIndexSet> all_regions;

        int i = 0;

        for (auto& intersecting_edge : topo_stats.self_intersecting_edges())
        {
            auto& triangle_containing_edge = triangles()[intersecting_edge.edge_triangle_id()];

            VertexIndex vertex_1 = triangle_containing_edge.edge(intersecting_edge.edge_index()).first();
            VertexIndex vertex_2 = triangle_containing_edge.edge(intersecting_edge.edge_index()).second();

            TriangleByIndicesIndexSet triangles_containing_vertex_1{
                std::move(find_triangles_containing_vertex(vertex_1))};

            TriangleByIndicesIndexSet triangles_containing_vertex_2{
                std::move(find_triangles_containing_vertex(vertex_2))};

            TriangleByIndicesIndexSet triangles_containing_both_vertices(triangles_containing_vertex_1,
                                                                         triangles_containing_vertex_2);

            TriangleByIndicesIndexSet si_region;

            for (int ring_size = 5; ring_size < 10; ring_size++)
            {
                si_region = find_enclosing_triangles(triangles_containing_both_vertices, ring_size, true)
                                .merge(triangles_containing_both_vertices);

                if (si_region.contains(intersecting_edge.triangle_instersected_id()))
                {
                    break;
                }
                else
                {
                    std::cout << "Increasing ring to " << ring_size << std::endl;
                }
            }

            if (!si_region.contains(intersecting_edge.triangle_instersected_id()))
            {
                std::cout << "Region " << i << " does not contain triangle intersected" << std::endl;
            }

            all_regions.emplace_back(si_region);

                        TriangleRemapper        remapper( *this );

            std::unique_ptr<MeshBase> patch = extract_surface(remapper, all_regions[i]);

            Meshes::MeshTopoCache minimal_cache(*patch, topo_cache().quantizer());

            Intersection::SelfIntersectionFinder si_finder(minimal_cache);

            auto si_stats = si_finder.CheckSelfIntersection();

            std::cout << "Initially, Region " << i << " has " << all_regions[i].size() << " triangles and "
                      << si_stats.size() << " self intersections" << std::endl;
            i++;
        }

        std::cout << "Starting with " << all_regions.size() << std::endl;

        for (int i = 0; i < all_regions.size(); i++)
        {
            for (int j = i + 1; j < all_regions.size(); j++)
            {
                if (all_regions[i].intersects(all_regions[j]))
                {
                    all_regions[i].merge(all_regions[j]);
                    all_regions.erase(all_regions.begin() + j);
                    j--;
                }
            }
        }

        std::cout << "Finished with " << all_regions.size() << std::endl;

        int total_sis = 0;

        for (int i = 0; i < all_regions.size(); i++)
        {
            TriangleRemapper        remapper( *this );

            std::unique_ptr<MeshBase> patch = extract_surface(remapper, all_regions[i]);

            Meshes::MeshTopoCache minimal_cache(*patch, topo_cache().quantizer());

            Intersection::SelfIntersectionFinder si_finder(minimal_cache);

            auto si_stats = si_finder.CheckSelfIntersection();

            std::cout << "Region " << i << " has " << all_regions[i].size() << " triangles and " << si_stats.size()
                      << " self intersections" << std::endl;

            total_sis += si_stats.size();
        }

        std::cout << "Total num sis " << total_sis << std::endl;

        for (int i = 0; i < all_regions.size(); i++)
        {
            TriangleRemapper        remapper( *this );

            std::unique_ptr<MeshBase> patch = extract_surface(remapper, all_regions[i]);

            Cork::Files::writeOFF("../../UnitTest/Test Results/patch.off", *patch);


            Meshes::MeshTopoCache minimal_cache(*patch, topo_cache().quantizer());

            Intersection::SelfIntersectionFinder si_finder(minimal_cache);

            auto si_stats = si_finder.CheckSelfIntersection();

            for (auto& intersecting_edge : si_stats)
            {
                auto& triangle_containing_edge = patch->triangles()[intersecting_edge.edge_triangle_id()];

                VertexIndex vertex_1 = triangle_containing_edge.edge(intersecting_edge.edge_index()).first();
                VertexIndex vertex_2 = triangle_containing_edge.edge(intersecting_edge.edge_index()).second();

                //  Try to fix the self intersection by removing different sets of triangles:
                //
                //  1)  First SE vertex only
                //  2)  Second SE vertex only
                //  3)  First and Second vertices together
                //  4)  First and Second vertices with the enclosing ring



                TriangleByIndicesIndexSet triangles_containing_vertex_1{
                    std::move(patch->find_triangles_containing_vertex(vertex_1))};

                TriangleByIndicesIndexSet triangles_containing_vertex_2{
                    std::move(patch->find_triangles_containing_vertex(vertex_2))};


            TriangleByIndicesIndexSet triangles_containing_both_vertices(triangles_containing_vertex_1,
                                                                         triangles_containing_vertex_2);

            TriangleByIndicesIndexSet both_vertices_and_ring(
                triangles_containing_both_vertices, patch->find_enclosing_triangles(triangles_containing_both_vertices, 2, true));

                std::vector<BoundaryEdge> holes_for_se = patch->get_boundary_edge(both_vertices_and_ring);

                GetHoleClosingTrianglesResult get_hole_closing_triangles_result =
                    patch->get_hole_closing_triangles(holes_for_se[0]);

                if (!get_hole_closing_triangles_result.succeeded())
                {
                    std::cout << "could not close hole" << std::endl;
                    continue;
                }

                std::cout << "Num Tris before removal " << patch->triangles().size() << std::endl;

                for (auto triangle_itr = patch->triangles().begin(); triangle_itr != patch->triangles().end();
                     triangle_itr++)
                {
                    if (both_vertices_and_ring.contains(triangle_itr->uid()))
                    {
                        triangle_itr = patch->triangles().erase(triangle_itr);
                        
                        if( triangle_itr == patch->triangles().end() )
                        {
                            break;
                        }
                    }
                }

                Cork::Files::writeOFF("../../UnitTest/Test Results/patch_with_holes.off", *patch);

                std::cout << "Num Tris after removal " << patch->triangles().size() << std::endl;


                for (auto tri_to_add : *(get_hole_closing_triangles_result.return_ptr()))
                {
                    patch->triangles().emplace_back(tri_to_add);
//                    remapper.remap_into_mesh( *patch, tri_to_add );
                }

                
                std::cout << "Num Tris after additions " << patch->triangles().size() << std::endl;

            }

                Cork::Files::writeOFF("../../UnitTest/Test Results/patch_with_fixes.off", *patch);

            patch->clear_topo_cache();

            Meshes::MeshTopoCache minimal_cache2(*patch, topo_cache().quantizer());

            Intersection::SelfIntersectionFinder si_finder2(minimal_cache2);

            auto si_stats2 = si_finder2.CheckSelfIntersection();

            std::cout << "After punching out vertex1 intersections, patch contains " << si_stats2.size() << std::endl;
        }
    }
*/
}  // namespace Cork::Meshes