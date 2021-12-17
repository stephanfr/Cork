/*
 Copyright (c) 2021 Stephan Friedl

 Permission is hereby granted, free of charge, to any person obtaining a copy of
 this software and associated documentation files (the "Software"), to deal in
 the Software without restriction, including without limitation the rights to
 use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 the Software, and to permit persons to whom the Software is furnished to do so,
 subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <catch2/catch_all.hpp>
#include <list>

#include "file_formats/files.hpp"
#include "intersection/triangulator.hpp"
#include "mesh/boundary_edge_builder.hpp"

//  The pragma below is to disable to false errors flagged by intellisense for Catch2 REQUIRE macros.

#if __INTELLISENSE__
#pragma diag_suppress 2486
#endif

TEST_CASE("Topology Tests", "[file io]")
{
    SECTION("2 Manifold Single Body")
    {
        auto read_result = Cork::Files::readOFF("../../UnitTest/Test Files/Quadrilateral1.off");

        REQUIRE(read_result.succeeded());

        auto* mesh(read_result.return_ptr().release());

        REQUIRE(mesh->num_vertices() == 26);
        REQUIRE(mesh->num_triangles() == 48);

        auto stats = mesh->ComputeTopologicalStatistics(Cork::Statistics::TopologicalProperties::TOPO_ALL);

        REQUIRE(stats.succeeded());
        REQUIRE(stats.return_value().non_manifold_edges().size() == 0);
        REQUIRE(stats.return_value().num_edges() == 72);
    }

    SECTION("HoleBuilder Test")
    {
        //  Create a list with a bunch of edges that form a bunch of holes sharing vertices.
        //      If you change any ordering of the entries in the list, the ordering of the holes and the ordering
        //      of the vertices forming the hole may change as well.  The holes will be correct - but ordering may
        //      differ.

        std::vector<Cork::Primitives::EdgeByIndices> hole_edges;

        hole_edges.emplace_back(Cork::Primitives::EdgeByIndices(31u, 50u));
        hole_edges.emplace_back(Cork::Primitives::EdgeByIndices(2u, 3u));
        hole_edges.emplace_back(Cork::Primitives::EdgeByIndices(1u, 2u));
        hole_edges.emplace_back(Cork::Primitives::EdgeByIndices(0u, 1u));
        hole_edges.emplace_back(Cork::Primitives::EdgeByIndices(3u, 4u));
        hole_edges.emplace_back(Cork::Primitives::EdgeByIndices(11u, 12u));
        hole_edges.emplace_back(Cork::Primitives::EdgeByIndices(4u, 5u));
        hole_edges.emplace_back(Cork::Primitives::EdgeByIndices(5u, 6u));
        hole_edges.emplace_back(Cork::Primitives::EdgeByIndices(21u, 22u));
        hole_edges.emplace_back(Cork::Primitives::EdgeByIndices(7u, 0u));
        hole_edges.emplace_back(Cork::Primitives::EdgeByIndices(1u, 10u));
        hole_edges.emplace_back(Cork::Primitives::EdgeByIndices(30u, 31u));
        hole_edges.emplace_back(Cork::Primitives::EdgeByIndices(10u, 11u));
        hole_edges.emplace_back(Cork::Primitives::EdgeByIndices(12u, 13u));
        hole_edges.emplace_back(Cork::Primitives::EdgeByIndices(40u, 50u));
        hole_edges.emplace_back(Cork::Primitives::EdgeByIndices(13u, 1u));
        hole_edges.emplace_back(Cork::Primitives::EdgeByIndices(4u, 20u));
        hole_edges.emplace_back(Cork::Primitives::EdgeByIndices(20u, 21u));
        hole_edges.emplace_back(Cork::Primitives::EdgeByIndices(22u, 4u));
        hole_edges.emplace_back(Cork::Primitives::EdgeByIndices(6u, 7u));
        hole_edges.emplace_back(Cork::Primitives::EdgeByIndices(21u, 30u));
        hole_edges.emplace_back(Cork::Primitives::EdgeByIndices(31u, 21u));
        hole_edges.emplace_back(Cork::Primitives::EdgeByIndices(22u, 40u));
        hole_edges.emplace_back(Cork::Primitives::EdgeByIndices(40u, 41u));
        hole_edges.emplace_back(Cork::Primitives::EdgeByIndices(31u, 40u));
        hole_edges.emplace_back(Cork::Primitives::EdgeByIndices(41u, 22u));

        auto extract_boundaries_result = Cork::Meshes::BoundaryEdgeBuilder().extract_boundaries(hole_edges);

        REQUIRE( extract_boundaries_result.succeeded() );

        std::vector<Cork::BoundaryEdge>& holes = *(extract_boundaries_result.return_ptr());

        REQUIRE(holes.size() == 6);
        REQUIRE(((holes[0].vertices()[0] == 13u) && (holes[0].vertices()[1] == 12u) &&
                 (holes[0].vertices()[2] == 11u) && (holes[0].vertices()[3] == 10ul) &&
                 (holes[0].vertices()[4] == 1ul)));
        REQUIRE(
            ((holes[1].vertices()[0] == 31u) && (holes[1].vertices()[1] == 21u) && (holes[1].vertices()[2] == 30u)));
        REQUIRE(
            ((holes[2].vertices()[0] == 40u) && (holes[2].vertices()[1] == 31u) && (holes[2].vertices()[2] == 50u)));
        REQUIRE(((holes[3].vertices()[0] == 4u) && (holes[3].vertices()[1] == 5u) && (holes[3].vertices()[2] == 6u) &&
                 (holes[3].vertices()[3] == 7ul) && (holes[3].vertices()[4] == 0ul) &&
                 (holes[3].vertices()[5] == 1ul) && (holes[3].vertices()[6] == 2ul) &&
                 (holes[3].vertices()[7] == 3ul)));
        REQUIRE(((holes[4].vertices()[0] == 22u) && (holes[4].vertices()[1] == 4u) && (holes[4].vertices()[2] == 20u) &&
                 (holes[4].vertices()[3] == 21ul)));
        REQUIRE(
            ((holes[5].vertices()[0] == 40u) && (holes[5].vertices()[1] == 22u) && (holes[5].vertices()[2] == 41u)));
    }

    SECTION("Find Holes - One Hole")
    {
        auto read_result = Cork::Files::readOFF("../../UnitTest/Test Files/Quadrilateral with Hole.off");

        REQUIRE(read_result.succeeded());

        auto* mesh(read_result.return_ptr().release());

        REQUIRE(mesh->num_vertices() == 26);
        REQUIRE(mesh->num_triangles() == 47);

        auto stats = mesh->ComputeTopologicalStatistics(Cork::Statistics::TopologicalProperties::TOPO_ALL);

        REQUIRE(stats.succeeded());
        REQUIRE(stats.return_value().non_manifold_edges().size() != 0);
        REQUIRE(stats.return_value().holes().size() == 1);
        REQUIRE(((stats.return_value().holes()[0].vertices()[0] == 21u) &&
                 (stats.return_value().holes()[0].vertices()[1] == 5u) &&
                 (stats.return_value().holes()[0].vertices()[2] == 11u)));
        REQUIRE(stats.return_value().self_intersecting_edges().size() == 0);
        //        REQUIRE(stats.numBodies() == 1);
    }

    SECTION("Find Holes - Three Holes")
    {
        auto read_result = Cork::Files::readOFF("../../UnitTest/Test Files/Quadrilateral with 3 Holes.off");

        REQUIRE(read_result.succeeded());

        auto* mesh(read_result.return_ptr().release());

        REQUIRE(mesh->num_vertices() == 26);
        REQUIRE(mesh->num_triangles() == 44);

        auto stats = mesh->ComputeTopologicalStatistics(Cork::Statistics::TopologicalProperties::TOPO_ALL);

        REQUIRE(stats.succeeded());
        REQUIRE(stats.return_value().non_manifold_edges().size() != 0);
        REQUIRE(stats.return_value().holes().size() == 3);
        REQUIRE(((stats.return_value().holes()[0].vertices()[0] == 21u) &&
                 (stats.return_value().holes()[0].vertices()[1] == 11u) &&
                 (stats.return_value().holes()[0].vertices()[2] == 5u) &&
                 (stats.return_value().holes()[0].vertices()[3] == 13u)));
        REQUIRE(((stats.return_value().holes()[1].vertices()[0] == 8u) &&
                 (stats.return_value().holes()[1].vertices()[1] == 16u) &&
                 (stats.return_value().holes()[1].vertices()[2] == 24u)));
        REQUIRE(((stats.return_value().holes()[2].vertices()[0] == 18u) &&
                 (stats.return_value().holes()[2].vertices()[1] == 12u) &&
                 (stats.return_value().holes()[2].vertices()[2] == 16u)));
        REQUIRE(stats.return_value().self_intersecting_edges().size() == 0);
        //        REQUIRE(stats.numBodies() == 1);
    }
/*
    SECTION("Find and Fix Self Intersections - Simple")
    {
        auto read_result = Cork::Files::readOFF("../../UnitTest/Test Files/JuliaVaseWithSelfIntersection.off");

        REQUIRE(read_result.succeeded());

        auto* mesh(read_result.return_ptr().release());

        auto topo_stats = mesh->ComputeTopologicalStatistics(Cork::Statistics::TopologicalProperties::TOPO_ALL);

        REQUIRE(topo_stats.succeeded());
        REQUIRE(topo_stats.return_value().non_manifold_edges().size() == 0);
        REQUIRE(topo_stats.return_value().holes().size() == 0);
        REQUIRE(topo_stats.return_value().self_intersecting_edges().size() == 1);

        mesh->remove_self_intersections(topo_stats.return_value());

        {
            auto write_result = Cork::Files::writeOFF("../../UnitTest/Test Results/JuliaVaseRepaired.off", *mesh);

            REQUIRE(write_result.succeeded());
        }

        auto topo_stats_after_se_removal =
            mesh->ComputeTopologicalStatistics(Cork::Statistics::TopologicalProperties::TOPO_ALL);

        REQUIRE(topo_stats_after_se_removal.succeeded());
        REQUIRE(topo_stats_after_se_removal.return_value().non_manifold_edges().size() == 0);
        REQUIRE(topo_stats_after_se_removal.return_value().holes().size() == 0);
        REQUIRE(topo_stats_after_se_removal.return_value().self_intersecting_edges().size() == 0);
    }

    SECTION("Find and Fix Self Intersections - Medium")
    {
        auto read_result = Cork::Files::readOFF("../../UnitTest/Test Files/Schoen_16WithSelfIntersections.off");

        REQUIRE(read_result.succeeded());

        std::unique_ptr<Cork::TriangleMesh> mesh(read_result.return_ptr().release());

        auto topo_stats = mesh->ComputeTopologicalStatistics(Cork::Statistics::TopologicalProperties::TOPO_ALL);

        REQUIRE(topo_stats.succeeded());
        REQUIRE(topo_stats.return_value().non_manifold_edges().size() == 0);
        REQUIRE(topo_stats.return_value().holes().size() == 0);
        REQUIRE(topo_stats.return_value().self_intersecting_edges().size() == 4);

        mesh->remove_self_intersections(topo_stats.return_value());

        auto topo_stats_after_se_removal =
            mesh->ComputeTopologicalStatistics(Cork::Statistics::TopologicalProperties::TOPO_ALL);

        REQUIRE(topo_stats_after_se_removal.succeeded());
        REQUIRE(topo_stats_after_se_removal.return_value().non_manifold_edges().size() == 0);
        REQUIRE(topo_stats_after_se_removal.return_value().holes().size() == 0);
        REQUIRE(topo_stats_after_se_removal.return_value().self_intersecting_edges().size() == 0);

        {
            auto write_result = Cork::Files::writeOFF("../../UnitTest/Test Results/Schoen_16Repaired.off", *mesh);

            REQUIRE(write_result.succeeded());
        }
    }

    SECTION("Find and Fix Self Intersections - Hard")
    {
        auto read_result = Cork::Files::readOFF("../../UnitTest/Test Files/bladeWithSelfIntersections.off");

        REQUIRE(read_result.succeeded());

        std::unique_ptr<Cork::TriangleMesh> mesh(read_result.return_ptr().release());

        auto topo_stats = mesh->ComputeTopologicalStatistics(Cork::Statistics::TopologicalProperties::TOPO_ALL);

        REQUIRE(topo_stats.succeeded());
        REQUIRE(topo_stats.return_value().non_manifold_edges().size() == 0);
        REQUIRE(topo_stats.return_value().holes().size() == 0);
        REQUIRE(topo_stats.return_value().self_intersecting_edges().size() == 22);

        mesh->remove_self_intersections(topo_stats.return_value());

        auto topo_stats_after_se_removal =
            mesh->ComputeTopologicalStatistics(Cork::Statistics::TopologicalProperties::TOPO_ALL);

        REQUIRE(topo_stats_after_se_removal.succeeded());
        REQUIRE(topo_stats_after_se_removal.return_value().non_manifold_edges().size() == 0);
        REQUIRE(topo_stats_after_se_removal.return_value().holes().size() == 0);
        REQUIRE(topo_stats_after_se_removal.return_value().self_intersecting_edges().size() == 0);

        {
            auto write_result = Cork::Files::writeOFF("../../UnitTest/Test Results/bladeRepaired.off", *mesh);

            REQUIRE(write_result.succeeded());
        }
    }
*/
    SECTION("Find and Fix Self Intersections - Harder")
    {
        auto read_result = Cork::Files::readOFF("../../UnitTest/Test Files/TulipWithSelfIntersections.off");

        REQUIRE(read_result.succeeded());

        std::unique_ptr<Cork::TriangleMesh> mesh(read_result.return_ptr().release());

        auto topo_stats = mesh->ComputeTopologicalStatistics(Cork::Statistics::TopologicalProperties::TOPO_ALL);

        REQUIRE(topo_stats.succeeded());
        REQUIRE(topo_stats.return_value().non_manifold_edges().size() == 0);
        REQUIRE(topo_stats.return_value().holes().size() == 0);
        REQUIRE(topo_stats.return_value().self_intersecting_edges().size() == 178);

        mesh->remove_self_intersections(topo_stats.return_value());

        auto topo_stats_after_se_removal =
            mesh->ComputeTopologicalStatistics(Cork::Statistics::TopologicalProperties::TOPO_ALL);

        REQUIRE(topo_stats_after_se_removal.succeeded());
        REQUIRE(topo_stats_after_se_removal.return_value().non_manifold_edges().size() == 0);
        REQUIRE(topo_stats_after_se_removal.return_value().holes().size() == 0);
        REQUIRE(topo_stats_after_se_removal.return_value().self_intersecting_edges().size() == 104);

        {
            auto write_result = Cork::Files::writeOFF("../../UnitTest/Test Results/tulipRepaired.off", *mesh);

            REQUIRE(write_result.succeeded());
        }
    }
}