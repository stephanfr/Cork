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

#include "mesh/mesh_base.hpp"

//  The pragma below is to disable to false errors flagged by intellisense for Catch2 REQUIRE macros.

#if __INTELLISENSE__
#pragma diag_suppress 2486
#endif

using VertexIndex = Cork::Primitives::VertexIndex;
using EdgeByIndices = Cork::Primitives::EdgeByIndices;

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

        std::vector<EdgeByIndices> hole_edges;

        hole_edges.emplace_back(EdgeByIndices(VertexIndex(31), VertexIndex(50)));
        hole_edges.emplace_back(EdgeByIndices(VertexIndex(2), VertexIndex(3)));
        hole_edges.emplace_back(EdgeByIndices(VertexIndex(1), VertexIndex(2)));
        hole_edges.emplace_back(EdgeByIndices(VertexIndex(0), VertexIndex(1)));
        hole_edges.emplace_back(EdgeByIndices(VertexIndex(3), VertexIndex(4)));
        hole_edges.emplace_back(EdgeByIndices(VertexIndex(11), VertexIndex(12)));
        hole_edges.emplace_back(EdgeByIndices(VertexIndex(4), VertexIndex(5)));
        hole_edges.emplace_back(EdgeByIndices(VertexIndex(5), VertexIndex(6)));
        hole_edges.emplace_back(EdgeByIndices(VertexIndex(21), VertexIndex(22)));
        hole_edges.emplace_back(EdgeByIndices(VertexIndex(7), VertexIndex(0)));
        hole_edges.emplace_back(EdgeByIndices(VertexIndex(1), VertexIndex(10)));
        hole_edges.emplace_back(EdgeByIndices(VertexIndex(30), VertexIndex(31)));
        hole_edges.emplace_back(EdgeByIndices(VertexIndex(10), VertexIndex(11)));
        hole_edges.emplace_back(EdgeByIndices(VertexIndex(12), VertexIndex(13)));
        hole_edges.emplace_back(EdgeByIndices(VertexIndex(40), VertexIndex(50)));
        hole_edges.emplace_back(EdgeByIndices(VertexIndex(13), VertexIndex(1)));
        hole_edges.emplace_back(EdgeByIndices(VertexIndex(4), VertexIndex(20)));
        hole_edges.emplace_back(EdgeByIndices(VertexIndex(20), VertexIndex(21)));
        hole_edges.emplace_back(EdgeByIndices(VertexIndex(22), VertexIndex(4)));
        hole_edges.emplace_back(EdgeByIndices(VertexIndex(6), VertexIndex(7)));
        hole_edges.emplace_back(EdgeByIndices(VertexIndex(21), VertexIndex(30)));
        hole_edges.emplace_back(EdgeByIndices(VertexIndex(31), VertexIndex(21)));
        hole_edges.emplace_back(EdgeByIndices(VertexIndex(22), VertexIndex(40)));
        hole_edges.emplace_back(EdgeByIndices(VertexIndex(40), VertexIndex(41)));
        hole_edges.emplace_back(EdgeByIndices(VertexIndex(31), VertexIndex(40)));
        hole_edges.emplace_back(EdgeByIndices(VertexIndex(41), VertexIndex(22)));

        auto fake_mesh = Cork::Meshes::MeshBase(100,0);

        for( uint32_t i = 0; i < 100; i++ )
        {
            fake_mesh.vertices().emplace_back( i, i, i );
        }

        auto extract_boundaries_result = Cork::Meshes::BoundaryEdgeBuilder(fake_mesh).extract_boundaries(hole_edges);

        REQUIRE( extract_boundaries_result.succeeded() );

        std::vector<Cork::BoundaryEdge>& holes = *(extract_boundaries_result.return_ptr());

        REQUIRE(holes.size() == 6);
        REQUIRE(((holes[0].vertex_indices()[0] == VertexIndex(13)) &&
                 (holes[0].vertex_indices()[1] == VertexIndex(12)) &&
                 (holes[0].vertex_indices()[2] == VertexIndex(11)) &&
                 (holes[0].vertex_indices()[3] == VertexIndex(10)) &&
                 (holes[0].vertex_indices()[4] == VertexIndex(1))));
        REQUIRE(((holes[1].vertex_indices()[0] == VertexIndex(31)) &&
                 (holes[1].vertex_indices()[1] == VertexIndex(21)) &&
                 (holes[1].vertex_indices()[2] == VertexIndex(30))));
        REQUIRE(
            ((holes[2].vertex_indices()[0] == VertexIndex(40)) &&
            (holes[2].vertex_indices()[1] == VertexIndex(31)) &&
            (holes[2].vertex_indices()[2] == VertexIndex(50))));

        REQUIRE(((holes[3].vertex_indices()[0] == VertexIndex(4)) &&
        (holes[3].vertex_indices()[1] == VertexIndex(5)) &&
        (holes[3].vertex_indices()[2] == VertexIndex(6)) &&
                 (holes[3].vertex_indices()[3] == VertexIndex(7)) &&
                 (holes[3].vertex_indices()[4] == VertexIndex(0)) &&
                 (holes[3].vertex_indices()[5] == VertexIndex(1)) &&
                 (holes[3].vertex_indices()[6] == VertexIndex(2)) &&
                 (holes[3].vertex_indices()[7] == VertexIndex(3))));

        REQUIRE(((holes[4].vertex_indices()[0] == VertexIndex(22)) &&
        (holes[4].vertex_indices()[1] == VertexIndex(4)) &&
        (holes[4].vertex_indices()[2] == VertexIndex(20)) &&
                 (holes[4].vertex_indices()[3] == VertexIndex(21))));

        REQUIRE(
            ((holes[5].vertex_indices()[0] == VertexIndex(40)) &&
            (holes[5].vertex_indices()[1] == VertexIndex(22)) &&
            (holes[5].vertex_indices()[2] == VertexIndex(41))));
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
        REQUIRE(((stats.return_value().holes()[0].vertex_indices()[0] == VertexIndex(21)) &&
                 (stats.return_value().holes()[0].vertex_indices()[1] == VertexIndex(5)) &&
                 (stats.return_value().holes()[0].vertex_indices()[2] == VertexIndex(11))));
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
        REQUIRE(((stats.return_value().holes()[0].vertex_indices()[0] == VertexIndex(21)) &&
                 (stats.return_value().holes()[0].vertex_indices()[1] == VertexIndex(11)) &&
                 (stats.return_value().holes()[0].vertex_indices()[2] == VertexIndex(5)) &&
                 (stats.return_value().holes()[0].vertex_indices()[3] == VertexIndex(13))));
        REQUIRE(((stats.return_value().holes()[1].vertex_indices()[0] == VertexIndex(8)) &&
                 (stats.return_value().holes()[1].vertex_indices()[1] == VertexIndex(16)) &&
                 (stats.return_value().holes()[1].vertex_indices()[2] == VertexIndex(24))));
        REQUIRE(((stats.return_value().holes()[2].vertex_indices()[0] == VertexIndex(18)) &&
                 (stats.return_value().holes()[2].vertex_indices()[1] == VertexIndex(12)) &&
                 (stats.return_value().holes()[2].vertex_indices()[2] == VertexIndex(16))));
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
        REQUIRE(topo_stats_after_se_removal.return_value().self_intersecting_edges().size() == 106);

        {
            auto write_result = Cork::Files::writeOFF("../../UnitTest/Test Results/tulipRepaired.off", *mesh);

            REQUIRE(write_result.succeeded());
        }
    }
}