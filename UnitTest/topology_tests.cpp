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

#include "file_formats/files.h"

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

        REQUIRE(mesh->numVertices() == 26);
        REQUIRE(mesh->numTriangles() == 48);

        auto stats = mesh->ComputeTopologicalStatistics();
        //        REQUIRE(stats.numBodies() == 1);
        REQUIRE(stats.is_two_manifold());
        REQUIRE(stats.num_edges() == 72);
    }

    SECTION("HoleBuilder Test")
    {
        //  Create a list with a bunch of edges that form a bunch of holes sharing vertices.
        //      If you change any ordering of the entries in the list, the ordering of the holes and the ordering
        //      of the vertices forming the hole may change as well.  The holes will be correct - but ordering may differ.

        std::vector<Cork::Math::EdgeByIndices>    hole_edges;
        
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 31u, 50u ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 2u, 3u ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 1u, 2u ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 0u, 1u ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 3u, 4u ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 11u, 12u ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 4u, 5u ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 5u, 6u ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 21u, 22u ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 7u, 0u ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 1u, 10u ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 30u, 31u ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 10u, 11u ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 12u, 13u ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 40u, 50u ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 13u, 1u ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 4u, 20u ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 20u, 21u ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 22u, 4u ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 6u, 7u ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 21u, 30u ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 31u, 21u ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 22u, 40u ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 40u, 41u ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 31u, 40u ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 41u, 22u ) );

        std::vector<Cork::Meshes::Hole> holes = Cork::Meshes::HoleBuilder::extract_holes( hole_edges );

        REQUIRE( holes.size() == 6 );
        REQUIRE((( holes[0].vertices()[0] == 13u ) && ( holes[0].vertices()[1] == 12u ) && ( holes[0].vertices()[2] == 11u ) && ( holes[0].vertices()[3] == 10ul ) && ( holes[0].vertices()[4] == 1ul )));
        REQUIRE((( holes[1].vertices()[0] == 31u ) && ( holes[1].vertices()[1] == 21u ) && ( holes[1].vertices()[2] == 30u )));
        REQUIRE((( holes[2].vertices()[0] == 40u ) && ( holes[2].vertices()[1] == 31u ) && ( holes[2].vertices()[2] == 50u )));
        REQUIRE((( holes[3].vertices()[0] == 4u ) && ( holes[3].vertices()[1] == 5u ) && ( holes[3].vertices()[2] == 6u ) && ( holes[3].vertices()[3] == 7ul ) && ( holes[3].vertices()[4] == 0ul ) && ( holes[3].vertices()[5] == 1ul ) && ( holes[3].vertices()[6] == 2ul ) && ( holes[3].vertices()[7] == 3ul )));
        REQUIRE((( holes[4].vertices()[0] == 22u ) && ( holes[4].vertices()[1] == 4u ) && ( holes[4].vertices()[2] == 20u ) && ( holes[4].vertices()[3] == 21ul )));
        REQUIRE((( holes[5].vertices()[0] == 40u ) && ( holes[5].vertices()[1] == 22u ) && ( holes[5].vertices()[2] == 41u )));
    }

    SECTION("Find Holes - One Hole")
    {
        auto read_result = Cork::Files::readOFF("../../UnitTest/Test Files/Quadrilateral with Hole.off");

        REQUIRE(read_result.succeeded());

        auto* mesh(read_result.return_ptr().release());

        REQUIRE(mesh->numVertices() == 26);
        REQUIRE(mesh->numTriangles() == 47);

        auto stats = mesh->ComputeTopologicalStatistics();

        REQUIRE(!stats.is_two_manifold());
        REQUIRE(stats.holes().size() == 1);
        REQUIRE(((stats.holes()[0].vertices()[0] == 21u) && (stats.holes()[0].vertices()[1] == 5u) &&
                 (stats.holes()[0].vertices()[2] == 11u)));
        REQUIRE(stats.self_intersections().size() == 0);
        //        REQUIRE(stats.numBodies() == 1);
    }

    SECTION("Find Holes - Three Holes")
    {
        auto read_result = Cork::Files::readOFF("../../UnitTest/Test Files/Quadrilateral with 3 Holes.off");

        REQUIRE(read_result.succeeded());

        auto* mesh(read_result.return_ptr().release());

        REQUIRE(mesh->numVertices() == 26);
        REQUIRE(mesh->numTriangles() == 44);

        auto stats = mesh->ComputeTopologicalStatistics();

        REQUIRE(!stats.is_two_manifold());
        REQUIRE(stats.holes().size() == 3);
        REQUIRE(((stats.holes()[0].vertices()[0] == 21u) && (stats.holes()[0].vertices()[1] == 11u) &&
                 (stats.holes()[0].vertices()[2] == 5u) && (stats.holes()[0].vertices()[3] == 13u)));
        REQUIRE(((stats.holes()[1].vertices()[0] == 16u) && (stats.holes()[1].vertices()[1] == 24u) &&
                 (stats.holes()[1].vertices()[2] == 8u) ));
        REQUIRE(((stats.holes()[2].vertices()[0] == 16u) && (stats.holes()[2].vertices()[1] == 18u) &&
                 (stats.holes()[2].vertices()[2] == 12u) ));
        REQUIRE(stats.self_intersections().size() == 0);
        //        REQUIRE(stats.numBodies() == 1);
    }

    SECTION("Find Self Intersections")
    {
        auto read_result = Cork::Files::readOFF("../../UnitTest/Test Files/JuliaVaseWithSelfIntersection.off");

        REQUIRE(read_result.succeeded());

        auto* mesh(read_result.return_ptr().release());

        auto stats = mesh->ComputeTopologicalStatistics();

        std::set<Cork::Math::TriangleByIndicesIndex, std::greater<Cork::Math::TriangleByIndicesIndex>>
            triangles_to_remove;

        for (auto record : stats.self_intersections())
        {
            Cork::Math::EdgeByVertices edge_with_se(
                Cork::Math::TriangleByVertices(mesh->triangles()[record.edge_triangle_id_], mesh->vertices())
                    .edge(record.edge_index_));

//            std::cout << "Edge Index: " << record.edge_index_ << " Edge: " << edge_with_se << "    Triangle: "
//                      << Cork::Math::TriangleByVertices(mesh->triangles()[record.triangle_instersected_id_],
//                                                        mesh->vertices())
//                      << std::endl;

            for (auto triangle_id : record.triangles_sharing_edge_)
            {
                triangles_to_remove.insert(triangle_id);
            }
        }

        REQUIRE(stats.is_two_manifold());
        REQUIRE(stats.holes().size() == 0);
        REQUIRE(stats.self_intersections().size() == 2);
        //        REQUIRE(stats.numBodies() == 1);

        auto read_result2 = Cork::Files::readOFF("../../UnitTest/Test Files/JuliaVaseWithSelfIntersection.off");

        REQUIRE(read_result2.succeeded());

        auto original_mesh(read_result2.return_ptr().release());

        for (auto tri_to_remove_index : triangles_to_remove)
        {
            original_mesh->remove_triangle(tri_to_remove_index);
        }

        auto write_result = Cork::Files::writeOFF(
            "../../UnitTest/Test Results/JuliaVaseWithSelfIntersectionRemovedHoleLeft.off", *original_mesh);

        auto* mesh2(original_mesh);

        auto stats2 = mesh2->ComputeTopologicalStatistics();

        REQUIRE(!stats2.is_two_manifold());
        REQUIRE(stats2.holes().size() == 1);
        REQUIRE(stats2.self_intersections().size() == 0);
    }
}