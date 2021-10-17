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
        
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 31ul, 50ul ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 2ul, 3ul ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 1ul, 2ul ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 0ul, 1ul ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 3ul, 4ul ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 11ul, 12ul ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 4ul, 5ul ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 5ul, 6ul ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 21ul, 22ul ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 7ul, 0ul ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 1ul, 10ul ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 30ul, 31ul ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 10ul, 11ul ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 12ul, 13ul ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 40ul, 50ul ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 13ul, 1ul ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 4ul, 20ul ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 20ul, 21ul ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 22ul, 4ul ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 6ul, 7ul ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 21ul, 30ul ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 31ul, 21ul ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 22ul, 40ul ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 40ul, 41ul ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 31ul, 40ul ) );
        hole_edges.emplace_back( Cork::Math::EdgeByIndices( 41ul, 22ul ) );

        std::vector<Cork::Meshes::Hole> holes = Cork::Meshes::HoleBuilder::extract_holes( hole_edges );

        REQUIRE( holes.size() == 6 );
        REQUIRE((( holes[0].vertices()[0] == 13ul ) && ( holes[0].vertices()[1] == 12ul ) && ( holes[0].vertices()[2] == 11ul ) && ( holes[0].vertices()[3] == 10ul ) && ( holes[0].vertices()[4] == 1ul )));
        REQUIRE((( holes[1].vertices()[0] == 31ul ) && ( holes[1].vertices()[1] == 21ul ) && ( holes[1].vertices()[2] == 30ul )));
        REQUIRE((( holes[2].vertices()[0] == 40ul ) && ( holes[2].vertices()[1] == 31ul ) && ( holes[2].vertices()[2] == 50ul )));
        REQUIRE((( holes[3].vertices()[0] == 4ul ) && ( holes[3].vertices()[1] == 5ul ) && ( holes[3].vertices()[2] == 6ul ) && ( holes[3].vertices()[3] == 7ul ) && ( holes[3].vertices()[4] == 0ul ) && ( holes[3].vertices()[5] == 1ul ) && ( holes[3].vertices()[6] == 2ul ) && ( holes[3].vertices()[7] == 3ul )));
        REQUIRE((( holes[4].vertices()[0] == 22ul ) && ( holes[4].vertices()[1] == 4ul ) && ( holes[4].vertices()[2] == 20ul ) && ( holes[4].vertices()[3] == 21ul )));
        REQUIRE((( holes[5].vertices()[0] == 40ul ) && ( holes[5].vertices()[1] == 22ul ) && ( holes[5].vertices()[2] == 41ul )));
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
        REQUIRE(((stats.holes()[0].vertices()[0] == 21ul) && (stats.holes()[0].vertices()[1] == 5ul) &&
                 (stats.holes()[0].vertices()[2] == 11ul)));
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
        REQUIRE(((stats.holes()[0].vertices()[0] == 21ul) && (stats.holes()[0].vertices()[1] == 11ul) &&
                 (stats.holes()[0].vertices()[2] == 5ul) && (stats.holes()[0].vertices()[3] == 13ul)));
        REQUIRE(((stats.holes()[1].vertices()[0] == 16ul) && (stats.holes()[1].vertices()[1] == 24ul) &&
                 (stats.holes()[1].vertices()[2] == 8ul) ));
        REQUIRE(((stats.holes()[2].vertices()[0] == 16ul) && (stats.holes()[2].vertices()[1] == 18ul) &&
                 (stats.holes()[2].vertices()[2] == 12ul) ));
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