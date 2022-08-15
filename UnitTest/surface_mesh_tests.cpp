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

#include "result_codes.hpp"
#include "file_formats/files.hpp"
#include "mesh/triangle_mesh_wrapper.hpp"

//  The pragma below is to disable to false errors flagged by intellisense for Catch2 REQUIRE macros.

#if __INTELLISENSE__
#pragma diag_suppress 2486
#endif

//  NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers)

TEST_CASE("Surface Mesh Tests", "[surface]")
{
    SECTION("2 Manifold Single Body")
    {
        auto read_result = Cork::Files::readOFF("../../UnitTest/Test Files/ant.off");

        REQUIRE(read_result.succeeded());

        std::unique_ptr<Cork::Meshes::TriangleMeshWrapper> mesh_wrapper(
            dynamic_cast<Cork::Meshes::TriangleMeshWrapper*>(read_result.return_ptr().release()));

        Cork::TriangleMesh& mesh = *mesh_wrapper;

        REQUIRE(mesh.num_vertices() == 5001);
        REQUIRE(mesh.num_triangles() == 9998);

        {
            auto surface1 = mesh.extract_surface( Cork::Primitives::TriangleByIndicesIndex{5000UL}, 1);

            Cork::Files::writeOFF("../../UnitTest/Test Results/surface_mesh_tests/ant_extract_1.off", *surface1);

            REQUIRE(surface1->num_vertices() == 15);
            REQUIRE(surface1->num_triangles() == 16);
        }

        {
            auto surface2 = mesh.extract_surface(Cork::Primitives::TriangleByIndicesIndex{5000UL}, 2);

            Cork::Files::writeOFF("../../UnitTest/Test Results/surface_mesh_tests/ant_extract_2.off", *surface2);

            REQUIRE(surface2->num_vertices() == 37);
            REQUIRE(surface2->num_triangles() == 50);
        }

        {
            auto surface3 = mesh.extract_surface(Cork::Primitives::TriangleByIndicesIndex{5000UL}, 3);

            Cork::Files::writeOFF("../../UnitTest/Test Results/surface_mesh_tests/ant_extract_3.off", *surface3);

            REQUIRE(surface3->num_vertices() == 71);
            REQUIRE(surface3->num_triangles() == 108);
        }

        {
            auto surface4 = mesh.extract_surface(Cork::Primitives::TriangleByIndicesIndex{5000UL}, 4);

            Cork::Files::writeOFF("../../UnitTest/Test Results/surface_mesh_tests/ant_extract_4.off", *surface4);

            REQUIRE(surface4->num_vertices() == 120);
            REQUIRE(surface4->num_triangles() == 190);
        }

        {
            auto surface5 = mesh.extract_surface(Cork::Primitives::TriangleByIndicesIndex{5000UL}, 5);

            Cork::Files::writeOFF("../../UnitTest/Test Results/surface_mesh_tests/ant_extract_5.off", *surface5);

            REQUIRE(surface5->num_vertices() == 164);
            REQUIRE(surface5->num_triangles() == 287);
        }

        {
            auto surface6 = mesh.extract_surface(Cork::Primitives::TriangleByIndicesIndex{5000UL}, 6);

            Cork::Files::writeOFF("../../UnitTest/Test Results/surface_mesh_tests/ant_extract_6.off", *surface6);

            REQUIRE(surface6->num_vertices() == 207);
            REQUIRE(surface6->num_triangles() == 372);
        }

        {
            auto surface7 = mesh.extract_surface(Cork::Primitives::TriangleByIndicesIndex{5000UL}, 7);

            Cork::Files::writeOFF("../../UnitTest/Test Results/surface_mesh_tests/ant_extract_7.off", *surface7);

            REQUIRE(surface7->num_vertices() == 252);
            REQUIRE(surface7->num_triangles() == 461);
        }

        {
            auto surface8 = mesh.extract_surface(Cork::Primitives::TriangleByIndicesIndex{5000UL}, 8);

            Cork::Files::writeOFF("../../UnitTest/Test Results/surface_mesh_tests/ant_extract_8.off", *surface8);

            REQUIRE(surface8->num_vertices() == 300);
            REQUIRE(surface8->num_triangles() == 554);
        }

        {
            auto surface9 = mesh.extract_surface(Cork::Primitives::TriangleByIndicesIndex{5000UL}, 9);

            Cork::Files::writeOFF("../../UnitTest/Test Results/surface_mesh_tests/ant_extract_9.off", *surface9);

            REQUIRE(surface9->num_vertices() == 362);
            REQUIRE(surface9->num_triangles() == 663);
        }

        {
            auto surface10 = mesh.extract_surface(Cork::Primitives::TriangleByIndicesIndex{5000UL}, 10);

            Cork::Files::writeOFF("../../UnitTest/Test Results/surface_mesh_tests/ant_extract_10.off", *surface10);

            REQUIRE(surface10->num_vertices() == 442);
            REQUIRE(surface10->num_triangles() == 805);
        }
    }
}

//  NOLINTEND(cppcoreguidelines-avoid-magic-numbers)
