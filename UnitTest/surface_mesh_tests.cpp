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

#include "file_formats/files.hpp"
#include "mesh/triangle_mesh_wrapper.hpp"

//  The pragma below is to disable to false errors flagged by intellisense for Catch2 REQUIRE macros.

#if __INTELLISENSE__
#pragma diag_suppress 2486
#endif

TEST_CASE("Surface Mesh Tests", "[surface]")
{
    SECTION("2 Manifold Single Body")
    {
        auto read_result = Cork::Files::readOFF("../../UnitTest/Test Files/ant.off");

        REQUIRE(read_result.succeeded());

        std::unique_ptr<Cork::Meshes::TriangleMeshWrapper> mesh_wrapper(
            static_cast<Cork::Meshes::TriangleMeshWrapper*>(read_result.return_ptr().release()));

        Cork::TriangleMesh& mesh = *mesh_wrapper;

        REQUIRE(mesh.num_vertices() == 5001);
        REQUIRE(mesh.num_triangles() == 9998);

        {
            auto surface1 = mesh.extract_surface(5000u, 1, false);

            Cork::Files::writeOFF("../../UnitTest/Test Results/surface_mesh_tests/ant_extract_1.off", *surface1);

            REQUIRE(surface1->num_vertices() == 15);
            REQUIRE(surface1->num_triangles() == 16);
        }

        {
            auto surface1_smoothed = mesh.extract_surface(5000u, 1, true);

            Cork::Files::writeOFF("../../UnitTest/Test Results/surface_mesh_tests/ant_extract_1_smoothed.off",
                                  *surface1_smoothed);

            REQUIRE(surface1_smoothed->num_vertices() == 15);
            REQUIRE(surface1_smoothed->num_triangles() == 16);
        }

        {
            auto surface2 = mesh.extract_surface(5000u, 2, false);

            Cork::Files::writeOFF("../../UnitTest/Test Results/surface_mesh_tests/ant_extract_2.off", *surface2);

            REQUIRE(surface2->num_vertices() == 37);
            REQUIRE(surface2->num_triangles() == 50);
        }

        {
            auto surface2_smoothed = mesh.extract_surface(5000u, 2, true);

            Cork::Files::writeOFF("../../UnitTest/Test Results/surface_mesh_tests/ant_extract_2_smoothed.off",
                                  *surface2_smoothed);

            REQUIRE(surface2_smoothed->num_vertices() == 37);
            REQUIRE(surface2_smoothed->num_triangles() == 51);
        }

        {
            auto surface3 = mesh.extract_surface(5000u, 3, false);

            Cork::Files::writeOFF("../../UnitTest/Test Results/surface_mesh_tests/ant_extract_3.off", *surface3);

            REQUIRE(surface3->num_vertices() == 71);
            REQUIRE(surface3->num_triangles() == 108);
        }

        {
            auto surface3_smoothed = mesh.extract_surface(5000u, 3, true);

            Cork::Files::writeOFF("../../UnitTest/Test Results/surface_mesh_tests/ant_extract_3_smoothed.off",
                                  *surface3_smoothed);

            REQUIRE(surface3_smoothed->num_vertices() == 71);
            REQUIRE(surface3_smoothed->num_triangles() == 111);
        }

        {
            auto surface4 = mesh.extract_surface(5000u, 4, false);

            Cork::Files::writeOFF("../../UnitTest/Test Results/surface_mesh_tests/ant_extract_4.off", *surface4);

            REQUIRE(surface4->num_vertices() == 120);
            REQUIRE(surface4->num_triangles() == 190);
        }

        {
            auto surface4_smoothed = mesh.extract_surface(5000u, 4, true);

            Cork::Files::writeOFF("../../UnitTest/Test Results/surface_mesh_tests/ant_extract_4_smoothed.off",
                                  *surface4_smoothed);

            REQUIRE(surface4_smoothed->num_vertices() == 120);
            REQUIRE(surface4_smoothed->num_triangles() == 197);
        }

        {
            auto surface5 = mesh.extract_surface(5000u, 5, false);

            Cork::Files::writeOFF("../../UnitTest/Test Results/surface_mesh_tests/ant_extract_5.off", *surface5);

            REQUIRE(surface5->num_vertices() == 164);
            REQUIRE(surface5->num_triangles() == 287);
        }

        {
            auto surface5_smoothed = mesh.extract_surface(5000u, 5, true);

            Cork::Files::writeOFF("../../UnitTest/Test Results/surface_mesh_tests/ant_extract_5_smoothed.off",
                                  *surface5_smoothed);

            REQUIRE(surface5_smoothed->num_vertices() == 164);
            REQUIRE(surface5_smoothed->num_triangles() == 292);
        }

        {
            auto surface6 = mesh.extract_surface(5000u, 6, false);

            Cork::Files::writeOFF("../../UnitTest/Test Results/surface_mesh_tests/ant_extract_6.off", *surface6);

            REQUIRE(surface6->num_vertices() == 207);
            REQUIRE(surface6->num_triangles() == 372);
        }

        {
            auto surface6_smoothed = mesh.extract_surface(5000u, 6, true);

            Cork::Files::writeOFF("../../UnitTest/Test Results/surface_mesh_tests/ant_extract_6_smoothed.off",
                                  *surface6_smoothed);

            REQUIRE(surface6_smoothed->num_vertices() == 207);
            REQUIRE(surface6_smoothed->num_triangles() == 375);
        }
    }
}
