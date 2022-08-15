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

#include "math/plane.hpp"

//  The pragma below is to disable to false errors flagged by intellisense for Catch2 REQUIRE macros.

#if __INTELLISENSE__
#pragma diag_suppress 2486
#endif

//  NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers)

TEST_CASE("3D Utilities Tests", "[geometry]")
{
    const std::vector<Cork::Primitives::Vertex3D> SURFACE_1 = {
        {1.782, 1.7115, -14.2455},   {1.8315, 1.7895, -14.1585},  {1.779, 1.962, -14.0775},
        {1.6905, 1.977, -14.151},    {1.638, 1.98, -14.2455},     {1.6395, 2.0115, -14.2455},
        {0.57, 2.9715, -14.2455},    {0.4605, 2.9325, -14.2455},  {0.228, 2.823, -14.2455},
        {0.141, 2.7585, -14.2455},   {0.015, 2.6535, -14.2455},   {-0.1005, 2.5335, -14.2455},
        {-0.147, 2.4735, -14.2455},  {-0.156, 2.4615, -14.2455},  {-0.2475, 2.37, -14.2455},
        {-0.4395, 2.1225, -14.2455}, {-0.45, 2.1135, -14.2455},   {-0.5955, 1.9395, -14.2455},
        {-1.281, 1.389, -14.2455},   {-1.3095, 1.389, -14.223},   {-1.308, 1.3545, -14.2455},
        {-1.389, 1.2405, -14.2455},  {-1.4235, 1.1985, -14.2455}, {-1.4625, 1.149, -14.2455},
        {1.683, 1.7775, -14.2455}};

    const std::vector<Cork::Primitives::Vertex3D> SURFACE_2 = {
        {1.971, -2.3655, -14.0625}, {2.004, -2.064, -14.1735},   {1.977, -1.8885, -14.2455}, {2.01, -1.887, -14.2455},
        {2.157, -1.9635, -14.205},  {2.1375, -1.8705, -14.2455}, {2.232, -1.845, -14.2455},  {2.562, -1.3935, -14.2455},
        {2.5605, -1.308, -14.2455}, {2.5545, -1.2495, -14.2455}, {1.6845, -1.854, -14.2455}, {1.5825, -1.9335, -14.202},
        {1.602, -2.1285, -14.1225}, {1.71, -2.382, -14.031},     {1.8165, -2.382, -14.0445}};

    SECTION("Surface 1")
    {
        Cork::Math::BestFitPlaneEquation best_fit_plane =
            Cork::Math::BestFitPlaneEquation(SURFACE_1.size(), SURFACE_1.begin(), SURFACE_1.end());

        REQUIRE(best_fit_plane.centroid().x() == Catch::Approx(0.12594).epsilon(10e-4));
        REQUIRE(best_fit_plane.centroid().y() == Catch::Approx(2.0433).epsilon(10e-4));
        REQUIRE(best_fit_plane.centroid().z() == Catch::Approx(-14.2306).epsilon(10e-4));

        REQUIRE(best_fit_plane.unit_normal().x() == Catch::Approx(-0.0198111).epsilon(10e-4));
        REQUIRE(best_fit_plane.unit_normal().y() == Catch::Approx(0.0212086).epsilon(10e-4));
        REQUIRE(best_fit_plane.unit_normal().z() == Catch::Approx(0.999579).epsilon(10e-4));

        REQUIRE( best_fit_plane.unit_normal().len() == Catch::Approx(1).epsilon(10e-8) );

        REQUIRE( best_fit_plane.rms_error() == Catch::Approx(0.166599).epsilon(10e-4) );
    }

    SECTION("Surface 2")
    {
        Cork::Math::BestFitPlaneEquation best_fit_plane =
            Cork::Math::BestFitPlaneEquation(SURFACE_2.size(), SURFACE_2.begin(), SURFACE_2.end());

        REQUIRE(best_fit_plane.centroid().x() == Catch::Approx(2.0374).epsilon(10e-4));
        REQUIRE(best_fit_plane.centroid().y() == Catch::Approx(-1.901).epsilon(10e-4));
        REQUIRE(best_fit_plane.centroid().z() == Catch::Approx(-14.187).epsilon(10e-4));

        REQUIRE(best_fit_plane.unit_normal().x() == Catch::Approx(-0.0792589).epsilon(10e-4));
        REQUIRE(best_fit_plane.unit_normal().y() == Catch::Approx(0.238026).epsilon(10e-4));
        REQUIRE(best_fit_plane.unit_normal().z() == Catch::Approx(0.968019).epsilon(10e-4));

        REQUIRE( best_fit_plane.unit_normal().len() == Catch::Approx(1).epsilon(10e-8) );

        REQUIRE( best_fit_plane.rms_error() == Catch::Approx(0.166785).epsilon(10e-4) );
    }
}

//  NOLINTEND(cppcoreguidelines-avoid-magic-numbers)
