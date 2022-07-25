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

#include <numeric>

#include <catch2/catch_all.hpp>
#include <catch2/catch_approx.hpp>

#include "primitives/boundary_edge.hpp"
#include "math/basic_stats.hpp"

//  The pragma below is to disable to false errors flagged by intellisense for Catch2 REQUIRE macros.

#if __INTELLISENSE__
#pragma diag_suppress 2486
#endif

using Vertex3D = Cork::Primitives::Vertex3D;

TEST_CASE("Boundary Edge Tests", "[3D Basic]")
{
    const std::vector<Cork::Primitives::Vertex3D> NON_SELF_INTERSECTING_POLYGON_1 = {
        {-2.298, 0.687, -12.612},  {-1.899, 1.128, -12.6165},   {-1.7865, 1.2315, -12.606}, {-1.788, 1.23, -12.1275},
        {-1.791, 1.2315, -12.0495}, {-1.7145, 1.3125, -12.054}, {-1.6425, 1.3965, -12.054}, {-1.5855, 1.485, -12.057},
        {-1.5345, 1.5945, -12.06},  {-1.53, 1.572, -12.4425},    {-1.524, 1.599, -12.6225}, {-1.398, 1.656, -12.6225},
        {-1.0665, 1.989, -12.624},  {-0.7335, -1.3635, -12.5835}};

    SECTION("Best Fit Plane")
    {
        Cork::Primitives::Vertex3DVector vertices( NON_SELF_INTERSECTING_POLYGON_1 );

        std::vector<Cork::Primitives::VertexIndex>      indices_raw(vertices.size(), Cork::Primitives::VertexIndex{0});
        std::iota( indices_raw.begin(), indices_raw.end(), Cork::Primitives::VertexIndex{0} );

        Cork::Primitives::VertexIndexVector     indices( std::move( indices_raw ) );

        Cork::Primitives::BoundaryEdge boundry(std::move(vertices), std::move( indices ));

        Cork::Math::BestFitPlaneEquation   best_fit_plane = boundry.best_fit_plane();

        auto deviations = boundry.get_point_deviations(best_fit_plane);

        REQUIRE(deviations.size() == 14);

        Cork::Math::Statistics      deviation_stats( deviations.begin(), deviations.end() );

        REQUIRE( deviation_stats.count() == 14 );
    }
}