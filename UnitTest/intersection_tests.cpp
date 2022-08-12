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

#include "intersection/intersection_tests_3d.hpp"

//  The pragma below is to disable to false errors flagged by intellisense for Catch2 REQUIRE macros.

#if __INTELLISENSE__
#pragma diag_suppress 2486
#endif

using VertexIndex = Cork::Primitives::VertexIndex;
using EdgeByIndices = Cork::Primitives::EdgeByIndices;

TEST_CASE("Intersection Tests", "[intersection]")
{
    SECTION("Triangle Edge Intersection Positive")
    {
        Cork::Empty3d::ExactArithmeticContext arith_context;

        Cork::Empty3d::IntersectingTriangle tri(Cork::Primitives::Vector3D(2, 2, 1),
                                                 Cork::Primitives::Vector3D(7, 4, 2),
                                                 Cork::Primitives::Vector3D(4, 9, 3));

        Cork::Empty3d::IntersectingEdge     edge(Cork::Primitives::Vector3D(2, 2, 3),
                                                 Cork::Primitives::Vector3D(6, 7, 0));

        Cork::Empty3d::TriangleEdgeIntersection tri_edge(std::move(tri), std::move(edge));

        //  First for the regular, perhaps uncertain computations

        REQUIRE(tri_edge.hasIntersectionFilter() == Cork::Empty3d::HasIntersection::YES);

        Cork::Primitives::Vector3D intersection = tri_edge.coords();

        REQUIRE(((intersection.x() == Catch::Approx(538.0 / 145.0).epsilon(0.00001)) &&
                 (intersection.y() == Catch::Approx(120.0 / 29.0).epsilon(0.00001)) &&
                 (intersection.z() == Catch::Approx(249.0 / 145.0).epsilon(0.00001))));

        //  Second the exact computations

        auto       quantizer_result = Cork::Math::Quantizer::get_quantizer( 100, 0.01 );   //  Need some reasonable values for max mag and min length

        REQUIRE( quantizer_result.succeeded() );

        REQUIRE(tri_edge.hasIntersectionExact(quantizer_result.return_value(), arith_context) == Cork::Empty3d::HasIntersection::YES);
        REQUIRE(tri_edge.hasIntersection(arith_context) == Cork::Empty3d::HasIntersection::YES);

        Cork::Primitives::Vector3D intersection_exact = tri_edge.coordsExact( quantizer_result.return_value() );

        REQUIRE(((intersection.x() == Catch::Approx(538.0 / 145.0).epsilon(0.00001)) &&
                 (intersection.y() == Catch::Approx(120.0 / 29.0).epsilon(0.00001)) &&
                 (intersection.z() == Catch::Approx(249.0 / 145.0).epsilon(0.00001))));
    }

    SECTION("Triangle Edge Intersection Negative")
    {
        Cork::Empty3d::ExactArithmeticContext arith_context;

        Cork::Empty3d::IntersectingTriangle tri(Cork::Primitives::Vector3D(2, 2, 1),
                                                 Cork::Primitives::Vector3D(7, 4, 2),
                                                 Cork::Primitives::Vector3D(4, 9, 3));

        Cork::Empty3d::IntersectingEdge     edge(Cork::Primitives::Vector3D(2, 2, 3),           //  This edge will not intersect
                                                 Cork::Primitives::Vector3D(-6, -7, 0));

        Cork::Empty3d::TriangleEdgeIntersection tri_edge(std::move(tri), std::move(edge));

        //  First for the regular, perhaps uncertain computations

        REQUIRE(tri_edge.hasIntersectionFilter() == Cork::Empty3d::HasIntersection::NO);

        //  Second the exact computations

        auto       quantizer_result = Cork::Math::Quantizer::get_quantizer( 100, 0.01 );   //  Need some reasonable values for max mag and min length

        REQUIRE( quantizer_result.succeeded() );

        REQUIRE(tri_edge.hasIntersectionExact(quantizer_result.return_value(), arith_context) == Cork::Empty3d::HasIntersection::NO);
        REQUIRE(tri_edge.hasIntersection(arith_context) == Cork::Empty3d::HasIntersection::NO);
    }

    SECTION("Triangle Triangle Triangle Intersection Positive")
    {
        Cork::Empty3d::ExactArithmeticContext arith_context;

        Cork::Empty3d::IntersectingTriangle tri1(Cork::Primitives::Vector3D(1, 3, 1),
                                                 Cork::Primitives::Vector3D(4, 7, 4),
                                                 Cork::Primitives::Vector3D(7, 9, 3));

        Cork::Empty3d::IntersectingTriangle tri2(Cork::Primitives::Vector3D(2, 2, 3),
                                                 Cork::Primitives::Vector3D(6, 7, 3),
                                                 Cork::Primitives::Vector3D(4, 8, 4));

        Cork::Empty3d::IntersectingTriangle tri3(Cork::Primitives::Vector3D(2, 2, 4),
                                                 Cork::Primitives::Vector3D(6, 7, 1),
                                                 Cork::Primitives::Vector3D(4, 8, 5));

        Cork::Empty3d::TriangleTriangleTriangleIntersection tri_tri_tri(std::move(tri1), std::move(tri2),
                                                                        std::move(tri3));

        //  First for the regular, perhaps uncertain computations

        REQUIRE(tri_tri_tri.hasIntersectionFilter() == Cork::Empty3d::HasIntersection::YES);

        Cork::Primitives::Vector3D intersection = tri_tri_tri.coords();

        REQUIRE(((intersection.x() == Catch::Approx(104.0 / 23.0).epsilon(0.00001)) &&
                 (intersection.y() == Catch::Approx(499.0 / 69.0).epsilon(0.00001)) &&
                 (intersection.z() == Catch::Approx(248.0 / 69.0).epsilon(0.00001))));

        //  Second the exact computations

        auto       quantizer_result = Cork::Math::Quantizer::get_quantizer( 100, 0.01 );   //  Need some reasonable values for max mag and min length

        REQUIRE( quantizer_result.succeeded() );

        REQUIRE(tri_tri_tri.hasIntersectionExact(quantizer_result.return_value(), arith_context) == Cork::Empty3d::HasIntersection::YES);
        REQUIRE(tri_tri_tri.hasIntersection(arith_context) == Cork::Empty3d::HasIntersection::YES);

        Cork::Primitives::Vector3D intersection_exact = tri_tri_tri.coordsExact( quantizer_result.return_value() );

        REQUIRE(((intersection_exact.x() == Catch::Approx(104.0 / 23.0).epsilon(0.00001)) &&
                 (intersection_exact.y() == Catch::Approx(499.0 / 69.0).epsilon(0.00001)) &&
                 (intersection_exact.z() == Catch::Approx(248.0 / 69.0).epsilon(0.00001))));
    }

        SECTION("Triangle Triangle Triangle Intersection Negative")
    {
        Cork::Empty3d::ExactArithmeticContext arith_context;

        Cork::Empty3d::IntersectingTriangle tri1(Cork::Primitives::Vector3D(1, 3, 1),
                                                 Cork::Primitives::Vector3D(4, 7, 4),
                                                 Cork::Primitives::Vector3D(7, 9, 3));

        Cork::Empty3d::IntersectingTriangle tri2(Cork::Primitives::Vector3D(2, 2, 3),
                                                 Cork::Primitives::Vector3D(6, 7, 3),
                                                 Cork::Primitives::Vector3D(4, 8, 4));

        Cork::Empty3d::IntersectingTriangle tri3(Cork::Primitives::Vector3D(-2, -2, 4),        //  This tri will not intersect the others
                                                 Cork::Primitives::Vector3D(-6, -7, 1),
                                                 Cork::Primitives::Vector3D(4, 8, 5));

        Cork::Empty3d::TriangleTriangleTriangleIntersection tri_tri_tri(std::move(tri1), std::move(tri2),
                                                                        std::move(tri3));

        //  First for the regular, perhaps uncertain computations

        REQUIRE(tri_tri_tri.hasIntersectionFilter() == Cork::Empty3d::HasIntersection::NO);

        //  Second the exact computations

        auto       quantizer_result = Cork::Math::Quantizer::get_quantizer( 100, 0.01 );   //  Need some reasonable values for max mag and min length

        REQUIRE( quantizer_result.succeeded() );

        REQUIRE(tri_tri_tri.hasIntersectionExact(quantizer_result.return_value(), arith_context) == Cork::Empty3d::HasIntersection::NO);
        REQUIRE(tri_tri_tri.hasIntersection(arith_context) == Cork::Empty3d::HasIntersection::NO);
    }
}
