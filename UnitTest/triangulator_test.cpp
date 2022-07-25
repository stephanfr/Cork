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

#include "../src/constants.hpp"

#include "intersection/triangulator.hpp"

#include <catch2/catch_all.hpp>
#include <iostream>
#include <list>

#include "result_codes.hpp"

//  The pragma below is to disable to false errors flagged by intellisense for Catch2 REQUIRE macros.

#if __INTELLISENSE__
#pragma diag_suppress 2486
#endif

TEST_CASE("Triangulator Tests", "[cork basic]")
{
    SECTION("Simple Test")
    {
        Cork::Triangulator::Triangulator triangulator;

        triangulator.add_point(Cork::Triangulator::Point(0.0, 0.0, false));
        triangulator.add_point(Cork::Triangulator::Point(10.0, 0.0, false));
        triangulator.add_point(Cork::Triangulator::Point(10.0, 10.0, false));
        triangulator.add_point(Cork::Triangulator::Point(0.0, 10.0, false));

        triangulator.add_segment(Cork::Triangulator::Segment(0, 1, false));
        triangulator.add_segment(Cork::Triangulator::Segment(1, 2, false));
        triangulator.add_segment(Cork::Triangulator::Segment(2, 3, false));
        triangulator.add_segment(Cork::Triangulator::Segment(3, 0, false));

        auto result = triangulator.compute_triangulation();

        REQUIRE(result.succeeded());

        auto triangles(result.return_ptr().release());

        REQUIRE(triangles->size() == 2);
        REQUIRE((*triangles)[0] == Cork::Triangulator::Triangle(3, 0, 1));
        REQUIRE((*triangles)[1] == Cork::Triangulator::Triangle(1, 2, 3));
    }

    SECTION("10 Sided Polygon")
    {
        Cork::Triangulator::Triangulator triangulator;

        triangulator.add_point(Cork::Triangulator::Point(3.1, -1.0, true));
        triangulator.add_point(Cork::Triangulator::Point(6.5, -1.0, true));
        triangulator.add_point(Cork::Triangulator::Point(9.2, 1, true));
        triangulator.add_point(Cork::Triangulator::Point(10.2, 4.2, true));
        triangulator.add_point(9.2, 7.3, true);
        triangulator.add_point(6.5, 9.3, true);
        triangulator.add_point(3.1, 9.3, true);
        triangulator.add_point(Cork::Triangulator::Point(0.4, 7.3, true));
        triangulator.add_point(Cork::Triangulator::Point(-0.6, 4.2, true));
        triangulator.add_point(0.4, 1.0, true);

        triangulator.add_segment(0, 1, true);
        triangulator.add_segment(Cork::Triangulator::Segment(1, 2, true));
        triangulator.add_segment(2, 3, true);
        triangulator.add_segment(Cork::Triangulator::Segment(3, 4, true));
        triangulator.add_segment(4, 5, true);
        triangulator.add_segment(Cork::Triangulator::Segment(5, 6, true));
        triangulator.add_segment(6, 7, true);
        triangulator.add_segment(Cork::Triangulator::Segment(7, 8, true));
        triangulator.add_segment(Cork::Triangulator::Segment(8, 9, true));
        triangulator.add_segment(Cork::Triangulator::Segment(9, 0, true));

        auto result = triangulator.compute_triangulation();

        REQUIRE(result.succeeded());

        auto triangles(result.return_ptr().release());

        REQUIRE(triangles->size() == 8);

        REQUIRE((*triangles)[0] == Cork::Triangulator::Triangle(8, 9, 2));
        REQUIRE((*triangles)[1] == Cork::Triangulator::Triangle(7, 4, 6));
        REQUIRE((*triangles)[2] == Cork::Triangulator::Triangle(8, 3, 7));
        REQUIRE((*triangles)[3] == Cork::Triangulator::Triangle(2, 9, 0));
        REQUIRE((*triangles)[4] == Cork::Triangulator::Triangle(2, 3, 8));
        REQUIRE((*triangles)[5] == Cork::Triangulator::Triangle(6, 4, 5));
        REQUIRE((*triangles)[6] == Cork::Triangulator::Triangle(7, 3, 4));
        REQUIRE((*triangles)[7] == Cork::Triangulator::Triangle(0, 1, 2));
    }

    
    SECTION("Too Many Points")
    {
        Cork::Triangulator::Triangulator triangulator;

        auto result = triangulator.will_problem_fit( MAX_TRIANGULATION_POINTS + 1, 1 );

        REQUIRE( result == Cork::TriangulationResultCodes::TOO_MANY_POINTS );
    }

        
    SECTION("Too Many Segments")
    {
        Cork::Triangulator::Triangulator triangulator;

        auto result = triangulator.will_problem_fit( 1, MAX_TRIANGULATION_POINTS + 1 );

        REQUIRE( result == Cork::TriangulationResultCodes::TOO_MANY_SEGMENTS );
    }
}