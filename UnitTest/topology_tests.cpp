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
        REQUIRE( stats.IsTwoManifold() );
        REQUIRE( stats.numEdges() == 72 );
    }
    
    SECTION("Find Holes")
    {
        auto read_result = Cork::Files::readOFF("../../UnitTest/Test Files/Quadrilateral with Hole.off");

        REQUIRE(read_result.succeeded());

        auto* mesh(read_result.return_ptr().release());

        REQUIRE(mesh->numVertices() == 26);
        REQUIRE(mesh->numTriangles() == 47);

        auto stats = mesh->ComputeTopologicalStatistics();
//        REQUIRE(stats.numBodies() == 1);
    }
}