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

#include "catch2_matchers.h"

#include "result_codes.hpp"
#include "file_formats/files.hpp"

//  The pragma below is to disable to false errors flagged by intellisense for Catch2 REQUIRE macros.

#if __INTELLISENSE__
#pragma diag_suppress 2486
#endif

TEST_CASE("OFF File Tests", "[file io]")
{
    SECTION("Read, Write, Compare")
    {
        auto read_result = Cork::Files::readOFF( "../../UnitTest/Test Files/Quadrilateral1.off" );
        
        REQUIRE( read_result.succeeded() );

        auto* mesh( read_result.return_ptr().release() );

        REQUIRE( mesh->num_vertices() == 26 );
        REQUIRE( mesh->num_triangles() == 48 );
        
        auto write_result = Cork::Files::writeOFF("../../UnitTest/Test Results/Quadrilateral1.off", *mesh );

        REQUIRE( write_result.succeeded() );

        REQUIRE_THAT( "../../UnitTest/Test Files/Quadrilateral1.off", MatchesFile( "../../UnitTest/Test Results/Quadrilateral1.off" ) );
    }

    SECTION("Read Errors")
    {
        {
            auto read_result = Cork::Files::readOFF( "nosuchfile" );
            
            REQUIRE(( read_result.failed() && ( read_result.error_code() == Cork::ReadFileResultCodes::UNABLE_TO_OPEN_FILE )));
        }

        {
            auto read_result = Cork::Files::readOFF( "../../UnitTest/Test Files/Quadrilateral1 Bad Vertex.off" );
            
            REQUIRE(( read_result.failed() && ( read_result.error_code() == Cork::ReadFileResultCodes::OFF_ERROR_READING_VERTICES )));
        }

        {
            auto read_result = Cork::Files::readOFF( "../../UnitTest/Test Files/Quadrilateral1 Bad Triangle.off" );
            
            REQUIRE(( read_result.failed() && ( read_result.error_code() == Cork::ReadFileResultCodes::OFF_ERROR_READING_FACES )));
        }

        {
            auto read_result = Cork::Files::readOFF( "../../UnitTest/Test Files/Quadrilateral1 Bad Counts.off" );
            
            REQUIRE(( read_result.failed() && ( read_result.error_code() == Cork::ReadFileResultCodes::OFF_ERROR_READING_COUNTS )));
        }

        {
            auto read_result = Cork::Files::readOFF( "../../UnitTest/Test Files/Quadrilateral1 One Nontriangle Face.off" );
            
            REQUIRE(( read_result.failed() && ( read_result.error_code() == Cork::ReadFileResultCodes::OFF_NON_TRIANGULAR_FACE )));
        }

        {
            auto read_result = Cork::Files::readOFF( "../../UnitTest/Test Files/Quadrilateral1 Duplicate Vertex.off" );
            
            REQUIRE(( read_result.failed() && ( read_result.error_code() == Cork::ReadFileResultCodes::OFF_READ_DUPLICATE_VERTICES )));
        }

        {
            auto read_result = Cork::Files::readOFF( "../../UnitTest/Test Files/Quadrilateral1 Comment.off" );
            
            REQUIRE( read_result.succeeded() );
        }
    }
}

