#include <catch2/catch_all.hpp>

#include "catch2_matchers.h"

#include "file_formats/files.h"

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

        REQUIRE( mesh->numVertices() == 26 );
        REQUIRE( mesh->numTriangles() == 48 );
        
        auto write_result = Cork::Files::writeOFF("../../UnitTest/Test Results/Quadrilateral1.off", *mesh );

        REQUIRE( write_result.succeeded() );

        REQUIRE_THAT( "../../UnitTest/Test Files/Quadrilateral1.off", MatchesFile( "../../UnitTest/Test Results/Quadrilateral1.off" ) );
    }

    SECTION("Read Errors")
    {
        {
            auto read_result = Cork::Files::readOFF( "nosuchfile" );
            
            REQUIRE(( read_result.failed() && ( read_result.error_code() == Cork::Files::ReadFileResultCodes::UNABLE_TO_OPEN_FILE )));
        }

        {
            auto read_result = Cork::Files::readOFF( "../../UnitTest/Test Files/Quadrilateral1 Bad Vertex.off" );
            
            REQUIRE(( read_result.failed() && ( read_result.error_code() == Cork::Files::ReadFileResultCodes::OFF_ERROR_READING_VERTICES )));
        }

        {
            auto read_result = Cork::Files::readOFF( "../../UnitTest/Test Files/Quadrilateral1 Bad Triangle.off" );
            
            REQUIRE(( read_result.failed() && ( read_result.error_code() == Cork::Files::ReadFileResultCodes::OFF_ERROR_READING_FACES )));
        }

        {
            auto read_result = Cork::Files::readOFF( "../../UnitTest/Test Files/Quadrilateral1 Bad Counts.off" );
            
            REQUIRE(( read_result.failed() && ( read_result.error_code() == Cork::Files::ReadFileResultCodes::OFF_ERROR_READING_COUNTS )));
        }

        {
            auto read_result = Cork::Files::readOFF( "../../UnitTest/Test Files/Quadrilateral1 One Nontriangle Face.off" );
            
            REQUIRE(( read_result.failed() && ( read_result.error_code() == Cork::Files::ReadFileResultCodes::OFF_NON_TRIANGULAR_FACE )));
        }

        {
            auto read_result = Cork::Files::readOFF( "../../UnitTest/Test Files/Quadrilateral1 Duplicate Vertex.off" );
            
            REQUIRE(( read_result.failed() && ( read_result.error_code() == Cork::Files::ReadFileResultCodes::OFF_READ_DUPLICATE_VERTICES )));
        }
    }
}

