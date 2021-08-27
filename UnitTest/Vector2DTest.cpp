
#include <catch2/catch_all.hpp>

#include "math/Vector2DTemplate.h"



//  The pragma below is to disable to false errors flagged by intellisense for Catch2 REQUIRE macros.

#if __INTELLISENSE__
#pragma diag_suppress 2486
#endif

typedef Cork::Math::Vector2DTemplate<double>     Vector2D;


TEST_CASE("Vector2D Tests", "[cork-math]" )
{
    SECTION( "Element Access" )
    {
        Vector2D        test_vec( 7, 4 );

        REQUIRE(( test_vec.x() == 7 && test_vec.y() == 4 ));
        REQUIRE(( test_vec[0] == 7 && test_vec[1] == 4 ));
    }

    SECTION( "Addition" )
    {
        Vector2D     test_vec1( 1, 2 );
        Vector2D     test_vec2( 4, 5 );

        Vector2D    result = test_vec1 + test_vec2;

        REQUIRE((( result.x() == 5 ) && ( result.y() == 7 )));
    }

    SECTION( "Subtraction" )
    {
        Vector2D     test_vec1( 1, 8.5 );
        Vector2D     test_vec2( 4, 3 );

        Vector2D    result = test_vec1 - test_vec2;

        REQUIRE((( result.x() == (double)-3 ) && ( result.y() == (double)5.5 )));
    }
    
    SECTION( "Scalar Multiplication" )
    {
        Vector2D     test_vec1( -11, 13 );

        Vector2D    result = test_vec1 * 3.5;

        REQUIRE((( result.x() == (-11 * 3.5) ) && ( result.y() == (13 * 3.5) )));

        result = -5.0 * test_vec1;

        REQUIRE(( result == Vector2D( 55, -65 ) ));
    }
    
    SECTION( "Scalar Division" )
    {
        Vector2D     test_vec1( 23, 32 );

        Vector2D    result = test_vec1 / 5.6;

        REQUIRE((( result.x() == ( (double)23 / (double)5.6 )) && ( result.y() == ( (double)32 / (double)5.6 ))));
    }
        
    SECTION( "Negation and ABS" )
    {
        Vector2D    test_vec1( -23, -32 );

        Vector2D    result = test_vec1.abs();

        REQUIRE((( result.x() == 23 ) && ( result.y() == 32 )));

        result = -test_vec1;
        
        REQUIRE((( result.x() == 23 ) && ( result.y() == 32 )));
    }

    SECTION( "Destructive Addition" )
    {
        Vector2D     test_vec1( 1, 2 );
        Vector2D     test_vec2( 4, 5 );

        test_vec1 += test_vec2;

        REQUIRE((( test_vec1.x() == 5 ) && ( test_vec1.y() == 7 )));
    }

    SECTION( "Destructive Subtraction" )
    {
        Vector2D     test_vec1( 1, 2 );
        Vector2D     test_vec2( 4, 6 );

        test_vec1 -= test_vec2;

        REQUIRE((( test_vec1.x() == -3 ) && ( test_vec1.y() == -4 )));
    }
    
    SECTION( "Destructive Scalar Multiplication" )
    {
        Vector2D     test_vec1( 11, 13 );

        test_vec1 *= 3.5;

        REQUIRE((( test_vec1.x() == 11 * 3.5 ) && ( test_vec1.y() == 13 * 3.5 )));
    }
    
    SECTION( "Destructive Scalar Division" )
    {
        Vector2D     test_vec1( 23, 32 );

        test_vec1 /= 5.6;

        REQUIRE((( test_vec1.x() == 23 / 5.6 ) && ( test_vec1.y() == 32 / 5.6 )));
    }

    SECTION( "Equality and Inequality" )
    {
        Vector2D     test_vec1( 1, 2 );
        Vector2D     test_vec2( 1, 2 );

        REQUIRE( test_vec1 == test_vec2 );
        REQUIRE( !( test_vec1 != test_vec2 ) );

        test_vec2 = Vector2D( 10, 2 );

        REQUIRE( !(test_vec1 == test_vec2 ));
        REQUIRE( test_vec1 != test_vec2 );

        test_vec2 = Vector2D( 1, 20 );

        REQUIRE( !(test_vec1 == test_vec2 ));
        REQUIRE( test_vec1 != test_vec2 );
    }

    SECTION( "Max" )
    {
        Vector2D     test_vec1( 1, 20 );
        Vector2D     test_vec2( 1, 2 );
        Vector2D     test_vec3( 100, 2 );

        REQUIRE( test_vec1.max() == 20 );
        REQUIRE( test_vec1.min() == 1 );

        REQUIRE( test_vec1.max( test_vec2 ) == Vector2D( 1, 20 ) );

        test_vec1 = Vector2D( 30, 2 );

        REQUIRE( test_vec1.max( test_vec2 ) == Vector2D( 30, 2 ) );

        test_vec2 = Vector2D( 10, 5 );

        REQUIRE(( test_vec1.max( test_vec2 ) == Vector2D( 30, 5 ) ));
    }

    SECTION( "Min" )
    {
        Vector2D     test_vec1( -1, 20 );
        Vector2D     test_vec2( 1, 2 );
        Vector2D     test_vec3( 100, 2 );

        REQUIRE( test_vec1.min( test_vec2 ) == Vector2D( -1, 2 ) );

        test_vec1 = Vector2D( 1, -2 );

        REQUIRE( test_vec1.min( test_vec2 ) == Vector2D( 1, -2 ) );

        test_vec2 = Vector2D( 2, -3 );

        REQUIRE( test_vec1.min( test_vec2 ) == Vector2D( 1, -3 ) );
    }


    SECTION( "Max and Min Dimension" )
    {
        REQUIRE( Vector2D( 1, 2 ).maxDim() == 1 );
        REQUIRE( Vector2D( 1, 3 ).maxDim() == 1 );
        REQUIRE( Vector2D( 3, 2 ).maxDim() == 0 );

        REQUIRE( Vector2D( 1, 2 ).minDim() == 0 );
        REQUIRE( Vector2D( 3, 1 ).minDim() == 1 );
        REQUIRE( Vector2D( 3, 2 ).minDim() == 1 );
    }

    SECTION( "Length" )
    {
        Vector2D     test_vec( -1, 20 );

        REQUIRE( test_vec.len() == sqrt( 401 ) );
        REQUIRE( test_vec.len_squared() == 401 );
    }


    SECTION( "Normalization" )
    {
        Vector2D    test_vec( 4, 5 );
        NUMERIC_PRECISION   length = test_vec.len();

        test_vec.normalize();

        REQUIRE(( test_vec.x() == ( 4 / length) && test_vec.y() == ( 5 / length ) ));

        test_vec = Vector2D( 7, 8 ).normalized();
        length = Vector2D( 7, 8 ).len();

        REQUIRE(( test_vec.x() == 7 / length && test_vec.y() == 8 /length ));
    }

    SECTION( "Dot" )
    {
        Vector2D     test_vec1( 2, 3 );
        Vector2D     test_vec2( 5, 6 );

        REQUIRE( test_vec1.dot( test_vec2 ) == 10 + 18 );
    }

    SECTION( "Determinant" )
    {
        Vector2D        test_vec1( 3, -2 );
        Vector2D        test_vec2( 7, 4 );

        REQUIRE( determinant( test_vec1, test_vec2 ) == 26 );
    }
}

