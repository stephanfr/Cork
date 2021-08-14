
#include <catch2/catch_all.hpp>

#include "../CorkLib/SIMDInstructionSet.h"

#define __AVX_AVAILABLE__

#include "../CorkLib/Math/Vector3DTemplate.h"

#include <string>


//  The pragma below is to disable to false errors flagged by intellisense for Catch2 REQUIRE macros.

#if __INTELLISENSE__
#pragma diag_suppress 2486
#endif

typedef Cork::Math::Vector2DTemplate<double>     Vector2D;

typedef Cork::Math::Vector3DTemplate<double,SIMDInstructionSet::NONE>   Vector3D;
typedef Cork::Math::Vector3DTemplate<double,SIMDInstructionSet::AVX>    Vector3DAVX;

TEMPLATE_TEST_CASE("Vector3D Tests", "[cork-math]", Vector3D, Vector3DAVX )
{
    typedef TestType     Vector3D;
    
    SECTION( "Element Access" )
    {
        Vector3D        test_vec( 7, 4, 3 );

        REQUIRE(( test_vec.x() == 7 && test_vec.y() == 4 && test_vec.z() == 3 ));
        REQUIRE(( test_vec[0] == 7 && test_vec[1] == 4 && test_vec[2] == 3 ));
    }

    SECTION( "Addition" )
    {
        Vector3D     test_vec1( 1, 2, 3 );
        Vector3D     test_vec2( 4, 5, 6 );

        Vector3D    result = test_vec1 + test_vec2;

        REQUIRE((( result.x() == 5 ) && ( result.y() == 7 ) && ( result.z() == 9 )));
    }

    SECTION( "Subtraction" )
    {
        Vector3D     test_vec1( 1, 2, 8.5 );
        Vector3D     test_vec2( 4, 6, 3 );

        Vector3D    result = test_vec1 - test_vec2;

        REQUIRE((( result.x() == -3 ) && ( result.y() == -4 ) && ( result.z() == 5.5 )));
    }
    
    SECTION( "Scalar Multiplication" )
    {
        Vector3D     test_vec1( 11, 13, 14 );

        Vector3D    result = test_vec1 * 3.5;

        REQUIRE((( result.x() == 11 * 3.5 ) && ( result.y() == 13 * 3.5 ) && ( result.z() == 14 * 3.5 )));

        result = 5.0 * test_vec1;

        REQUIRE( result == Vector3D( 55, 65, 70 ) );
    }
    
    SECTION( "Scalar Multiplication" )
    {
        Vector3D     test_vec1( 11, 13, 14 );

        Vector3D    result = -4.25 * test_vec1;

        REQUIRE((( result.x() == 11 * -4.25 ) && ( result.y() == 13 * -4.25 ) && ( result.z() == 14 * -4.25 )));
    }
    
    SECTION( "Scalar Division" )
    {
        Vector3D     test_vec1( 23, 32, 51 );

        Vector3D    result = test_vec1 / 5.6;

        REQUIRE((( result.x() == 23 / 5.6 ) && ( result.y() == 32 / 5.6 ) && ( result.z() == 51 / 5.6 )));
    }
        
    SECTION( "Negation and ABS" )
    {
        Vector3D     test_vec1( -23, -32, -51 );

        Vector3D    result = test_vec1.abs();

        REQUIRE((( result.x() == 23 ) && ( result.y() == 32 ) && ( result.z() == 51 )));

        result = -test_vec1;
        
        REQUIRE((( result.x() == 23 ) && ( result.y() == 32 ) && ( result.z() == 51 )));
    }

    SECTION( "Destructive Addition" )
    {
        Vector3D     test_vec1( 1, 2, 3 );
        Vector3D     test_vec2( 4, 5, 6 );

        test_vec1 += test_vec2;

        REQUIRE((( test_vec1.x() == 5 ) && ( test_vec1.y() == 7 ) && ( test_vec1.z() == 9 )));
    }

    SECTION( "Destructive Subtraction" )
    {
        Vector3D     test_vec1( 1, 2, 8.5 );
        Vector3D     test_vec2( 4, 6, 3 );

        test_vec1 -= test_vec2;

        REQUIRE((( test_vec1.x() == -3 ) && ( test_vec1.y() == -4 ) && ( test_vec1.z() == 5.5 )));
    }
    
    SECTION( "Destructive Scalar Multiplication" )
    {
        Vector3D     test_vec1( 11, 13, 14 );

        test_vec1 *= 3.5;

        REQUIRE((( test_vec1.x() == 11 * 3.5 ) && ( test_vec1.y() == 13 * 3.5 ) && ( test_vec1.z() == 14 * 3.5 )));
    }
    
    SECTION( "Destructive Scalar Division" )
    {
        Vector3D     test_vec1( 23, 32, 51 );

        test_vec1 /= 5.6;

        REQUIRE((( test_vec1.x() == 23 / 5.6 ) && ( test_vec1.y() == 32 / 5.6 ) && ( test_vec1.z() == 51 / 5.6 )));
    }

    SECTION( "Equality and Inequality" )
    {
        Vector3D     test_vec1( 1, 2, 3 );
        Vector3D     test_vec2( 1, 2, 3 );

        REQUIRE( test_vec1 == test_vec2 );
        REQUIRE( !( test_vec1 != test_vec2 ) );

        test_vec2 = Vector3D( 10, 2, 3 );

        REQUIRE( !(test_vec1 == test_vec2 ));
        REQUIRE( test_vec1 != test_vec2 );

        test_vec2 = Vector3D( 1, 20, 3 );

        REQUIRE( !(test_vec1 == test_vec2 ));
        REQUIRE( test_vec1 != test_vec2 );

        test_vec2 = Vector3D( 1, 2, 30 );

        REQUIRE( !(test_vec1 == test_vec2 ));
        REQUIRE( test_vec1 != test_vec2 );
    }

    SECTION( "Max" )
    {
        Vector3D     test_vec1( 1, 20, 3 );
        Vector3D     test_vec2( 1, 2, 3 );
        Vector3D     test_vec3( 100, 2, 3 );

        REQUIRE( test_vec1.max() == 20 );
        REQUIRE( test_vec1.min() == 1 );

        REQUIRE( test_vec1.max( test_vec2 ) == Vector3D( 1, 20, 3 ) );

        test_vec1 = Vector3D( 1, 2, 30 );

        REQUIRE( test_vec1.max( test_vec2 ) == Vector3D( 1, 2, 30 ) );

        test_vec2 = Vector3D( 10, 2, 3 );

        REQUIRE( test_vec1.max( test_vec2 ) == Vector3D( 10, 2, 30 ) );

        REQUIRE( test_vec1.max( test_vec2, test_vec3 ) == Vector3D( 100, 2, 30 ) );

        test_vec3 = Vector3D( 1, 200, 3 );

        REQUIRE( test_vec1.max( test_vec2, test_vec3 ) == Vector3D( 10, 200, 30 ) );

        test_vec3 = Vector3D( 1, 2, 300 );

        REQUIRE( test_vec1.max( test_vec2, test_vec3 ) == Vector3D( 10, 2, 300 ) );
    }

    SECTION( "Min" )
    {
        Vector3D     test_vec1( -1, 20, 3 );
        Vector3D     test_vec2( 1, 2, 3 );
        Vector3D     test_vec3( 100, 2, -4 );

        REQUIRE( test_vec1.min( test_vec2 ) == Vector3D( -1, 2, 3 ) );

        test_vec1 = Vector3D( 1, -2, 30 );

        REQUIRE( test_vec1.min( test_vec2 ) == Vector3D( 1, -2, 3 ) );

        test_vec2 = Vector3D( 10, 2, -3 );

        REQUIRE( test_vec1.min( test_vec2 ) == Vector3D( 1, -2, -3 ) );

        REQUIRE( test_vec1.min( test_vec2, test_vec3 ) == Vector3D( 1, -2, -4 ) );

        test_vec3 = Vector3D( -1, 200, 3 );

        REQUIRE( test_vec1.min( test_vec2, test_vec3 ) == Vector3D( -1, -2, -3 ) );

        test_vec3 = Vector3D( -1, -5, 300 );

        REQUIRE( test_vec1.min( test_vec2, test_vec3 ) == Vector3D( -1, -5, -3 ) );
    }

    SECTION( "Max and Min Dimension" )
    {
        REQUIRE( Vector3D( 1, 2, 3 ).maxDim() == 2 );
        REQUIRE( Vector3D( 1, 3, 2 ).maxDim() == 1 );
        REQUIRE( Vector3D( 3, 2, 1 ).maxDim() == 0 );

        REQUIRE( Vector3D( 1, 2, 3 ).minDim() == 0 );
        REQUIRE( Vector3D( 3, 1, 2 ).minDim() == 1 );
        REQUIRE( Vector3D( 3, 2, 1 ).minDim() == 2 );
    }

    SECTION( "Length" )
    {
        Vector3D     test_vec( -1, 20, 3 );

        REQUIRE( test_vec.len() == sqrt( 410 ) );
        REQUIRE( test_vec.len_squared() == 410 );
    }

    SECTION( "Normalization" )
    {
        Vector3D    test_vec( 4, 5, 6 );
        NUMERIC_PRECISION   length = test_vec.len();

        test_vec.normalize();

        REQUIRE(( test_vec.x() == ( 4 / length) && test_vec.y() == ( 5 / length ) && test_vec.z() == ( 6 / length) ));

        test_vec = Vector3D( 7, 8, 9 ).normalized();
        length = Vector3D( 7, 8, 9 ).len();

        REQUIRE(( test_vec.x() == 7 / length && test_vec.y() == 8 / length && test_vec.z() == 9 / length ));
    }

    SECTION( "Dot and Cross" )
    {
        Vector3D     test_vec1( 1, 2, 3 );
        Vector3D     test_vec2( 4, 5, 6 );

        REQUIRE( test_vec1.dot( test_vec2 ) == 4 + 10 + 18 );

        REQUIRE( test_vec1.cross( test_vec2 ) == Vector3D( -3, 6, -3 ) );
    }

    SECTION( "Determinant" )
    {
        Vector3D        test_vec1( 3, -2, 5 );
        Vector3D        test_vec2( 7, 4, -8 );
        Vector3D        test_vec3( 5, -3, -4 );

        REQUIRE( determinant( test_vec1, test_vec2, test_vec3 ) == -301 );
    }

    SECTION( "Projection" )
    {
        Vector3D     test_vec( 4, 5, 6 );

        REQUIRE(( test_vec.project( 0 ) == Vector2D( 5, 6 ) ));
        REQUIRE(( test_vec.project( 1 ) == Vector2D( 6, 4 ) ));
        REQUIRE(( test_vec.project( 2 ) == Vector2D( 4, 5 ) ));
    }
}

