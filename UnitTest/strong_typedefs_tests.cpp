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

#include "primitives/primitives.hpp"

#include <type_traits>

//  The pragma below is to disable to false errors flagged by intellisense for Catch2 REQUIRE macros.

#if __INTELLISENSE__
#pragma diag_suppress 2486
#endif


TEST_CASE("Strong Typedef Tests", "[core]")
{
    //  This test is just static asserts to insure that the strong typing works correctly

    SECTION("VertexIndex and TriangleByIndicesIndex")
    {
        static_assert( !std::is_convertible<Cork::Primitives::VertexIndex, Cork::Primitives::TriangleByIndicesIndex>::value );
        static_assert( !std::is_convertible<Cork::Primitives::TriangleByIndicesIndex, Cork::Primitives::VertexIndex>::value );

        static_assert( !std::is_assignable<Cork::Primitives::VertexIndex, Cork::Primitives::TriangleByIndicesIndex>::value );
        static_assert( !std::is_assignable<Cork::Primitives::TriangleByIndicesIndex, Cork::Primitives::VertexIndex>::value );

        static_assert( !std::is_constructible<Cork::Primitives::VertexIndex, Cork::Primitives::TriangleByIndicesIndex>::value );
        static_assert( !std::is_constructible<Cork::Primitives::TriangleByIndicesIndex, Cork::Primitives::VertexIndex>::value );
    }
}