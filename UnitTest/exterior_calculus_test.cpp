// Copyright (c) 2021 Stephan Friedl
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files (the "Software"), to deal in
// the Software without restriction, including without limitation the rights to
// use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
// the Software, and to permit persons to whom the Software is furnished to do so,
// subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
// IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
// CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <catch2/catch_all.hpp>

#include "math/ext4.hpp"
#include "math/fixext4.hpp"
#include "math/gmpext4.hpp"

//  The pragma below is to disable to false errors flagged by intellisense for Catch2 REQUIRE macros.

#if __INTELLISENSE__
#pragma diag_suppress 2486
#endif

namespace Cork
{
    TEST_CASE("Exterior Calculus Tests", "[ext calc basics]")
    {
        SECTION("Basic Ext4_1 Functionality")
        {
            //  Create from a vector, assignment and equality

            Cork::Math::Vector3D test_vec(7, 4, 3);

            Math::ExteriorCalculusR4::Ext4_1 test_R4_ext(test_vec);
            Math::ExteriorCalculusR4::Ext4_1 test_R4_ext_copy(test_R4_ext);

            REQUIRE(((test_R4_ext.e0() == 7) && (test_R4_ext.e1() == 4) && (test_R4_ext.e2() == 3) &&
                     (test_R4_ext.e3() == 1)));
            REQUIRE(((test_R4_ext[0] == 7) && (test_R4_ext[1] == 4) && (test_R4_ext[2] == 3) && (test_R4_ext[3] == 1)));

            REQUIRE(test_R4_ext == test_R4_ext_copy);
            REQUIRE(test_R4_ext != test_R4_ext_copy.negative());

            //  Negation - destructive and non-destructive.

            test_R4_ext.negate();

            REQUIRE(((test_R4_ext.e0() == -7) && (test_R4_ext.e1() == -4) && (test_R4_ext.e2() == -3) &&
                     (test_R4_ext.e3() == -1)));

            Math::ExteriorCalculusR4::Ext4_1 negated_test_R4_ext(test_R4_ext.negative());

            REQUIRE(((test_R4_ext.e0() == -7) && (test_R4_ext.e1() == -4) && (test_R4_ext.e2() == -3) &&
                     (test_R4_ext.e3() == -1)));
            REQUIRE(((negated_test_R4_ext.e0() == 7) && (negated_test_R4_ext.e1() == 4) &&
                     (negated_test_R4_ext.e2() == 3) && (negated_test_R4_ext.e3() == 1)));

            //  Dual and reverse dual
            //
            //  Dual of an Ext4_1 returns an Ext4_3 (n=4, k=1, n-k = 3).  Reverse Dual on the Ext4_3 returns an Ext4_1
            //  (n=4, k=3, n-k=1) which is identical to the original vector.  Same for Ext4_1::reverse_dual() and
            //  Ext4_3::dual()

            test_R4_ext = test_vec;

            Math::ExteriorCalculusR4::Ext4_3 dual = test_R4_ext.dual();
            Math::ExteriorCalculusR4::Ext4_3 reverse_dual = test_R4_ext.reverse_dual();

            REQUIRE(((dual.e012() == 1) && (dual.e013() == -3) && (dual.e023() == 4) && (dual.e123() == -7)));
            REQUIRE(((reverse_dual.e012() == -1) && (reverse_dual.e013() == 3) && (reverse_dual.e023() == -4) &&
                     (reverse_dual.e123() == 7)));

            Math::ExteriorCalculusR4::Ext4_1 original = dual.reverse_dual();

            REQUIRE(original == test_R4_ext);

            original = reverse_dual.dual();

            REQUIRE(original == test_R4_ext);

            //  Print to stream

            std::ostringstream test_stream;

            test_stream << test_R4_ext << std::flush;

            REQUIRE_THAT(test_stream.str(), Catch::Matchers::Equals("[7,4,3,1]"));
        }

        SECTION("Ext4_1 Join (Wedge Product)")
        {
            Math::ExteriorCalculusR4::Ext4_1 vec_1(Cork::Math::Vector3D(3, 4, 5));
            Math::ExteriorCalculusR4::Ext4_1 vec_2(Cork::Math::Vector3D(6, 7, 8));
            Math::ExteriorCalculusR4::Ext4_1 vec_3(Cork::Math::Vector3D(14, -10, 5));

            //  Compute wedge of two Ext4_1 vectors

            Math::ExteriorCalculusR4::Ext4_2 vec_1_wedge_vec_2 = vec_1.join(vec_2);

            REQUIRE(((vec_1_wedge_vec_2.e01() == -3) && (vec_1_wedge_vec_2.e02() == -6) &&
                     (vec_1_wedge_vec_2.e03() == -3) && (vec_1_wedge_vec_2.e12() == -3) &&
                     (vec_1_wedge_vec_2.e13() == -3) && (vec_1_wedge_vec_2.e23() == -3)));

            //  Now wedge of Ext4_1 vector and the Ext4_2.  This results in a
            //      Ext4_3.

            Math::ExteriorCalculusR4::Ext4_2 ext2_1 = vec_1_wedge_vec_2;

            Math::ExteriorCalculusR4::Ext4_3 vec_3_wedge_ext2_1 = vec_3.join(ext2_1);

            REQUIRE(((vec_3_wedge_ext2_1.e012() == -117) && (vec_3_wedge_ext2_1.e013() == -75) &&
                     (vec_3_wedge_ext2_1.e023() == -33) && (vec_3_wedge_ext2_1.e123() == 42)));
        }

        SECTION("Ext4_1 Inner (Dot Product)")
        {
            Math::ExteriorCalculusR4::Ext4_1 vec_1(Cork::Math::Vector3D(3, 4, 5));
            Math::ExteriorCalculusR4::Ext4_1 vec_2(Cork::Math::Vector3D(6, 7, 8));

            double dot_1_2 = vec_1.inner(vec_2);

            REQUIRE(dot_1_2 == 87);
        }

        SECTION("Basic Ext4_2 Functionality")
        {
            //  Create from a vector, assignment and equality

            Math::ExteriorCalculusR4::Ext4_1 vec_1(Cork::Math::Vector3D(7, 4, 3));
            Math::ExteriorCalculusR4::Ext4_1 vec_2(Cork::Math::Vector3D(11, 17, 13));

            Math::ExteriorCalculusR4::Ext4_2 ext2_1 = vec_1.join(vec_2);

            REQUIRE(((ext2_1.e01() == 75) && (ext2_1.e02() == 58) && (ext2_1.e03() == -4) && (ext2_1.e12() == 1) &&
                     (ext2_1.e13() == -13) && (ext2_1.e23() == -10)));
            REQUIRE(((ext2_1[0] == 75) && (ext2_1[1] == 58) && (ext2_1[2] == -4) && (ext2_1[3] == 1) && (ext2_1[4] == -13) &&
                     (ext2_1[5] == -10)));

            //  Negation - leave the ext2_1 back in its initial values

            Math::ExteriorCalculusR4::Ext4_2 ext2_1_negative = ext2_1.negative();

            REQUIRE(((ext2_1_negative.e01() == -75) && (ext2_1_negative.e02() == -58) && (ext2_1_negative.e03() == 4) &&
                     (ext2_1_negative.e12() == -1) && (ext2_1_negative.e13() == 13) && (ext2_1_negative.e23() == 10)));

            REQUIRE(ext2_1_negative != ext2_1);

            ext2_1.negate();

            REQUIRE(ext2_1_negative == ext2_1);

            ext2_1.negate();

            REQUIRE(((ext2_1.e01() == 75) && (ext2_1.e02() == 58) && (ext2_1.e03() == -4) && (ext2_1.e12() == 1) &&
                     (ext2_1.e13() == -13) && (ext2_1.e23() == -10)));

            //  Dual and Reverse Dual
            //
            //  Reverse dual of the dual is the original R4 2

            Math::ExteriorCalculusR4::Ext4_2 dual = ext2_1.dual();

            REQUIRE(((dual.e01() == -10) && (dual.e02() == 13) && (dual.e03() == 1) && (dual.e12() == -4) &&
                     (dual.e13() == -58) && (dual.e23() == 75)));

            Math::ExteriorCalculusR4::Ext4_2 original = dual.reverse_dual();

            REQUIRE(((original.e01() == 75) && (original.e02() == 58) && (original.e03() == -4) &&
                     (original.e12() == 1) && (original.e13() == -13) && (original.e23() == -10)));
            REQUIRE(original == ext2_1);

            //  Print to stream

            std::ostringstream test_stream;

            test_stream << ext2_1 << std::flush;

            REQUIRE_THAT(test_stream.str(), Catch::Matchers::Equals("[75,58,-4,1,-13,-10]"));
        }

        SECTION("Ext4_2 Join (Wedge Product)")
        {
            //  Create from a pair of vectors and check values

            Math::ExteriorCalculusR4::Ext4_1 vec_1(Cork::Math::Vector3D(7, 4, 3));
            Math::ExteriorCalculusR4::Ext4_1 vec_2(Cork::Math::Vector3D(11, 17, 13));

            Math::ExteriorCalculusR4::Ext4_2 ext2_1 = vec_1.join(vec_2);

            REQUIRE(((ext2_1.e01() == 75) && (ext2_1.e02() == 58) && (ext2_1.e03() == -4) && (ext2_1.e12() == 1) &&
                     (ext2_1.e13() == -13) && (ext2_1.e23() == -10)));
            REQUIRE(((ext2_1[0] == 75) && (ext2_1[1] == 58) && (ext2_1[2] == -4) && (ext2_1[3] == 1) && (ext2_1[4] == -13) &&
                     (ext2_1[5] == -10)));

            //  Compute the wedge product of the ext2_1 and the vector leaving an Ext4_3

            Math::ExteriorCalculusR4::Ext4_1 vec_3(Cork::Math::Vector3D(14, -10, 5));

            Math::ExteriorCalculusR4::Ext4_3 ext3_1 = ext2_1.join(vec_3);

            REQUIRE(((ext3_1.e012() == 969) && (ext3_1.e013() == -147) && (ext3_1.e023() == -62) &&
                     (ext3_1.e123() == 166)));
        }

        SECTION("Ext4_2 Inner (Dot Product)")
        {
            //  First Ext4_2

            Math::ExteriorCalculusR4::Ext4_1 vec_1(Cork::Math::Vector3D(7, 4, 3));
            Math::ExteriorCalculusR4::Ext4_1 vec_2(Cork::Math::Vector3D(11, 17, 13));

            Math::ExteriorCalculusR4::Ext4_2 ext2_1 = vec_1.join(vec_2);

            REQUIRE(((ext2_1.e01() == 75) && (ext2_1.e02() == 58) && (ext2_1.e03() == -4) && (ext2_1.e12() == 1) &&
                     (ext2_1.e13() == -13) && (ext2_1.e23() == -10)));
            REQUIRE(((ext2_1[0] == 75) && (ext2_1[1] == 58) && (ext2_1[2] == -4) && (ext2_1[3] == 1) &&
                     (ext2_1[4] == -13) && (ext2_1[5] == -10)));

            //  Second Ext4_2

            Math::ExteriorCalculusR4::Ext4_1 vec_3(Cork::Math::Vector3D(-5, 6, 9));
            Math::ExteriorCalculusR4::Ext4_1 vec_4(Cork::Math::Vector3D(13, -17, 4));

            Math::ExteriorCalculusR4::Ext4_2 ext2_2 = vec_3.join(vec_4);

            REQUIRE(((ext2_2.e01() == 7) && (ext2_2.e02() == -137) && (ext2_2.e03() == -18) &&
                     (ext2_2.e12() == 177) && (ext2_2.e13() == 23) && (ext2_2.e23() == 5)));

            //  Inner product

            double dot_product = ext2_1.inner(ext2_2);

            REQUIRE(dot_product == -7521);
        }

        SECTION("Triangle Edge Intersection with Ext4_2 Meet")
        {
            Math::ExteriorCalculusR4::Ext4_1 tri_1(Cork::Math::Vector3D(2, 2, 1));
            Math::ExteriorCalculusR4::Ext4_1 tri_2(Cork::Math::Vector3D(7, 4, 2));
            Math::ExteriorCalculusR4::Ext4_1 tri_3(Cork::Math::Vector3D(4, 9, 3));

            Math::ExteriorCalculusR4::Ext4_1 edge_1(Cork::Math::Vector3D(2, 2, 3));
            Math::ExteriorCalculusR4::Ext4_1 edge_2(Cork::Math::Vector3D(6, 7, 0));

            //  Build the Ext4_3 for the triangle

            Math::ExteriorCalculusR4::Ext4_3 t_ext3((tri_1.join(tri_2)).join(tri_3));

            //  Now the Ext4_2 for the edge

            Math::ExteriorCalculusR4::Ext4_2 e_ext2(edge_1.join(edge_2));

            //  Compute the point of intersection.  Meet does some duals and a join.

            Math::ExteriorCalculusR4::Ext4_1 intersection(e_ext2.meet(t_ext3));

            //  Convert back into a point, which is mostly just normalizing by e3.
            //      Negative e3 values will be resolved by the division.

            Cork::Math::Vector3D result(intersection);

            REQUIRE(((result.x() == Catch::Approx(538.0 / 145.0).epsilon(0.00001)) &&
                     (result.y() == Catch::Approx(120.0 / 29.0).epsilon(0.00001)) &&
                     (result.z() == Catch::Approx(249.0 / 145.0).epsilon(0.00001))));
        }

        SECTION("Ext4_3 Basic Tests")
        {
            Math::ExteriorCalculusR4::Ext4_1 tri_1(Cork::Math::Vector3D(2, 2, 1));
            Math::ExteriorCalculusR4::Ext4_1 tri_2(Cork::Math::Vector3D(7, 4, 2));
            Math::ExteriorCalculusR4::Ext4_1 tri_3(Cork::Math::Vector3D(4, 9, 3));

            //  Build the Ext4_3 for the triangle

            Math::ExteriorCalculusR4::Ext4_3 t_ext3((tri_1.join(tri_2)).join(tri_3));

            REQUIRE(((t_ext3.e012() == 9) && (t_ext3.e013() == 31) && (t_ext3.e023() == 8) && (t_ext3.e123() == -3)));
            REQUIRE(((t_ext3[0] == 9) && (t_ext3[1] == 31) && (t_ext3[2] == 8) && (t_ext3[3] == -3)));

            //  Negation

            Math::ExteriorCalculusR4::Ext4_3 t_ext3_negative(t_ext3.negative());

            REQUIRE(((t_ext3_negative.e012() == -9) && (t_ext3_negative.e013() == -31) &&
                     (t_ext3_negative.e023() == -8) && (t_ext3_negative.e123() == 3)));

            t_ext3.negate();

            REQUIRE(t_ext3 == t_ext3_negative);

            t_ext3.negate();

            REQUIRE(t_ext3 != t_ext3_negative);
            REQUIRE(((t_ext3.e012() == 9) && (t_ext3.e013() == 31) && (t_ext3.e023() == 8) && (t_ext3.e123() == -3)));

            //  Dual and Reverse Dual

            Math::ExteriorCalculusR4::Ext4_1 t_ext3_dual(t_ext3.dual());

            REQUIRE(((t_ext3_dual.e0() == -3) && (t_ext3_dual.e1() == -8) && (t_ext3_dual.e2() == 31) &&
                     (t_ext3_dual.e3() == -9)));

            Math::ExteriorCalculusR4::Ext4_1 t_ext3_reverse_dual(t_ext3.reverse_dual());

            REQUIRE(((t_ext3_reverse_dual.e0() == 3) && (t_ext3_reverse_dual.e1() == 8) &&
                     (t_ext3_reverse_dual.e2() == -31) && (t_ext3_reverse_dual.e3() == 9)));

            //  Print to stream

            std::ostringstream test_stream;

            test_stream << t_ext3 << std::flush;

            REQUIRE_THAT(test_stream.str(), Catch::Matchers::Equals("[9,31,8,-3]"));
        }

        SECTION("Triangle Triangle Triangle Intersection with Ext4_3 Meet")
        {
            Math::ExteriorCalculusR4::Ext4_1 tri1_1(Cork::Math::Vector3D(1, 3, 1));
            Math::ExteriorCalculusR4::Ext4_1 tri1_2(Cork::Math::Vector3D(4, 7, 4));
            Math::ExteriorCalculusR4::Ext4_1 tri1_3(Cork::Math::Vector3D(7, 9, 3));

            Math::ExteriorCalculusR4::Ext4_1 tri2_1(Cork::Math::Vector3D(2, 2, 3));
            Math::ExteriorCalculusR4::Ext4_1 tri2_2(Cork::Math::Vector3D(6, 7, 3));
            Math::ExteriorCalculusR4::Ext4_1 tri2_3(Cork::Math::Vector3D(4, 8, 4));

            Math::ExteriorCalculusR4::Ext4_1 tri3_1(Cork::Math::Vector3D(2, 2, 4));
            Math::ExteriorCalculusR4::Ext4_1 tri3_2(Cork::Math::Vector3D(6, 7, 1));
            Math::ExteriorCalculusR4::Ext4_1 tri3_3(Cork::Math::Vector3D(4, 8, 5));

            //  Build the Ext4_3 for the triangle 1

            Math::ExteriorCalculusR4::Ext4_3 t1_ext3((tri1_1.join(tri1_2)).join(tri1_3));

            //  Build the Ext4_3 for the triangle 2

            Math::ExteriorCalculusR4::Ext4_3 t2_ext3((tri2_1.join(tri2_2)).join(tri2_3));

            //  Build the Ext4_3 for the triangle 3

            Math::ExteriorCalculusR4::Ext4_3 t3_ext3((tri3_1.join(tri3_2)).join(tri3_3));

            //  Compute the point of intersection.  First we get an edge on the first two triangles and then
            //      we find the intersection of that edge with the third triangle.

            Math::ExteriorCalculusR4::Ext4_2 intersect_edge_1_2(t1_ext3.meet(t2_ext3));
            Math::ExteriorCalculusR4::Ext4_1 intersection(intersect_edge_1_2.meet(t3_ext3));

            //  Convert back into a point, which is mostly just normalizing by e3.
            //      Negative e3 values will be resolved by the division.

            Cork::Math::Vector3D result(intersection);

            REQUIRE(((result.x() == Catch::Approx(104.0 / 23.0).epsilon(0.00001)) &&
                     (result.y() == Catch::Approx(499.0 / 69.0).epsilon(0.00001)) &&
                     (result.z() == Catch::Approx(248.0 / 69.0).epsilon(0.00001))));
        }

        //*******************************************************************************************************
        //
        //  ABS Ext classes next
        //
        //*******************************************************************************************************

        SECTION("Basic AbsExt4_1 Functionality")
        {
            //  Create from a vector, assignment and equality

            Cork::Math::Vector3D test_vec(-7, -4, -3);
            Cork::Math::Vector3D test_vec_2(-7, 4, -3);

            Math::ExteriorCalculusR4::Ext4_1 temp(test_vec);

            Math::ExteriorCalculusR4::AbsExt4_1 test_R4_ext(temp);
            Math::ExteriorCalculusR4::AbsExt4_1 test_R4_ext_copy(test_R4_ext);

            REQUIRE(((test_R4_ext.e0() == 7) && (test_R4_ext.e1() == 4) && (test_R4_ext.e2() == 3) &&
                     (test_R4_ext.e3() == 1)));
            REQUIRE(((test_R4_ext[0] == 7) && (test_R4_ext[1] == 4) && (test_R4_ext[2] == 3) && (test_R4_ext[3] == 1)));

            temp = test_vec_2;

            REQUIRE(test_R4_ext == test_R4_ext_copy);
            REQUIRE(test_R4_ext == Math::ExteriorCalculusR4::AbsExt4_1(temp));

            //  Negation - destructive and non-destructive.

            test_R4_ext.negate();

            REQUIRE(((test_R4_ext.e0() == 7) && (test_R4_ext.e1() == 4) && (test_R4_ext.e2() == 3) &&
                     (test_R4_ext.e3() == 1)));

            Math::ExteriorCalculusR4::AbsExt4_1 negated_test_R4_ext(test_R4_ext.negative());

            REQUIRE(((test_R4_ext.e0() == 7) && (test_R4_ext.e1() == 4) && (test_R4_ext.e2() == 3) &&
                     (test_R4_ext.e3() == 1)));
            REQUIRE(((negated_test_R4_ext.e0() == 7) && (negated_test_R4_ext.e1() == 4) &&
                     (negated_test_R4_ext.e2() == 3) && (negated_test_R4_ext.e3() == 1)));

            //  Dual and reverse dual
            //
            //  Dual of an Ext4_1 returns an Ext4_3 (n=4, k=1, n-k = 3).  Reverse Dual on the Ext4_3 returns an Ext4_1
            //  (n=4, k=3, n-k=1) which is identical to the original vector.  Same for Ext4_1::reverse_dual() and
            //  Ext4_3::dual()

            test_R4_ext = Math::ExteriorCalculusR4::AbsExt4_1(Math::ExteriorCalculusR4::Ext4_1(test_vec));

            Math::ExteriorCalculusR4::AbsExt4_3 dual = test_R4_ext.dual();
            Math::ExteriorCalculusR4::AbsExt4_3 reverse_dual = test_R4_ext.reverse_dual();

            REQUIRE(((dual.e012() == 1) && (dual.e013() == 3) && (dual.e023() == 4) && (dual.e123() == 7)));
            REQUIRE(((reverse_dual.e012() == 1) && (reverse_dual.e013() == 3) && (reverse_dual.e023() == 4) &&
                     (reverse_dual.e123() == 7)));

            Math::ExteriorCalculusR4::AbsExt4_1 original = dual.reverse_dual();

            REQUIRE(original == test_R4_ext);

            original = reverse_dual.dual();

            REQUIRE(original == test_R4_ext);

            //  Print to stream

            std::ostringstream test_stream;

            test_stream << test_R4_ext << std::flush;

            REQUIRE_THAT(test_stream.str(), Catch::Matchers::Equals("[7,4,3,1]"));
        }

        SECTION("AbsExt4_1 Join (Wedge Product)")
        {
            Math::ExteriorCalculusR4::AbsExt4_1 vec_1(Math::ExteriorCalculusR4::Ext4_1(Cork::Math::Vector3D(-3, 4, 5)));
            Math::ExteriorCalculusR4::AbsExt4_1 vec_2(Math::ExteriorCalculusR4::Ext4_1(Cork::Math::Vector3D(6, -7, 8)));
            Math::ExteriorCalculusR4::AbsExt4_1 vec_3(Math::ExteriorCalculusR4::Ext4_1(Cork::Math::Vector3D(14, 10, -5)));

            //  Compute wedge of two AbsExt4_1 vectors

            Math::ExteriorCalculusR4::AbsExt4_2 vec_1_wedge_vec_2 = vec_1.join(vec_2);

            REQUIRE(((vec_1_wedge_vec_2.e01() == 45) && (vec_1_wedge_vec_2.e02() == 54) &&
                     (vec_1_wedge_vec_2.e03() == 9) && (vec_1_wedge_vec_2.e12() == 67) &&
                     (vec_1_wedge_vec_2.e13() == 11) && (vec_1_wedge_vec_2.e23() == 13)));

            //  Now wedge of AbsExt4_1 vector and the AbsExt4_2 from the wedge operation above.  This results in a
            //  AbsExt4_3.

            Math::ExteriorCalculusR4::AbsExt4_3 vec_1_wedge_vec_2_wedge_vec_3 = vec_3.join(vec_1_wedge_vec_2);

            REQUIRE(((vec_1_wedge_vec_2_wedge_vec_3.e012() == 1703) && (vec_1_wedge_vec_2_wedge_vec_3.e013() == 289) &&
                     (vec_1_wedge_vec_2_wedge_vec_3.e023() == 281) && (vec_1_wedge_vec_2_wedge_vec_3.e123() == 252)));
        }

        SECTION("AbsExt4_1 Inner (Dot Product)")
        {
            Math::ExteriorCalculusR4::AbsExt4_1 vec_1(Math::ExteriorCalculusR4::Ext4_1(Cork::Math::Vector3D(3, -4, 5)));
            Math::ExteriorCalculusR4::AbsExt4_1 vec_2(Math::ExteriorCalculusR4::Ext4_1(Cork::Math::Vector3D(6, 7, -8)));

            double dot_1_2 = vec_1.inner(vec_2);

            REQUIRE(dot_1_2 == 87);
        }

        SECTION("Basic AbsExt4_2 Functionality")
        {
            //  Create from an Ext4_2 from joining a pair of vectors and create an AbsExt2 from it

            Math::ExteriorCalculusR4::Ext4_1 vec_1(Cork::Math::Vector3D(7, 4, 3));
            Math::ExteriorCalculusR4::Ext4_1 vec_2(Cork::Math::Vector3D(11, 17, 13));

            Math::ExteriorCalculusR4::Ext4_2 ext2 = vec_1.join(vec_2);

            REQUIRE(((ext2.e01() == 75) && (ext2.e02() == 58) && (ext2.e03() == -4) && (ext2.e12() == 1) &&
                     (ext2.e13() == -13) && (ext2.e23() == -10)));

            Math::ExteriorCalculusR4::AbsExt4_2 absext2(ext2);

            REQUIRE(((absext2.e01() == 75) && (absext2.e02() == 58) && (absext2.e03() == 4) && (absext2.e12() == 1) &&
                     (absext2.e13() == 13) && (absext2.e23() == 10)));

            //  Negation - nothing should change - all values remain positive

            Math::ExteriorCalculusR4::AbsExt4_2 absext2_negative = absext2.negative();

            REQUIRE(((absext2_negative.e01() == 75) && (absext2_negative.e02() == 58) &&
                     (absext2_negative.e03() == 4) && (absext2_negative.e12() == 1) && (absext2_negative.e13() == 13) &&
                     (absext2_negative.e23() == 10)));

            REQUIRE(absext2_negative == absext2);

            Math::ExteriorCalculusR4::Ext4_2 ext2_negative =
                ext2.negative();  //  This flips all the signs to insure we get for
                                  //  abs of all the data members in the Ext4_2

            REQUIRE(absext2_negative == Math::ExteriorCalculusR4::AbsExt4_2(ext2_negative));

            absext2_negative.negate();

            REQUIRE(absext2_negative == absext2);

            //  Dual and Reverse Dual
            //
            //  Reverse dual of the dual is the original R4 2

            Math::ExteriorCalculusR4::AbsExt4_2 dual = absext2.dual();

            REQUIRE(((dual.e01() == 10) && (dual.e02() == 13) && (dual.e03() == 1) && (dual.e12() == 4) &&
                     (dual.e13() == 58) && (dual.e23() == 75)));

            Math::ExteriorCalculusR4::AbsExt4_2 original = dual.reverse_dual();

            REQUIRE(((original.e01() == 75) && (original.e02() == 58) && (original.e03() == 4) &&
                     (original.e12() == 1) && (original.e13() == 13) && (original.e23() == 10)));
            REQUIRE(original == absext2);

            //  Print to stream

            std::ostringstream test_stream;

            test_stream << original << std::flush;

            REQUIRE_THAT(test_stream.str(), Catch::Matchers::Equals("[75,58,4,1,13,10]"));
        }

        SECTION("AbsExt4_2 Join (Wedge Product)")
        {
            //  Create from a pair of vectors and check values

            Math::ExteriorCalculusR4::AbsExt4_1 vec_1(Math::ExteriorCalculusR4::Ext4_1(Cork::Math::Vector3D(7, 4, 3)));
            Math::ExteriorCalculusR4::AbsExt4_1 vec_2(Math::ExteriorCalculusR4::Ext4_1(Cork::Math::Vector3D(11, 17, 13)));

            Math::ExteriorCalculusR4::AbsExt4_2 ext2 = vec_1.join(vec_2);

            REQUIRE(((ext2.e01() == 163) && (ext2.e02() == 124) && (ext2.e03() == 18) && (ext2.e12() == 103) &&
                     (ext2.e13() == 21) && (ext2.e23() == 16)));
            REQUIRE(((ext2[0] == 163) && (ext2[1] == 124) && (ext2[2] == 18) && (ext2[3] == 103) && (ext2[4] == 21) &&
                     (ext2[5] == 16)));

            //  Compute the wedge product of the Ext4_2 and the vector leaving an Ext4_3

            Math::ExteriorCalculusR4::AbsExt4_1 vec_3(Math::ExteriorCalculusR4::Ext4_1(Cork::Math::Vector3D(14, -10, 5)));

            Math::ExteriorCalculusR4::AbsExt4_3 ext3 = ext2.join(vec_3);

            REQUIRE(((ext3.e012() == 3497) && (ext3.e013() == 637) && (ext3.e023() == 438) && (ext3.e123() == 368)));
        }

        SECTION("AbsExt4_2 Inner (Dot Product)")
        {
            //  First Ext4_2

            Math::ExteriorCalculusR4::Ext4_1 vec_1(Cork::Math::Vector3D(7, 4, 3));
            Math::ExteriorCalculusR4::Ext4_1 vec_2(Cork::Math::Vector3D(11, 17, 13));

            Math::ExteriorCalculusR4::Ext4_2 ext2_1 = vec_1.join(vec_2);

            REQUIRE(((ext2_1.e01() == 75) && (ext2_1.e02() == 58) && (ext2_1.e03() == -4) && (ext2_1.e12() == 1) &&
                     (ext2_1.e13() == -13) && (ext2_1.e23() == -10)));

            //  Second Ext4_2

            Math::ExteriorCalculusR4::Ext4_1 vec_3(Cork::Math::Vector3D(-5, 6, 9));
            Math::ExteriorCalculusR4::Ext4_1 vec_4(Cork::Math::Vector3D(13, -17, 4));

            Math::ExteriorCalculusR4::Ext4_2 ext2_2 = vec_3.join(vec_4);

            REQUIRE(((ext2_2.e01() == 7) && (ext2_2.e02() == -137) && (ext2_2.e03() == -18) && (ext2_2.e12() == 177) &&
                     (ext2_2.e13() == 23) && (ext2_2.e23() == 5)));

            //  Inner product

            Math::ExteriorCalculusR4::AbsExt4_2 absext_1(ext2_1);
            Math::ExteriorCalculusR4::AbsExt4_2 absext_2(ext2_2);

            double dot_product = absext_1.inner(absext_2);

            REQUIRE(dot_product == 9069);
        }

        SECTION("Triangle Edge Intersection with AbsExt4_2 Meet")
        {
            Math::ExteriorCalculusR4::AbsExt4_1 tri_1(Math::ExteriorCalculusR4::Ext4_1(Cork::Math::Vector3D(2, 2, 1)));
            Math::ExteriorCalculusR4::AbsExt4_1 tri_2(Math::ExteriorCalculusR4::Ext4_1(Cork::Math::Vector3D(7, 4, 2)));
            Math::ExteriorCalculusR4::AbsExt4_1 tri_3(Math::ExteriorCalculusR4::Ext4_1(Cork::Math::Vector3D(4, 9, 3)));

            Math::ExteriorCalculusR4::AbsExt4_1 edge_1(Math::ExteriorCalculusR4::Ext4_1(Cork::Math::Vector3D(2, 2, 3)));
            Math::ExteriorCalculusR4::AbsExt4_1 edge_2(Math::ExteriorCalculusR4::Ext4_1(Cork::Math::Vector3D(6, 7, 0)));

            //  Build the Ext4_3 for the triangle

            Math::ExteriorCalculusR4::AbsExt4_3 t_ext3((tri_1.join(tri_2)).join(tri_3));

            //  Now the Ext4_2 for the edge

            Math::ExteriorCalculusR4::AbsExt4_2 e_ext2(edge_1.join(edge_2));

            //  Compute the point of intersection.  Meet does some duals and a join.

            Math::ExteriorCalculusR4::AbsExt4_1 intersection(e_ext2.meet(t_ext3));

            //  Convert back into a point, which is mostly just normalizing by e3.
            //      Negative e3 values will be resolved by the division.

            Cork::Math::Vector3D result(intersection);

            REQUIRE(((result.x() == Catch::Approx(5162.0 / 1255.0).epsilon(0.00001)) &&
                     (result.y() == Catch::Approx(5818.0 / 1255.0).epsilon(0.00001)) &&
                     (result.z() == Catch::Approx(519.0 / 251.0).epsilon(0.00001))));
        }

        SECTION("AbsExt4_3 Basic Tests")
        {
            Math::ExteriorCalculusR4::AbsExt4_1 tri_1(Math::ExteriorCalculusR4::Ext4_1(Cork::Math::Vector3D(2, -2, 1)));
            Math::ExteriorCalculusR4::AbsExt4_1 tri_2(Math::ExteriorCalculusR4::Ext4_1(Cork::Math::Vector3D(-7, 4, 2)));
            Math::ExteriorCalculusR4::AbsExt4_1 tri_3(Math::ExteriorCalculusR4::Ext4_1(Cork::Math::Vector3D(4, 9, -3)));

            //  Build the AbsExt4_3 for the triangle

            Math::ExteriorCalculusR4::AbsExt4_3 t_ext3((tri_1.join(tri_2)).join(tri_3));

            REQUIRE(
                ((t_ext3.e012() == 197) && (t_ext3.e013() == 127) && (t_ext3.e023() == 50) && (t_ext3.e123() == 53)));
            REQUIRE(((t_ext3[0] == 197) && (t_ext3[1] == 127) && (t_ext3[2] == 50) && (t_ext3[3] == 53)));

            //  Negation - does nothing

            Math::ExteriorCalculusR4::AbsExt4_3 t_ext3_negative(t_ext3.negative());

            REQUIRE(((t_ext3_negative.e012() == 197) && (t_ext3_negative.e013() == 127) &&
                     (t_ext3_negative.e023() == 50) && (t_ext3_negative.e123() == 53)));

            t_ext3.negate();

            REQUIRE(t_ext3 == t_ext3_negative);

            t_ext3.negate();

            REQUIRE(t_ext3 == t_ext3_negative);
            REQUIRE(
                ((t_ext3.e012() == 197) && (t_ext3.e013() == 127) && (t_ext3.e023() == 50) && (t_ext3.e123() == 53)));

            //  Dual and Reverse Dual - are the same, reversed order of elements

            Math::ExteriorCalculusR4::AbsExt4_1 t_ext3_dual(t_ext3.dual());

            REQUIRE(((t_ext3_dual.e0() == 53) && (t_ext3_dual.e1() == 50) && (t_ext3_dual.e2() == 127) &&
                     (t_ext3_dual.e3() == 197)));

            Math::ExteriorCalculusR4::AbsExt4_1 t_ext3_reverse_dual(t_ext3.reverse_dual());

            REQUIRE(t_ext3_reverse_dual == t_ext3_dual);

            //  Print to stream

            std::ostringstream test_stream;

            test_stream << t_ext3 << std::flush;

            REQUIRE_THAT(test_stream.str(), Catch::Matchers::Equals("[197,127,50,53]"));
        }

        SECTION("Triangle Triangle Triangle Intersection with AbsExt4_3 Meet")
        {
            Math::ExteriorCalculusR4::AbsExt4_1 tri1_1(Math::ExteriorCalculusR4::Ext4_1(Cork::Math::Vector3D(1, 3, 1)));
            Math::ExteriorCalculusR4::AbsExt4_1 tri1_2(Math::ExteriorCalculusR4::Ext4_1(Cork::Math::Vector3D(4, 7, 4)));
            Math::ExteriorCalculusR4::AbsExt4_1 tri1_3(Math::ExteriorCalculusR4::Ext4_1(Cork::Math::Vector3D(7, 9, 3)));

            Math::ExteriorCalculusR4::AbsExt4_1 tri2_1(Math::ExteriorCalculusR4::Ext4_1(Cork::Math::Vector3D(2, 2, 3)));
            Math::ExteriorCalculusR4::AbsExt4_1 tri2_2(Math::ExteriorCalculusR4::Ext4_1(Cork::Math::Vector3D(6, 7, 3)));
            Math::ExteriorCalculusR4::AbsExt4_1 tri2_3(Math::ExteriorCalculusR4::Ext4_1(Cork::Math::Vector3D(4, 8, 4)));

            Math::ExteriorCalculusR4::AbsExt4_1 tri3_1(Math::ExteriorCalculusR4::Ext4_1(Cork::Math::Vector3D(2, 2, 4)));
            Math::ExteriorCalculusR4::AbsExt4_1 tri3_2(Math::ExteriorCalculusR4::Ext4_1(Cork::Math::Vector3D(6, 7, 1)));
            Math::ExteriorCalculusR4::AbsExt4_1 tri3_3(Math::ExteriorCalculusR4::Ext4_1(Cork::Math::Vector3D(4, 8, 5)));

            //  Build the AbsExt4_3 for the triangle 1

            Math::ExteriorCalculusR4::AbsExt4_3 t1_ext3((tri1_1.join(tri1_2)).join(tri1_3));

            //  Build the AbsExt4_3 for the triangle 2

            Math::ExteriorCalculusR4::AbsExt4_3 t2_ext3((tri2_1.join(tri2_2)).join(tri2_3));

            //  Build the AbsExt4_3 for the triangle 3

            Math::ExteriorCalculusR4::AbsExt4_3 t3_ext3((tri3_1.join(tri3_2)).join(tri3_3));

            //  Compute the point of intersection.  First we get an edge on the first two triangles and then
            //      we find the intersection of that edge with the third triangle.

            Math::ExteriorCalculusR4::AbsExt4_2 intersect_edge_1_2(t1_ext3.meet(t2_ext3));
            Math::ExteriorCalculusR4::AbsExt4_1 intersection(intersect_edge_1_2.meet(t3_ext3));

            //  Convert back into a point, which is mostly just normalizing by e3.
            //      Negative e3 values will be resolved by the division.

            Cork::Math::Vector3D result(intersection);

            REQUIRE(((result.x() == Catch::Approx(5336236.0 / 1532419.0).epsilon(0.00001)) &&
                     (result.y() == Catch::Approx(7677043.0 / 1532419.0).epsilon(0.00001)) &&
                     (result.z() == Catch::Approx(4316208.0 / 1532419.0).epsilon(0.00001))));
        }

        //***********************************************************************************************
        //
        //      FixInt flavors follow
        //
        //***********************************************************************************************

        SECTION("Basic FixExt4_1 Functionality")
        {
            Cork::Quantization::Quantizer::GetQuantizerResult get_quantizer_result =
                Cork::Quantization::Quantizer::get_quantizer(10e2, 10e-2);

            REQUIRE(get_quantizer_result.succeeded());

            Cork::Quantization::Quantizer quantizer(get_quantizer_result.return_value());

            //  Create from a vector, assignment and equality

            Cork::Math::Vector3D test_vec(7, 4, 3);

            Math::ExteriorCalculusR4::FixExt4_1<FIXED_INTEGER_BITS> test_R4_ext(test_vec, quantizer);

            Math::ExteriorCalculusR4::FixExt4_1<FIXED_INTEGER_BITS> test_R4_ext_copy(test_R4_ext);

            REQUIRE(((test_R4_ext.e0() == 3670016) && (test_R4_ext.e1() == 2097152) && (test_R4_ext.e2() == 1572864) &&
                     (test_R4_ext.e3() == 1)));
            REQUIRE(((test_R4_ext[0] == 3670016) && (test_R4_ext[1] == 2097152) && (test_R4_ext[2] == 1572864) &&
                     (test_R4_ext[3] == 1)));

            REQUIRE(test_R4_ext == test_R4_ext_copy);
            REQUIRE(test_R4_ext != test_R4_ext_copy.negative());

            //  Negation - destructive and non-destructive.

            test_R4_ext.negate();

            REQUIRE(((test_R4_ext.e0() == -3670016) && (test_R4_ext.e1() == -2097152) &&
                     (test_R4_ext.e2() == -1572864) && (test_R4_ext.e3() == -1)));

            Math::ExteriorCalculusR4::FixExt4_1 negated_test_R4_ext(test_R4_ext.negative());

            REQUIRE(((test_R4_ext.e0() == -3670016) && (test_R4_ext.e1() == -2097152) &&
                     (test_R4_ext.e2() == -1572864) && (test_R4_ext.e3() == -1)));
            REQUIRE(((negated_test_R4_ext.e0() == 3670016) && (negated_test_R4_ext.e1() == 2097152) &&
                     (negated_test_R4_ext.e2() == 1572864) && (negated_test_R4_ext.e3() == 1)));

            test_R4_ext.negate();

            REQUIRE(((test_R4_ext.e0() == 3670016) && (test_R4_ext.e1() == 2097152) && (test_R4_ext.e2() == 1572864) &&
                     (test_R4_ext.e3() == 1)));

            //  Dual and reverse dual
            //
            //  Dual of an Ext4_1 returns an Ext4_3 (n=4, k=1, n-k = 3).  Reverse Dual on the Ext4_3 returns an Ext4_1
            //  (n=4, k=3, n-k=1) which is identical to the original vector.  Same for Ext4_1::reverse_dual() and
            //  Ext4_3::dual()

            Math::ExteriorCalculusR4::FixExt4_3<FIXED_INTEGER_BITS> dual = test_R4_ext.dual();
            Math::ExteriorCalculusR4::FixExt4_3<FIXED_INTEGER_BITS> reverse_dual = test_R4_ext.reverse_dual();

            REQUIRE(((dual.e012() == 1) && (dual.e013() == -1572864) && (dual.e023() == 2097152) &&
                     (dual.e123() == -3670016)));
            REQUIRE(((reverse_dual.e012() == -1) && (reverse_dual.e013() == 1572864) &&
                     (reverse_dual.e023() == -2097152) && (reverse_dual.e123() == 3670016)));

            auto original = dual.reverse_dual();

            REQUIRE(original == test_R4_ext);

            original = reverse_dual.dual();

            REQUIRE(original == test_R4_ext);

            //  Print to stream

            std::ostringstream test_stream;

            test_stream << test_R4_ext << std::flush;

            REQUIRE_THAT(test_stream.str(), Catch::Matchers::Equals("[3670016,2097152,1572864,1]"));
        }

        SECTION("Ext4_1 Join (Wedge Product)")
        {
            Cork::Quantization::Quantizer::GetQuantizerResult get_quantizer_result =
                Cork::Quantization::Quantizer::get_quantizer(10e6, 100);

            REQUIRE(get_quantizer_result.succeeded());

            Cork::Quantization::Quantizer quantizer(get_quantizer_result.return_value());

            Math::ExteriorCalculusR4::FixExt4_1<FIXED_INTEGER_BITS> vec_1(
                Math::ExteriorCalculusR4::Ext4_1(Cork::Math::Vector3D(3, 4, 5)), quantizer);
            Math::ExteriorCalculusR4::FixExt4_1<FIXED_INTEGER_BITS> vec_2(
                Math::ExteriorCalculusR4::Ext4_1(Cork::Math::Vector3D(6, 7, 8)), quantizer);
            Math::ExteriorCalculusR4::FixExt4_1<FIXED_INTEGER_BITS> vec_3(
                Math::ExteriorCalculusR4::Ext4_1(Cork::Math::Vector3D(14, -10, 5)), quantizer);

            //  Compute wedge of two Ext4_1 vectors

            auto vec_1_wedge_vec_2 = vec_1.join(vec_2);

            REQUIRE(((vec_1_wedge_vec_2.e01() == -3072) && (vec_1_wedge_vec_2.e02() == -6144) &&
                     (vec_1_wedge_vec_2.e03() == -96) && (vec_1_wedge_vec_2.e12() == -3072) &&
                     (vec_1_wedge_vec_2.e13() == -96) && (vec_1_wedge_vec_2.e23() == -96)));

            //  Now wedge of Ext4_1 vector and the Ext4_2.  This results in a Ext4_3.

            auto ext2_1 = vec_1_wedge_vec_2;

            auto vec_3_wedge_ext2 = vec_1_wedge_vec_2.join(vec_3);

            REQUIRE(((vec_3_wedge_ext2.e012() == -3833856) && (vec_3_wedge_ext2.e013() == -76800) &&
                     (vec_3_wedge_ext2.e023() == -33792) && (vec_3_wedge_ext2.e123() == 43008)));
        }

        SECTION("FixExt4_1 Inner (Dot Product)")
        {
            Cork::Quantization::Quantizer::GetQuantizerResult get_quantizer_result =
                Cork::Quantization::Quantizer::get_quantizer(10e2, 10e-2);

            REQUIRE(get_quantizer_result.succeeded());

            Cork::Quantization::Quantizer quantizer(get_quantizer_result.return_value());

            Math::ExteriorCalculusR4::FixExt4_1<FIXED_INTEGER_BITS> fixext1_1(Cork::Math::Vector3D(3, 4, 5), quantizer);
            Math::ExteriorCalculusR4::FixExt4_1<FIXED_INTEGER_BITS> fixext1_2(Cork::Math::Vector3D(6, 7, 8), quantizer);

            auto dot_1_2 = fixext1_1.inner(fixext1_2);

            REQUIRE(dot_1_2 == 23639499997185);
        }

        SECTION("Basic FixExt4_2 Functionality")
        {
            //  Create from a vector, assignment and equality

            Cork::Quantization::Quantizer::GetQuantizerResult get_quantizer_result =
                Cork::Quantization::Quantizer::get_quantizer(10e2, 10e-2);

            REQUIRE(get_quantizer_result.succeeded());

            Cork::Quantization::Quantizer quantizer(get_quantizer_result.return_value());

            Math::ExteriorCalculusR4::FixExt4_1<FIXED_INTEGER_BITS> fixext1_1(Cork::Math::Vector3D(7, 4, 3), quantizer);
            Math::ExteriorCalculusR4::FixExt4_1<FIXED_INTEGER_BITS> fixext1_2(Cork::Math::Vector3D(11, 17, 13), quantizer);

            auto fixext2_1(fixext1_1.join(fixext1_2));

            auto fixext2_1_copy(fixext2_1);

            REQUIRE(((fixext2_1.e01() == 20615843020800) && (fixext2_1.e02() == 15942918602752) &&
                     (fixext2_1.e03() == -2097152) && (fixext2_1.e12() == 274877906944) &&
                     (fixext2_1.e13() == -6815744) && (fixext2_1.e23() == -5242880)));
            REQUIRE(((fixext2_1_copy[0] == 20615843020800) && (fixext2_1_copy[1] == 15942918602752) &&
                     (fixext2_1_copy[2] == -2097152) && (fixext2_1_copy[3] == 274877906944) &&
                     (fixext2_1_copy[4] == -6815744) && (fixext2_1_copy[5] == -5242880)));

            REQUIRE(fixext2_1 == fixext2_1_copy);
            REQUIRE(fixext2_1 != fixext2_1.negative());

            //  Negation - destructive and non-destructive.

            fixext2_1.negate();

            REQUIRE(((fixext2_1.e01() == -20615843020800) && (fixext2_1.e02() == -15942918602752) &&
                     (fixext2_1.e03() == 2097152) && (fixext2_1.e12() == -274877906944) &&
                     (fixext2_1.e13() == 6815744) && (fixext2_1.e23() == 5242880)));

            Math::ExteriorCalculusR4::FixExt4_2 fixext2_1_double_negative(fixext2_1.negative());

            REQUIRE(fixext2_1_double_negative == fixext2_1_copy);

            fixext2_1.negate();

            REQUIRE(fixext2_1 == fixext2_1_copy);

            //  Dual and reverse dual
            //
            //  Dual of an Ext4_1 returns an Ext4_3 (n=4, k=1, n-k = 3).  Reverse Dual on the Ext4_3 returns an Ext4_1
            //  (n=4, k=3, n-k=1) which is identical to the original vector.  Same for Ext4_1::reverse_dual() and
            //  Ext4_3::dual()

            auto dual = fixext2_1.dual();
            auto reverse_dual = fixext2_1.reverse_dual();

            REQUIRE(((dual.e01() == -5242880) && (dual.e02() == 6815744) && (dual.e03() == 274877906944) &&
                     (dual.e12() == -2097152) && (dual.e13() == -15942918602752) && (dual.e23() == 20615843020800)));

            REQUIRE(((reverse_dual.e01() == -5242880) && (reverse_dual.e02() == 6815744) &&
                     (reverse_dual.e03() == 274877906944) && (reverse_dual.e12() == -2097152) &&
                     (reverse_dual.e13() == -15942918602752) && (reverse_dual.e23() == 20615843020800)));

            auto original = dual.reverse_dual();

            REQUIRE(original == fixext2_1);

            original = reverse_dual.dual();

            REQUIRE(original == fixext2_1);

            //  Print to stream

            std::ostringstream test_stream;

            test_stream << original << std::flush;

            REQUIRE_THAT(test_stream.str(), Catch::Matchers::Equals("[20615843020800,15942918602752,-2097152,274877906944,-6815744,-5242880]"));
        }

        SECTION("FixExt4_2 Inner (Dot Product)")
        {
            //  Create from a vector, assignment and equality

            Cork::Quantization::Quantizer::GetQuantizerResult get_quantizer_result =
                Cork::Quantization::Quantizer::get_quantizer(10e6, 100);

            REQUIRE(get_quantizer_result.succeeded());

            Cork::Quantization::Quantizer quantizer(get_quantizer_result.return_value());

            Math::ExteriorCalculusR4::FixExt4_1<FIXED_INTEGER_BITS> fixext1_1(Cork::Math::Vector3D(7, 4, 3), quantizer);
            Math::ExteriorCalculusR4::FixExt4_1<FIXED_INTEGER_BITS> fixext1_2(Cork::Math::Vector3D(11, 17, 13), quantizer);
            Math::ExteriorCalculusR4::FixExt4_1<FIXED_INTEGER_BITS> fixext1_3(Cork::Math::Vector3D(14, 9, 10), quantizer);
            Math::ExteriorCalculusR4::FixExt4_1<FIXED_INTEGER_BITS> fixext1_4(Cork::Math::Vector3D(19, 18, 16), quantizer);

            auto fixext2_1(fixext1_1.join(fixext1_2));
            auto fixext2_2(fixext1_3.join(fixext1_4));

            auto dot_1_2 = fixext2_1.inner(fixext2_2);

            REQUIRE(dot_1_2 == 8400344064);
        }

        SECTION("Basic FixExt4_3 Functionality")
        {
            //  Create from a vector, assignment and equality

            Cork::Quantization::Quantizer::GetQuantizerResult get_quantizer_result =
                Cork::Quantization::Quantizer::get_quantizer(10e6, 100);

            REQUIRE(get_quantizer_result.succeeded());

            Cork::Quantization::Quantizer quantizer(get_quantizer_result.return_value());

            Math::ExteriorCalculusR4::FixExt4_1<FIXED_INTEGER_BITS> fixext1_1(Cork::Math::Vector3D(7, 4, 3), quantizer);
            Math::ExteriorCalculusR4::FixExt4_1<FIXED_INTEGER_BITS> fixext1_2(Cork::Math::Vector3D(11, 17, 13), quantizer);
            Math::ExteriorCalculusR4::FixExt4_1<FIXED_INTEGER_BITS> fixext1_3(Cork::Math::Vector3D(14, 9, 10), quantizer);

            auto fixext3_1((fixext1_1.join(fixext1_2)).join(fixext1_3));

            auto fixext3_1_copy(fixext3_1);

            REQUIRE(((fixext3_1.e012() == 7929856) && (fixext3_1.e013() == -72704) && (fixext3_1.e023() == -43008) &&
                     (fixext3_1.e123() == 41984)));
            REQUIRE(((fixext3_1_copy[0] == 7929856) && (fixext3_1_copy[1] == -72704) && (fixext3_1_copy[2] == -43008) &&
                     (fixext3_1_copy[3] == 41984)));

            REQUIRE(fixext3_1 == fixext3_1_copy);
            REQUIRE(fixext3_1 != fixext3_1.negative());

            //  Negation - destructive and non-destructive.

            fixext3_1.negate();

            REQUIRE(((fixext3_1.e012() == -7929856) && (fixext3_1.e013() == 72704) && (fixext3_1.e023() == 43008) &&
                     (fixext3_1.e123() == -41984)));

            auto fixext3_1_double_negative(fixext3_1.negative());

            REQUIRE(fixext3_1_double_negative == fixext3_1_copy);

            fixext3_1.negate();

            REQUIRE(fixext3_1 == fixext3_1_copy);

            //  Dual and reverse dual
            //
            //  Dual of an Ext4_1 returns an Ext4_3 (n=4, k=1, n-k = 3).  Reverse Dual on the Ext4_3 returns an Ext4_1
            //  (n=4, k=3, n-k=1) which is identical to the original vector.  Same for Ext4_1::reverse_dual() and
            //  Ext4_3::dual()

            auto dual = fixext3_1.dual();
            auto reverse_dual = fixext3_1.reverse_dual();

            REQUIRE(((dual.e0() == 41984) && (dual.e1() == 43008) && (dual.e2() == -72704) && (dual.e3() == -7929856)));
            REQUIRE(((reverse_dual.e0() == -41984) && (reverse_dual.e1() == -43008) && (reverse_dual.e2() == 72704) &&
                     (reverse_dual.e3() == 7929856)));

            auto original = dual.reverse_dual();

            REQUIRE(original == fixext3_1);

            original = reverse_dual.dual();

            REQUIRE(original == fixext3_1);

            //  Print to stream

            std::ostringstream test_stream;

            test_stream << original << std::flush;

            REQUIRE_THAT(test_stream.str(), Catch::Matchers::Equals("[7929856,-72704,-43008,41984]"));
        }

        SECTION("FixExt4_3 Inner (Dot Product)")
        {
            //  Create from a vector, assignment and equality

            Cork::Quantization::Quantizer::GetQuantizerResult get_quantizer_result =
                Cork::Quantization::Quantizer::get_quantizer(10e6, 100);

            REQUIRE(get_quantizer_result.succeeded());

            Cork::Quantization::Quantizer quantizer(get_quantizer_result.return_value());

            Math::ExteriorCalculusR4::FixExt4_1<FIXED_INTEGER_BITS> fixext1_1(Cork::Math::Vector3D(7, 4, 3), quantizer);
            Math::ExteriorCalculusR4::FixExt4_1<FIXED_INTEGER_BITS> fixext1_2(Cork::Math::Vector3D(11, 17, 13), quantizer);
            Math::ExteriorCalculusR4::FixExt4_1<FIXED_INTEGER_BITS> fixext1_3(Cork::Math::Vector3D(14, 9, 10), quantizer);
            Math::ExteriorCalculusR4::FixExt4_1<FIXED_INTEGER_BITS> fixext1_4(Cork::Math::Vector3D(19, 18, 16), quantizer);
            Math::ExteriorCalculusR4::FixExt4_1<FIXED_INTEGER_BITS> fixext1_5(Cork::Math::Vector3D(14, 22, -6), quantizer);
            Math::ExteriorCalculusR4::FixExt4_1<FIXED_INTEGER_BITS> fixext1_6(Cork::Math::Vector3D(-5, -10, -15), quantizer);

            auto fixext3_1((fixext1_1.join(fixext1_2)).join(fixext1_3));
            auto fixext3_2((fixext1_4.join(fixext1_5)).join(fixext1_6));

            auto dot_1_2 = fixext3_1.inner(fixext3_2);

            REQUIRE(dot_1_2 == -927681468170240);
        }

        //***********************************************************************************************
        //
        //      GMPExt4 flavors follow
        //
        //***********************************************************************************************

        SECTION("Basic GMPExt4_1 Functionality")
        {
            Cork::Quantization::Quantizer::GetQuantizerResult get_quantizer_result =
                Cork::Quantization::Quantizer::get_quantizer(10e6, 100);

            REQUIRE(get_quantizer_result.succeeded());

            Cork::Quantization::Quantizer quantizer(get_quantizer_result.return_value());

            //  Create from a vector, assignment and equality

            Cork::Math::Vector3D test_vec(7, 4, 3);

            Math::ExteriorCalculusR4::GMPExt4_1 test_R4_ext(test_vec, quantizer);
            Math::ExteriorCalculusR4::GMPExt4_1 test_R4_ext_copy(test_R4_ext);

            REQUIRE(((test_R4_ext.e0() == 224) && (test_R4_ext.e1() == 128) && (test_R4_ext.e2() == 96) &&
                     (test_R4_ext.e3() == 1)));
            REQUIRE(((test_R4_ext[0] == 224) && (test_R4_ext[1] == 128) && (test_R4_ext[2] == 96) &&
                     (test_R4_ext[3] == 1)));

            REQUIRE(test_R4_ext == test_R4_ext_copy);
            REQUIRE(test_R4_ext != test_R4_ext_copy.negative());

            //  Negation - destructive and non-destructive.

            test_R4_ext.negate();

            REQUIRE(((test_R4_ext.e0() == -224) && (test_R4_ext.e1() == -128) && (test_R4_ext.e2() == -96) &&
                     (test_R4_ext.e3() == -1)));

            Math::ExteriorCalculusR4::GMPExt4_1 negated_test_R4_ext(test_R4_ext.negative());

            REQUIRE(((test_R4_ext.e0() == -224) && (test_R4_ext.e1() == -128) && (test_R4_ext.e2() == -96) &&
                     (test_R4_ext.e3() == -1)));
            REQUIRE(((negated_test_R4_ext.e0() == 224) && (negated_test_R4_ext.e1() == 128) &&
                     (negated_test_R4_ext.e2() == 96) && (negated_test_R4_ext.e3() == 1)));

            //  Dual and reverse dual
            //
            //  Dual of an Ext4_1 returns an Ext4_3 (n=4, k=1, n-k = 3).  Reverse Dual on the Ext4_3 returns an Ext4_1
            //  (n=4, k=3, n-k=1) which is identical to the original vector.  Same for Ext4_1::reverse_dual() and
            //  Ext4_3::dual()

            test_R4_ext = Math::ExteriorCalculusR4::GMPExt4_1(test_vec, quantizer);

            Math::ExteriorCalculusR4::GMPExt4_3 dual = test_R4_ext.dual();
            Math::ExteriorCalculusR4::GMPExt4_3 reverse_dual = test_R4_ext.reverse_dual();

            REQUIRE(((dual.e012() == 1) && (dual.e013() == -96) && (dual.e023() == 128) && (dual.e123() == -224)));
            REQUIRE(((reverse_dual.e012() == -1) && (reverse_dual.e013() == 96) && (reverse_dual.e023() == -128) &&
                     (reverse_dual.e123() == 224)));

            Math::ExteriorCalculusR4::GMPExt4_1 original = dual.reverse_dual();

            REQUIRE(original == test_R4_ext);

            original = reverse_dual.dual();

            REQUIRE(original == test_R4_ext);

            //  Print to stream

            std::ostringstream test_stream;

            test_stream << test_R4_ext << std::flush;

            REQUIRE_THAT(test_stream.str(), Catch::Matchers::Equals("[224,128,96,1]"));
        }

        SECTION("GMPExt4_1 Join (Wedge Product)")
        {
            Cork::Quantization::Quantizer::GetQuantizerResult get_quantizer_result =
                Cork::Quantization::Quantizer::get_quantizer(10e6, 100);

            REQUIRE(get_quantizer_result.succeeded());

            Cork::Quantization::Quantizer quantizer(get_quantizer_result.return_value());

            //  Create from a vector, assignment and equality

            Math::ExteriorCalculusR4::GMPExt4_1 vec_1(Cork::Math::Vector3D(3, 4, 5), quantizer);
            Math::ExteriorCalculusR4::GMPExt4_1 vec_2(Cork::Math::Vector3D(6, 7, 8), quantizer);
            Math::ExteriorCalculusR4::GMPExt4_1 vec_3(Cork::Math::Vector3D(14, -10, 5), quantizer);

            //  Compute wedge of two GMPExt4_1 vectors

            Math::ExteriorCalculusR4::GMPExt4_2 vec_1_wedge_vec_2 = vec_1.join(vec_2);

            REQUIRE(((vec_1_wedge_vec_2.e01() == -3072) && (vec_1_wedge_vec_2.e02() == -6144) &&
                     (vec_1_wedge_vec_2.e03() == -96) && (vec_1_wedge_vec_2.e12() == -3072) &&
                     (vec_1_wedge_vec_2.e13() == -96) && (vec_1_wedge_vec_2.e23() == -96)));

            //  Now wedge of GMPExt4_1 vector and the GMPExt4_2.

            Math::ExteriorCalculusR4::GMPExt4_2 ext4_2 = vec_1_wedge_vec_2;

            Math::ExteriorCalculusR4::GMPExt4_3 vec_3_wedge_ext4_2 = vec_3.join(ext4_2);

            REQUIRE(((vec_3_wedge_ext4_2.e012() == -3833856) && (vec_3_wedge_ext4_2.e013() == -76800) &&
                     (vec_3_wedge_ext4_2.e023() == -33792) && (vec_3_wedge_ext4_2.e123() == 43008)));
        }

        SECTION("GMPExt4_1 Inner (Dot Product)")
        {
            Cork::Quantization::Quantizer::GetQuantizerResult get_quantizer_result =
                Cork::Quantization::Quantizer::get_quantizer(10e6, 100);

            REQUIRE(get_quantizer_result.succeeded());

            Cork::Quantization::Quantizer quantizer(get_quantizer_result.return_value());

            Math::ExteriorCalculusR4::GMPExt4_1 vec_1(Cork::Math::Vector3D(3, 4, 5), quantizer);
            Math::ExteriorCalculusR4::GMPExt4_1 vec_2(Cork::Math::Vector3D(6, 7, 8), quantizer);

            auto dot_1_2 = vec_1.inner(vec_2);

            REQUIRE(dot_1_2 == mpz_class(18432 + 28672 + 40960 + 1));
        }

        SECTION("Basic GMPExt4_2 Functionality")
        {
            Cork::Quantization::Quantizer::GetQuantizerResult get_quantizer_result =
                Cork::Quantization::Quantizer::get_quantizer(10e6, 100);

            REQUIRE(get_quantizer_result.succeeded());

            Cork::Quantization::Quantizer quantizer(get_quantizer_result.return_value());

            //  Create from a vector, assignment and equality

            Math::ExteriorCalculusR4::GMPExt4_1 vec_1(Cork::Math::Vector3D(7, 4, 3), quantizer);
            Math::ExteriorCalculusR4::GMPExt4_1 vec_2(Cork::Math::Vector3D(11, 17, 13), quantizer);

            Math::ExteriorCalculusR4::GMPExt4_2 ext2_1 = vec_1.join(vec_2);

            REQUIRE(((ext2_1.e01() == 76800) && (ext2_1.e02() == 59392) && (ext2_1.e03() == -128) &&
                     (ext2_1.e12() == 1024) && (ext2_1.e13() == -416) && (ext2_1.e23() == -320)));
            REQUIRE(((ext2_1[0] == 76800) && (ext2_1[1] == 59392) && (ext2_1[2] == -128) && (ext2_1[3] == 1024) &&
                     (ext2_1[4] == -416) && (ext2_1[5] == -320)));

            //  Negation - leave the ext2_1 back in its initial values

            Math::ExteriorCalculusR4::GMPExt4_2 ext2_1_negative = ext2_1.negative();

            REQUIRE(((ext2_1_negative.e01() == -76800) && (ext2_1_negative.e02() == -59392) &&
                     (ext2_1_negative.e03() == 128) && (ext2_1_negative.e12() == -1024) &&
                     (ext2_1_negative.e13() == 416) && (ext2_1_negative.e23() == 320)));

            REQUIRE(ext2_1_negative != ext2_1);

            ext2_1.negate();

            REQUIRE(ext2_1_negative == ext2_1);

            ext2_1.negate();

            REQUIRE(((ext2_1.e01() == 76800) && (ext2_1.e02() == 59392) && (ext2_1.e03() == -128) &&
                     (ext2_1.e12() == 1024) && (ext2_1.e13() == -416) && (ext2_1.e23() == -320)));

            //  Dual and Reverse Dual
            //
            //  Reverse dual of the dual is the original R4 2

            Math::ExteriorCalculusR4::GMPExt4_2 dual = ext2_1.dual();

            REQUIRE(((dual.e01() == -320) && (dual.e02() == 416) && (dual.e03() == 1024) && (dual.e12() == -128) &&
                     (dual.e13() == -59392) && (dual.e23() == 76800)));

            Math::ExteriorCalculusR4::GMPExt4_2 original = dual.reverse_dual();

            REQUIRE(((original.e01() == 76800) && (original.e02() == 59392) && (original.e03() == -128) &&
                     (original.e12() == 1024) && (original.e13() == -416) && (original.e23() == -320)));
            REQUIRE(original == ext2_1);

            //  Print to stream

            std::ostringstream test_stream;

            test_stream << ext2_1 << std::flush;

            REQUIRE_THAT(test_stream.str(), Catch::Matchers::Equals("[76800,59392,-128,1024,-416,-320]"));
        }

        SECTION("GMPExt4_2 Join (Wedge Product)")
        {
            Cork::Quantization::Quantizer::GetQuantizerResult get_quantizer_result =
                Cork::Quantization::Quantizer::get_quantizer(10e6, 100);

            REQUIRE(get_quantizer_result.succeeded());

            Cork::Quantization::Quantizer quantizer(get_quantizer_result.return_value());

            //  Create from a pair of vectors and check values

            Math::ExteriorCalculusR4::GMPExt4_1 vec_1(Cork::Math::Vector3D(7, 4, 3), quantizer);
            Math::ExteriorCalculusR4::GMPExt4_1 vec_2(Cork::Math::Vector3D(11, 17, 13), quantizer);

            Math::ExteriorCalculusR4::GMPExt4_2 ext2_1 = vec_1.join(vec_2);

            REQUIRE(((ext2_1.e01() == 76800) && (ext2_1.e02() == 59392) && (ext2_1.e03() == -128) &&
                     (ext2_1.e12() == 1024) && (ext2_1.e13() == -416) && (ext2_1.e23() == -320)));

            //  Compute the wedge product of the ext2_1 and the vector leaving an Ext4_3

            Math::ExteriorCalculusR4::GMPExt4_1 vec_3(Cork::Math::Vector3D(14, -10, 5), quantizer);

            Math::ExteriorCalculusR4::GMPExt4_3 ext3_1 = ext2_1.join(vec_3);

            REQUIRE(((ext3_1.e012() == 31752192) && (ext3_1.e013() == -150528) && (ext3_1.e023() == -63488) &&
                     (ext3_1.e123() == 169984)));
        }

        SECTION("GMPExt4_2 Inner (Dot Product)")
        {
            Cork::Quantization::Quantizer::GetQuantizerResult get_quantizer_result =
                Cork::Quantization::Quantizer::get_quantizer(10e6, 100);

            REQUIRE(get_quantizer_result.succeeded());

            Cork::Quantization::Quantizer quantizer(get_quantizer_result.return_value());

            //  First GMPExt4_2

            Math::ExteriorCalculusR4::GMPExt4_1 vec_1(Cork::Math::Vector3D(7, 4, 3), quantizer);
            Math::ExteriorCalculusR4::GMPExt4_1 vec_2(Cork::Math::Vector3D(11, 17, 13), quantizer);

            Math::ExteriorCalculusR4::GMPExt4_2 ext2_1 = vec_1.join(vec_2);

            REQUIRE(((ext2_1.e01() == 76800) && (ext2_1.e02() == 59392) && (ext2_1.e03() == -128) &&
                     (ext2_1.e12() == 1024) && (ext2_1.e13() == -416) && (ext2_1.e23() == -320)));
            REQUIRE(((ext2_1[0] == 76800) && (ext2_1[1] == 59392) && (ext2_1[2] == -128) && (ext2_1[3] == 1024) &&
                     (ext2_1[4] == -416) && (ext2_1[5] == -320)));

            //  Second GMPExt4_2

            Math::ExteriorCalculusR4::GMPExt4_1 vec_3(Cork::Math::Vector3D(-5, 6, 9), quantizer);
            Math::ExteriorCalculusR4::GMPExt4_1 vec_4(Cork::Math::Vector3D(13, -17, 4), quantizer);

            Math::ExteriorCalculusR4::GMPExt4_2 ext2_2 = vec_3.join(vec_4);

            REQUIRE(((ext2_2.e01() == 7168) && (ext2_2.e02() == -140288) && (ext2_2.e03() == -576) &&
                     (ext2_2.e12() == 181248) && (ext2_2.e13() == 736) && (ext2_2.e23() == 160)));

            //  Inner product

            auto dot_product = ext2_1.inner(ext2_2);

            REQUIRE(dot_product == mpz_class(-7596168192));
        }

        SECTION("Triangle Edge Intersection with GMPExt4_2 Meet")
        {
            Cork::Quantization::Quantizer::GetQuantizerResult get_quantizer_result =
                Cork::Quantization::Quantizer::get_quantizer(10e6, 100);

            REQUIRE(get_quantizer_result.succeeded());

            Cork::Quantization::Quantizer quantizer(get_quantizer_result.return_value());

            Math::ExteriorCalculusR4::GMPExt4_1 tri_1(Cork::Math::Vector3D(2, 2, 1), quantizer);
            Math::ExteriorCalculusR4::GMPExt4_1 tri_2(Cork::Math::Vector3D(7, 4, 2), quantizer);
            Math::ExteriorCalculusR4::GMPExt4_1 tri_3(Cork::Math::Vector3D(4, 9, 3), quantizer);

            Math::ExteriorCalculusR4::GMPExt4_1 edge_1(Cork::Math::Vector3D(2, 2, 3), quantizer);
            Math::ExteriorCalculusR4::GMPExt4_1 edge_2(Cork::Math::Vector3D(6, 7, 0), quantizer);

            //  Build the Ext4_3 for the triangle

            Math::ExteriorCalculusR4::GMPExt4_3 t_ext3((tri_1.join(tri_2)).join(tri_3));

            //  Now the Ext4_2 for the edge

            Math::ExteriorCalculusR4::GMPExt4_2 e_ext2(edge_1.join(edge_2));

            //  Compute the point of intersection.  Meet does some duals and a join.

            Math::ExteriorCalculusR4::GMPExt4_1 intersection(e_ext2.meet(t_ext3));

            //  Convert back into a point, which is mostly just normalizing by e3.
            //      Negative e3 values will be resolved by the division.

            Cork::Math::Vector3D result(intersection(quantizer));

            REQUIRE(((result.x() == Catch::Approx(538.0 / 145.0).epsilon(0.00001)) &&
                     (result.y() == Catch::Approx(120.0 / 29.0).epsilon(0.00001)) &&
                     (result.z() == Catch::Approx(249.0 / 145.0).epsilon(0.00001))));
        }

        SECTION("GMPExt4_3 Basic Tests")
        {
            Cork::Quantization::Quantizer::GetQuantizerResult get_quantizer_result =
                Cork::Quantization::Quantizer::get_quantizer(10e6, 100);

            REQUIRE(get_quantizer_result.succeeded());

            Cork::Quantization::Quantizer quantizer(get_quantizer_result.return_value());

            Math::ExteriorCalculusR4::GMPExt4_1 tri_1(Cork::Math::Vector3D(2, 2, 1), quantizer);
            Math::ExteriorCalculusR4::GMPExt4_1 tri_2(Cork::Math::Vector3D(7, 4, 2), quantizer);
            Math::ExteriorCalculusR4::GMPExt4_1 tri_3(Cork::Math::Vector3D(4, 9, 3), quantizer);

            //  Build the Ext4_3 for the triangle

            Math::ExteriorCalculusR4::GMPExt4_3 t_ext3((tri_1.join(tri_2)).join(tri_3));
            Math::ExteriorCalculusR4::GMPExt4_3 t_ext3_copy(t_ext3);

            REQUIRE(((t_ext3.e012() == 294912) && (t_ext3.e013() == 31744) && (t_ext3.e023() == 8192) &&
                     (t_ext3.e123() == -3072)));
            REQUIRE(((t_ext3_copy[0] == 294912) && (t_ext3_copy[1] == 31744) && (t_ext3_copy[2] == 8192) &&
                     (t_ext3_copy[3] == -3072)));

            //  Negation

            Math::ExteriorCalculusR4::GMPExt4_3 t_ext3_negative(t_ext3.negative());

            REQUIRE(((t_ext3_negative.e012() == -294912) && (t_ext3_negative.e013() == -31744) && (t_ext3_negative.e023() == -8192) &&
                     (t_ext3_negative.e123() == 3072)));

            t_ext3.negate();

            REQUIRE(t_ext3 == t_ext3_negative);

            t_ext3.negate();

            REQUIRE(t_ext3 != t_ext3_negative);

            REQUIRE(((t_ext3.e012() == 294912) && (t_ext3.e013() == 31744) && (t_ext3.e023() == 8192) &&
                     (t_ext3.e123() == -3072)));

            //  Dual and Reverse Dual

            Math::ExteriorCalculusR4::GMPExt4_1 t_ext3_dual(t_ext3.dual());

            REQUIRE(((t_ext3_dual.e0() == -3072) && (t_ext3_dual.e1() == -8192) && (t_ext3_dual.e2() == 31744) &&
                     (t_ext3_dual.e3() == -294912)));

            Math::ExteriorCalculusR4::GMPExt4_1 t_ext3_reverse_dual(t_ext3.reverse_dual());

            REQUIRE(((t_ext3_reverse_dual.e0() == 3072) && (t_ext3_reverse_dual.e1() == 8192) && (t_ext3_reverse_dual.e2() == -31744) &&
                     (t_ext3_reverse_dual.e3() == 294912)));

            //  Print to stream

            std::ostringstream test_stream;

            test_stream << t_ext3 << std::flush;

            REQUIRE_THAT(test_stream.str(), Catch::Matchers::Equals("[294912,31744,8192,-3072]"));
        }
        
        SECTION("Triangle Triangle Triangle Intersection with Ext4_3 Meet")
        {
            Cork::Quantization::Quantizer::GetQuantizerResult get_quantizer_result =
                Cork::Quantization::Quantizer::get_quantizer(10e6, 100);

            REQUIRE(get_quantizer_result.succeeded());

            Cork::Quantization::Quantizer quantizer(get_quantizer_result.return_value());

            Math::ExteriorCalculusR4::GMPExt4_1 tri1_1(Cork::Math::Vector3D(1, 3, 1), quantizer);
            Math::ExteriorCalculusR4::GMPExt4_1 tri1_2(Cork::Math::Vector3D(4, 7, 4), quantizer);
            Math::ExteriorCalculusR4::GMPExt4_1 tri1_3(Cork::Math::Vector3D(7, 9, 3), quantizer);

            Math::ExteriorCalculusR4::GMPExt4_1 tri2_1(Cork::Math::Vector3D(2, 2, 3), quantizer);
            Math::ExteriorCalculusR4::GMPExt4_1 tri2_2(Cork::Math::Vector3D(6, 7, 3), quantizer);
            Math::ExteriorCalculusR4::GMPExt4_1 tri2_3(Cork::Math::Vector3D(4, 8, 4), quantizer);

            Math::ExteriorCalculusR4::GMPExt4_1 tri3_1(Cork::Math::Vector3D(2, 2, 4), quantizer);
            Math::ExteriorCalculusR4::GMPExt4_1 tri3_2(Cork::Math::Vector3D(6, 7, 1), quantizer);
            Math::ExteriorCalculusR4::GMPExt4_1 tri3_3(Cork::Math::Vector3D(4, 8, 5), quantizer);

            //  Build the Ext4_3 for the triangle 1

            Math::ExteriorCalculusR4::GMPExt4_3 t1_ext3((tri1_1.join(tri1_2)).join(tri1_3));

            //  Build the Ext4_3 for the triangle 2

            Math::ExteriorCalculusR4::GMPExt4_3 t2_ext3((tri2_1.join(tri2_2)).join(tri2_3));

            //  Build the Ext4_3 for the triangle 3

            Math::ExteriorCalculusR4::GMPExt4_3 t3_ext3((tri3_1.join(tri3_2)).join(tri3_3));

            //  Compute the point of intersection.  First we get an edge on the first two triangles and then
            //      we find the intersection of that edge with the third triangle.

            Math::ExteriorCalculusR4::GMPExt4_2 intersect_edge_1_2(t1_ext3.meet(t2_ext3));
            Math::ExteriorCalculusR4::GMPExt4_1 intersection(intersect_edge_1_2.meet(t3_ext3));

            //  Convert back into a point, which is mostly just normalizing by e3.
            //      Negative e3 values will be resolved by the division.

            Cork::Math::Vector3D result(intersection(quantizer));

            REQUIRE(((result.x() == Catch::Approx(104.0 / 23.0).epsilon(0.00001)) &&
                     (result.y() == Catch::Approx(499.0 / 69.0).epsilon(0.00001)) &&
                     (result.z() == Catch::Approx(248.0 / 69.0).epsilon(0.00001))));
        }
    }

    TEST_CASE("Exterior Calculus Benchmarks", "[ext calc perf]")
    {
        BENCHMARK_ADVANCED("Ext4_1 Dot Product")(Catch::Benchmark::Chronometer meter)
        {
            double dot_1_2_sum = 0;

            meter.measure([&dot_1_2_sum] {
                Math::ExteriorCalculusR4::Ext4_1 vec_1(Cork::Math::Vector3D(3, 4, 5));
                Math::ExteriorCalculusR4::Ext4_1 vec_2(Cork::Math::Vector3D(6, 7, 8));

                for (auto i = 0; i < 2000000; i++)
                {
                    dot_1_2_sum += vec_1.inner(vec_2);

                    vec_1.negate();  //  Negate is needed to prevent the compiler from optimizing out the operations
                }
            });
            return dot_1_2_sum;
        };

        BENCHMARK_ADVANCED("Ext4_1 Join Ext4_1")(Catch::Benchmark::Chronometer meter)
        {
            double running_sum = 0;

            meter.measure([&running_sum] {
                Math::ExteriorCalculusR4::Ext4_1 vec_1(Cork::Math::Vector3D(3, 4, 5));
                Math::ExteriorCalculusR4::Ext4_1 vec_2(Cork::Math::Vector3D(6, 7, 8));

                for (auto i = 0; i < 2000000; i++)
                {
                    Math::ExteriorCalculusR4::Ext4_2 wedge_product = vec_1.join(vec_2);

                    running_sum += wedge_product[0] + wedge_product[1] + wedge_product[2] + wedge_product[3] +
                                   wedge_product[4] + wedge_product[5];

                    vec_1.negate();  //  Negate is needed to prevent the compiler from optimizing out the operations
                }
            });
            return running_sum;
        };

        BENCHMARK_ADVANCED("Ext4_1 Join Ext4_2")(Catch::Benchmark::Chronometer meter)
        {
            double running_sum = 0;

            meter.measure([&running_sum] {
                Math::ExteriorCalculusR4::Ext4_1 vec_1(Cork::Math::Vector3D(3, 4, 5));
                Math::ExteriorCalculusR4::Ext4_1 vec_2(Cork::Math::Vector3D(6, 7, 8));
                Math::ExteriorCalculusR4::Ext4_1 vec_3(Cork::Math::Vector3D(14, -10, 5));

                Math::ExteriorCalculusR4::Ext4_2 e4_2 = vec_1.join(vec_2);

                for (auto i = 0; i < 2000000; i++)
                {
                    Math::ExteriorCalculusR4::Ext4_3 wedge_product = vec_3.join(e4_2);

                    running_sum += wedge_product[0] + wedge_product[1] + wedge_product[2] + wedge_product[3];

                    vec_3.negate();  //  Negate is needed to prevent the compiler from optimizing out the operations
                }
            });
            return running_sum;
        };

        BENCHMARK_ADVANCED("Ext4_2 Intersection with meet()")(Catch::Benchmark::Chronometer meter)
        {
            double running_dot_sum = 0;

            //  Compute the point of intersection.  Meet does some duals and a join.

            meter.measure([&running_dot_sum] {
                Math::ExteriorCalculusR4::Ext4_1 tri_1(Cork::Math::Vector3D(2, 2, 1));
                Math::ExteriorCalculusR4::Ext4_1 tri_2(Cork::Math::Vector3D(7, 4, 2));
                Math::ExteriorCalculusR4::Ext4_1 tri_3(Cork::Math::Vector3D(4, 9, 3));

                Math::ExteriorCalculusR4::Ext4_1 edge_1(Cork::Math::Vector3D(2, 2, 3));
                Math::ExteriorCalculusR4::Ext4_1 edge_2(Cork::Math::Vector3D(6, 7, 0));

                //  Build the Ext4_3 for the triangle

                Math::ExteriorCalculusR4::Ext4_3 t_ext3((tri_1.join(tri_2)).join(tri_3));

                //  Now the Ext4_2 for the edge

                Math::ExteriorCalculusR4::Ext4_2 e_ext2(edge_1.join(edge_2));

                for (auto i = 0; i < 2000000; i++)
                {
                    Cork::Math::Vector3D intersection(e_ext2.meet(t_ext3));

                    running_dot_sum += intersection.x() + intersection.y() + intersection.z();

                    t_ext3.negate();  //  Negate to prevent the optimizer from optmizing away the 'meet()' operation
                    e_ext2.negate();
                }
            });

            //  Return the running sum to prevent the benchmark from being optimized away.

            return running_dot_sum;
        };
    }
}  // namespace Cork