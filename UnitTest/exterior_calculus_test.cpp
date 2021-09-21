#include <catch2/catch_all.hpp>

#include "intersection/ext4.h"

//  The pragma below is to disable to false errors flagged by intellisense for Catch2 REQUIRE macros.

#if __INTELLISENSE__
#pragma diag_suppress 2486
#endif

TEST_CASE("Exterior Calculus Tests", "[ext calc basics]")
{
    SECTION("Basic Ext4_1 Functionality")
    {
        //  Create from a vector, assignment and equality

        Cork::Math::Vector3D test_vec(7, 4, 3);

        ExteriorCalculusR4::Ext4_1 test_R4_ext(test_vec);
        ExteriorCalculusR4::Ext4_1 test_R4_ext_copy(test_R4_ext);

        REQUIRE(
            ((test_R4_ext.e0() == 7) && (test_R4_ext.e1() == 4) && (test_R4_ext.e2() == 3) && (test_R4_ext.e3() == 1)));
        REQUIRE(((test_R4_ext[0] == 7) && (test_R4_ext[1] == 4) && (test_R4_ext[2] == 3) && (test_R4_ext[3] == 1)));

        REQUIRE(test_R4_ext == test_R4_ext_copy);
        REQUIRE(test_R4_ext != test_R4_ext_copy.negative());

        //  Negation - destructive and non-destructive.

        test_R4_ext.negate();

        REQUIRE(((test_R4_ext.e0() == -7) && (test_R4_ext.e1() == -4) && (test_R4_ext.e2() == -3) &&
                 (test_R4_ext.e3() == -1)));

        ExteriorCalculusR4::Ext4_1 negated_test_R4_ext(test_R4_ext.negative());

        REQUIRE(((test_R4_ext.e0() == -7) && (test_R4_ext.e1() == -4) && (test_R4_ext.e2() == -3) &&
                 (test_R4_ext.e3() == -1)));
        REQUIRE(((negated_test_R4_ext.e0() == 7) && (negated_test_R4_ext.e1() == 4) &&
                 (negated_test_R4_ext.e2() == 3) && (negated_test_R4_ext.e3() == 1)));

        //  Dual and reverse dual
        //
        //  Dual of an Ext4_1 returns an Ext4_3 (n=4, k=1, n-k = 3).  Reverse Dual on the Ext4_3 returns an Ext4_1 (n=4,
        //  k=3, n-k=1) which is identical to the original vector.  Same for Ext4_1::reverse_dual() and Ext4_3::dual()

        test_R4_ext = test_vec;

        ExteriorCalculusR4::Ext4_3 dual = test_R4_ext.dual();
        ExteriorCalculusR4::Ext4_3 reverse_dual = test_R4_ext.reverse_dual();

        REQUIRE(((dual.e012() == 1) && (dual.e013() == -3) && (dual.e023() == 4) && (dual.e123() == -7)));
        REQUIRE(((reverse_dual.e012() == -1) && (reverse_dual.e013() == 3) && (reverse_dual.e023() == -4) &&
                 (reverse_dual.e123() == 7)));

        ExteriorCalculusR4::Ext4_1 original = dual.reverse_dual();

        REQUIRE(original == test_R4_ext);

        original = reverse_dual.dual();

        REQUIRE(original == test_R4_ext);
    }

    SECTION("Ext4_1 Join (Wedge Product)")
    {
        ExteriorCalculusR4::Ext4_1 vec_1(Cork::Math::Vector3D(3, 4, 5));
        ExteriorCalculusR4::Ext4_1 vec_2(Cork::Math::Vector3D(6, 7, 8));
        ExteriorCalculusR4::Ext4_1 vec_3(Cork::Math::Vector3D(14, -10, 5));

        //  Compute wedge of two Ext4_1 vectors

        ExteriorCalculusR4::Ext4_2 vec_1_wedge_vec_2 = vec_1.join(vec_2);

        REQUIRE(((vec_1_wedge_vec_2.e01() == -3) && (vec_1_wedge_vec_2.e02() == -6) &&
                 (vec_1_wedge_vec_2.e03() == -3) && (vec_1_wedge_vec_2.e12() == -3) &&
                 (vec_1_wedge_vec_2.e13() == -3) && (vec_1_wedge_vec_2.e23() == -3)));

        //  Now wedge of Ext4_1 vector and the Ext4_2 parallelogram.  This results in a
        //      Ext4_3 parallelepiped.

        ExteriorCalculusR4::Ext4_2 parallelogram = vec_1_wedge_vec_2;

        ExteriorCalculusR4::Ext4_3 vec_3_wedge_pgram = vec_3.join(parallelogram);

        REQUIRE(((vec_3_wedge_pgram.e012() == -117) && (vec_3_wedge_pgram.e013() == -75) &&
                 (vec_3_wedge_pgram.e023() == -33) && (vec_3_wedge_pgram.e123() == 42)));
    }

    SECTION("Ext4_1 Inner (Dot Product)")
    {
        ExteriorCalculusR4::Ext4_1 vec_1(Cork::Math::Vector3D(3, 4, 5));
        ExteriorCalculusR4::Ext4_1 vec_2(Cork::Math::Vector3D(6, 7, 8));

        double dot_1_2 = vec_1.inner(vec_2);

        REQUIRE(dot_1_2 == 87);
    }

    SECTION("Basic Ext4_2 Functionality")
    {
        //  Create from a vector, assignment and equality

        ExteriorCalculusR4::Ext4_1 vec_1(Cork::Math::Vector3D(7, 4, 3));
        ExteriorCalculusR4::Ext4_1 vec_2(Cork::Math::Vector3D(11, 17, 13));

        ExteriorCalculusR4::Ext4_2 pgram = vec_1.join(vec_2);

        REQUIRE(((pgram.e01() == 75) && (pgram.e02() == 58) && (pgram.e03() == -4) && (pgram.e12() == 1) &&
                 (pgram.e13() == -13) && (pgram.e23() == -10)));
        REQUIRE(((pgram[0] == 75) && (pgram[1] == 58) && (pgram[2] == -4) && (pgram[3] == 1) && (pgram[4] == -13) &&
                 (pgram[5] == -10)));

        //  Negation - leave the pgram back in its initial values

        ExteriorCalculusR4::Ext4_2 pgram_negative = pgram.negative();

        REQUIRE(((pgram_negative.e01() == -75) && (pgram_negative.e02() == -58) && (pgram_negative.e03() == 4) &&
                 (pgram_negative.e12() == -1) && (pgram_negative.e13() == 13) && (pgram_negative.e23() == 10)));

        REQUIRE(pgram_negative != pgram);

        pgram.negate();

        REQUIRE(pgram_negative == pgram);

        pgram.negate();

        REQUIRE(((pgram.e01() == 75) && (pgram.e02() == 58) && (pgram.e03() == -4) && (pgram.e12() == 1) &&
                 (pgram.e13() == -13) && (pgram.e23() == -10)));

        //  Dual and Reverse Dual
        //
        //  Reverse dual of the dual is the original R4 2

        ExteriorCalculusR4::Ext4_2 dual = pgram.dual();

        REQUIRE(((dual.e01() == -10) && (dual.e02() == 13) && (dual.e03() == 1) && (dual.e12() == -4) &&
                 (dual.e13() == -58) && (dual.e23() == 75)));

        ExteriorCalculusR4::Ext4_2 original = dual.reverse_dual();

        REQUIRE(((original.e01() == 75) && (original.e02() == 58) && (original.e03() == -4) && (original.e12() == 1) &&
                 (original.e13() == -13) && (original.e23() == -10)));
        REQUIRE(original == pgram);
    }

    SECTION("Ext4_2 Join (Wedge Product)")
    {
        //  Create from a pair of vectors and check values

        ExteriorCalculusR4::Ext4_1 vec_1(Cork::Math::Vector3D(7, 4, 3));
        ExteriorCalculusR4::Ext4_1 vec_2(Cork::Math::Vector3D(11, 17, 13));

        ExteriorCalculusR4::Ext4_2 pgram = vec_1.join(vec_2);

        REQUIRE(((pgram.e01() == 75) && (pgram.e02() == 58) && (pgram.e03() == -4) && (pgram.e12() == 1) &&
                 (pgram.e13() == -13) && (pgram.e23() == -10)));
        REQUIRE(((pgram[0] == 75) && (pgram[1] == 58) && (pgram[2] == -4) && (pgram[3] == 1) && (pgram[4] == -13) &&
                 (pgram[5] == -10)));

        //  Compute the wedge product of the Parallelogram and the vector leaving an Ext4_4 parallelepiped

        ExteriorCalculusR4::Ext4_1 vec_3(Cork::Math::Vector3D(14, -10, 5));

        ExteriorCalculusR4::Ext4_3 ppiped = pgram.join(vec_3);

        REQUIRE(
            ((ppiped.e012() == 969) && (ppiped.e013() == -147) && (ppiped.e023() == -62) && (ppiped.e123() == 166)));
    }

    SECTION("Ext4_2 Inner (Dot Product)")
    {
        //  First Ext4_2

        ExteriorCalculusR4::Ext4_1 vec_1(Cork::Math::Vector3D(7, 4, 3));
        ExteriorCalculusR4::Ext4_1 vec_2(Cork::Math::Vector3D(11, 17, 13));

        ExteriorCalculusR4::Ext4_2 pgram_1 = vec_1.join(vec_2);

        REQUIRE(((pgram_1.e01() == 75) && (pgram_1.e02() == 58) && (pgram_1.e03() == -4) && (pgram_1.e12() == 1) &&
                 (pgram_1.e13() == -13) && (pgram_1.e23() == -10)));
        REQUIRE(((pgram_1[0] == 75) && (pgram_1[1] == 58) && (pgram_1[2] == -4) && (pgram_1[3] == 1) &&
                 (pgram_1[4] == -13) && (pgram_1[5] == -10)));

        //  Second Ext4_2

        ExteriorCalculusR4::Ext4_1 vec_3(Cork::Math::Vector3D(-5, 6, 9));
        ExteriorCalculusR4::Ext4_1 vec_4(Cork::Math::Vector3D(13, -17, 4));

        ExteriorCalculusR4::Ext4_2 pgram_2 = vec_3.join(vec_4);

        REQUIRE(((pgram_2.e01() == 7) && (pgram_2.e02() == -137) && (pgram_2.e03() == -18) && (pgram_2.e12() == 177) &&
                 (pgram_2.e13() == 23) && (pgram_2.e23() == 5)));

        //  Inner product

        double dot_product = pgram_1.inner(pgram_2);

        REQUIRE(dot_product == -7521);
    }

    SECTION("Triangle Edge Intersection with Ext4_2 Meet")
    {
        ExteriorCalculusR4::Ext4_1 tri_1(Cork::Math::Vector3D(2, 2, 1));
        ExteriorCalculusR4::Ext4_1 tri_2(Cork::Math::Vector3D(7, 4, 2));
        ExteriorCalculusR4::Ext4_1 tri_3(Cork::Math::Vector3D(4, 9, 3));

        ExteriorCalculusR4::Ext4_1 edge_1(Cork::Math::Vector3D(2, 2, 3));
        ExteriorCalculusR4::Ext4_1 edge_2(Cork::Math::Vector3D(6, 7, 0));

        //  Build the Ext4_3 for the triangle

        ExteriorCalculusR4::Ext4_3 t_ext3((tri_1.join(tri_2)).join(tri_3));

        //  Now the Ext4_2 for the edge

        ExteriorCalculusR4::Ext4_2 e_ext2(edge_1.join(edge_2));

        //  Compute the point of intersection.  Meet does some duals and a join.

        ExteriorCalculusR4::Ext4_1 intersection(e_ext2.meet(t_ext3));

        //  Convert back into a point, which is mostly just normalizing by e3.
        //      Negative e3 values will be resolved by the division.

        Cork::Math::Vector3D result(intersection);

        REQUIRE(((result.x() == Catch::Approx(538.0 / 145.0).epsilon(0.00001)) &&
                 (result.y() == Catch::Approx(120.0 / 29.0).epsilon(0.00001)) &&
                 (result.z() == Catch::Approx(249.0 / 145.0).epsilon(0.00001))));
    }
    
    SECTION("Ext4_3 Basic Tests")
    {
        ExteriorCalculusR4::Ext4_1 tri_1(Cork::Math::Vector3D(2, 2, 1));
        ExteriorCalculusR4::Ext4_1 tri_2(Cork::Math::Vector3D(7, 4, 2));
        ExteriorCalculusR4::Ext4_1 tri_3(Cork::Math::Vector3D(4, 9, 3));

        //  Build the Ext4_3 for the triangle

        ExteriorCalculusR4::Ext4_3 t_ext3((tri_1.join(tri_2)).join(tri_3));

        REQUIRE((( t_ext3.e012() == 9 ) && ( t_ext3.e013() == 31 ) && ( t_ext3.e023() == 8 ) && ( t_ext3.e123() == -3 ) ));
        REQUIRE((( t_ext3[0] == 9 ) && ( t_ext3[1] == 31 ) && ( t_ext3[2] == 8 ) && ( t_ext3[3] == -3 ) ));

        //  Negation

        ExteriorCalculusR4::Ext4_3  t_ext3_negative( t_ext3.negative() );

        REQUIRE((( t_ext3_negative.e012() == -9 ) && ( t_ext3_negative.e013() == -31 ) && ( t_ext3_negative.e023() == -8 ) && ( t_ext3_negative.e123() == 3 ) ));

        t_ext3.negate();

        REQUIRE( t_ext3 == t_ext3_negative );

        t_ext3.negate();

        REQUIRE( t_ext3 != t_ext3_negative );
        REQUIRE((( t_ext3.e012() == 9 ) && ( t_ext3.e013() == 31 ) && ( t_ext3.e023() == 8 ) && ( t_ext3.e123() == -3 ) ));

        //  Dual and Reverse Dual

        ExteriorCalculusR4::Ext4_1  t_ext3_dual( t_ext3.dual() );

        REQUIRE((( t_ext3_dual.e0() == -3 ) && ( t_ext3_dual.e1() == -8 ) && ( t_ext3_dual.e2() == 31 ) && ( t_ext3_dual.e3() == -9 ) ));

        ExteriorCalculusR4::Ext4_1  t_ext3_reverse_dual( t_ext3.reverse_dual() );

        REQUIRE((( t_ext3_reverse_dual.e0() == 3 ) && ( t_ext3_reverse_dual.e1() == 8 ) && ( t_ext3_reverse_dual.e2() == -31 ) && ( t_ext3_reverse_dual.e3() == 9 ) ));
    }

    SECTION("Triangle Triangle Triangle Intersection with Ext4_3 Meet")
    {
        ExteriorCalculusR4::Ext4_1 tri1_1(Cork::Math::Vector3D(1, 3, 1));
        ExteriorCalculusR4::Ext4_1 tri1_2(Cork::Math::Vector3D(4, 7, 4));
        ExteriorCalculusR4::Ext4_1 tri1_3(Cork::Math::Vector3D(7, 9, 3));

        ExteriorCalculusR4::Ext4_1 tri2_1(Cork::Math::Vector3D(2, 2, 3));
        ExteriorCalculusR4::Ext4_1 tri2_2(Cork::Math::Vector3D(6, 7, 3));
        ExteriorCalculusR4::Ext4_1 tri2_3(Cork::Math::Vector3D(4, 8, 4));

        ExteriorCalculusR4::Ext4_1 tri3_1(Cork::Math::Vector3D(2, 2, 4));
        ExteriorCalculusR4::Ext4_1 tri3_2(Cork::Math::Vector3D(6, 7, 1));
        ExteriorCalculusR4::Ext4_1 tri3_3(Cork::Math::Vector3D(4, 8, 5));

        //  Build the Ext4_3 for the triangle 1

        ExteriorCalculusR4::Ext4_3 t1_ext3((tri1_1.join(tri1_2)).join(tri1_3));

        //  Build the Ext4_3 for the triangle 2

        ExteriorCalculusR4::Ext4_3 t2_ext3((tri2_1.join(tri2_2)).join(tri2_3));

        //  Build the Ext4_3 for the triangle 3

        ExteriorCalculusR4::Ext4_3 t3_ext3((tri3_1.join(tri3_2)).join(tri3_3));

        //  Compute the point of intersection.  First we get an edge on the first two triangles and then
        //      we find the intersection of that edge with the third triangle.

        ExteriorCalculusR4::Ext4_2 intersect_edge_1_2(t1_ext3.meet(t2_ext3));
        ExteriorCalculusR4::Ext4_1 intersection(intersect_edge_1_2.meet(t3_ext3));

        //  Convert back into a point, which is mostly just normalizing by e3.
        //      Negative e3 values will be resolved by the division.

        Cork::Math::Vector3D result(intersection);

        REQUIRE(((result.x() == Catch::Approx(104.0/23.0).epsilon(0.00001)) &&
                 (result.y() == Catch::Approx(499.0/69.0).epsilon(0.00001)) &&
                 (result.z() == Catch::Approx(248.0/69.0).epsilon(0.00001))));
    }
}

TEST_CASE("Exterior Calculus Benchmarks", "[ext calc perf]")
{
    BENCHMARK_ADVANCED("Ext4_1 Dot Product")(Catch::Benchmark::Chronometer meter)
    {
        double dot_1_2_sum = 0;

        meter.measure([&dot_1_2_sum] {
            ExteriorCalculusR4::Ext4_1 vec_1(Cork::Math::Vector3D(3, 4, 5));
            ExteriorCalculusR4::Ext4_1 vec_2(Cork::Math::Vector3D(6, 7, 8));

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
            ExteriorCalculusR4::Ext4_1 vec_1(Cork::Math::Vector3D(3, 4, 5));
            ExteriorCalculusR4::Ext4_1 vec_2(Cork::Math::Vector3D(6, 7, 8));

            for (auto i = 0; i < 2000000; i++)
            {
                ExteriorCalculusR4::Ext4_2 wedge_product = vec_1.join(vec_2);

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
            ExteriorCalculusR4::Ext4_1 vec_1(Cork::Math::Vector3D(3, 4, 5));
            ExteriorCalculusR4::Ext4_1 vec_2(Cork::Math::Vector3D(6, 7, 8));
            ExteriorCalculusR4::Ext4_1 vec_3(Cork::Math::Vector3D(14, -10, 5));

            ExteriorCalculusR4::Ext4_2 e4_2 = vec_1.join(vec_2);

            for (auto i = 0; i < 2000000; i++)
            {
                ExteriorCalculusR4::Ext4_3 wedge_product = vec_3.join(e4_2);

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
            ExteriorCalculusR4::Ext4_1 tri_1(Cork::Math::Vector3D(2, 2, 1));
            ExteriorCalculusR4::Ext4_1 tri_2(Cork::Math::Vector3D(7, 4, 2));
            ExteriorCalculusR4::Ext4_1 tri_3(Cork::Math::Vector3D(4, 9, 3));

            ExteriorCalculusR4::Ext4_1 edge_1(Cork::Math::Vector3D(2, 2, 3));
            ExteriorCalculusR4::Ext4_1 edge_2(Cork::Math::Vector3D(6, 7, 0));

            //  Build the Ext4_3 for the triangle

            ExteriorCalculusR4::Ext4_3 t_ext3((tri_1.join(tri_2)).join(tri_3));

            //  Now the Ext4_2 for the edge

            ExteriorCalculusR4::Ext4_2 e_ext2(edge_1.join(edge_2));

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
