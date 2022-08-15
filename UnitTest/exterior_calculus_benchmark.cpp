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

//  NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers)

namespace Cork
{
    TEST_CASE("Exterior Calculus Benchmarks", "[ext calc perf]")
    {
        BENCHMARK_ADVANCED("Ext4_1 Dot Product")(Catch::Benchmark::Chronometer meter)
        {
            double dot_1_2_sum = 0;

            meter.measure([&dot_1_2_sum] {
                Math::ExteriorCalculusR4::Ext4_1 vec_1(Cork::Primitives::Vector3D(3, 4, 5));
                Math::ExteriorCalculusR4::Ext4_1 vec_2(Cork::Primitives::Vector3D(6, 7, 8));

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
                Math::ExteriorCalculusR4::Ext4_1 vec_1(Cork::Primitives::Vector3D(3, 4, 5));
                Math::ExteriorCalculusR4::Ext4_1 vec_2(Cork::Primitives::Vector3D(6, 7, 8));

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
                Math::ExteriorCalculusR4::Ext4_1 vec_1(Cork::Primitives::Vector3D(3, 4, 5));
                Math::ExteriorCalculusR4::Ext4_1 vec_2(Cork::Primitives::Vector3D(6, 7, 8));
                Math::ExteriorCalculusR4::Ext4_1 vec_3(Cork::Primitives::Vector3D(14, -10, 5));

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
                Math::ExteriorCalculusR4::Ext4_1 tri_1(Cork::Primitives::Vector3D(2, 2, 1));
                Math::ExteriorCalculusR4::Ext4_1 tri_2(Cork::Primitives::Vector3D(7, 4, 2));
                Math::ExteriorCalculusR4::Ext4_1 tri_3(Cork::Primitives::Vector3D(4, 9, 3));

                Math::ExteriorCalculusR4::Ext4_1 edge_1(Cork::Primitives::Vector3D(2, 2, 3));
                Math::ExteriorCalculusR4::Ext4_1 edge_2(Cork::Primitives::Vector3D(6, 7, 0));

                //  Build the Ext4_3 for the triangle

                Math::ExteriorCalculusR4::Ext4_3 t_ext3((tri_1.join(tri_2)).join(tri_3));

                //  Now the Ext4_2 for the edge

                Math::ExteriorCalculusR4::Ext4_2 e_ext2(edge_1.join(edge_2));

                for (auto i = 0; i < 2000000; i++)
                {
                    Cork::Primitives::Vector3D intersection(e_ext2.meet(t_ext3));

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

//  NOLINTEND(cppcoreguidelines-avoid-magic-numbers)
