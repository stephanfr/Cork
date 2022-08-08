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

#include <cstdlib>

#include <catch2/catch_all.hpp>

//  NOLINTBEGIN(bugprone-reserved-identifier, modernize-loop-convert, cppcoreguidelines-avoid-magic-numbers)

#define NUMERIC_PRECISION double
#define __AVX_AVAILABLE__

#include "math/bounding_box_3D_template.hpp"

using BBox3D = Cork::Math::BBox3DTemplate<NUMERIC_PRECISION, SIMDInstructionSet::NONE>;
using BBox3DAVX = Cork::Math::BBox3DTemplate<NUMERIC_PRECISION, SIMDInstructionSet::AVX2>;

using Vector3D = Cork::Math::Vector3DTemplate<NUMERIC_PRECISION, SIMDInstructionSet::NONE>;
using Vector3DAVX = Cork::Math::Vector3DTemplate<NUMERIC_PRECISION, SIMDInstructionSet::AVX2>;


constexpr int NUM_ITERATIONS = 50000;

bool RandomBoundingBoxGenerator(std::array<BBox3D, NUM_ITERATIONS>& bounding_boxes)
{
    for (size_t i = 0; i < bounding_boxes.size(); i++)
    {
        auto first_point = Vector3D::randomVector(0, 100);
        auto offset_to_second_point = Vector3D::randomVector(0, 100);

        bounding_boxes[i] = BBox3D(first_point, first_point + offset_to_second_point);
    }

    return true;
}

TEST_CASE("BBox3D Benchmarks - 50k Bounding Boxes", "[primitives]")
{
    std::array<BBox3D, NUM_ITERATIONS> bounding_boxes_non_AVX;
    std::array<BBox3DAVX, NUM_ITERATIONS> bounding_boxes_AVX;

    RandomBoundingBoxGenerator(bounding_boxes_non_AVX);

    for (uint32_t i = 0; i < NUM_ITERATIONS; i++)
    {
        bounding_boxes_AVX[i] =
            BBox3DAVX(Vector3DAVX(bounding_boxes_non_AVX[i].minima().x(), bounding_boxes_non_AVX[i].minima().y(),
                                  bounding_boxes_non_AVX[i].minima().z()),
                      Vector3DAVX(bounding_boxes_non_AVX[i].maxima().x(), bounding_boxes_non_AVX[i].maxima().y(),
                                  bounding_boxes_non_AVX[i].maxima().z()));
    }

    BENCHMARK_ADVANCED("Center Non-AVX")(Catch::Benchmark::Chronometer meter)
    {
        double running_sum_to_prevent_dead_code_elimination = 0;

        meter.measure([&bounding_boxes_non_AVX, &running_sum_to_prevent_dead_code_elimination] {
            for (size_t i = 0; i < NUM_ITERATIONS; i++)
            {
                running_sum_to_prevent_dead_code_elimination += bounding_boxes_non_AVX[i].center().x();
            }
        });

        return running_sum_to_prevent_dead_code_elimination;
    };

    BENCHMARK_ADVANCED("Center AVX")(Catch::Benchmark::Chronometer meter)
    {
        double running_sum_to_prevent_dead_code_elimination = 0;

        meter.measure([&bounding_boxes_AVX, &running_sum_to_prevent_dead_code_elimination] {
            for (size_t i = 0; i < NUM_ITERATIONS; i++)
            {
                running_sum_to_prevent_dead_code_elimination += bounding_boxes_AVX[i].center().x();
            }
        });

        return running_sum_to_prevent_dead_code_elimination;
    };

    BENCHMARK_ADVANCED("Intersects Non-AVX")(Catch::Benchmark::Chronometer meter)
    {
        int count = 0;

        meter.measure([&bounding_boxes_non_AVX, &count] {
            for (size_t i = 1; i < NUM_ITERATIONS; i++)
            {
                if(bounding_boxes_non_AVX[i-1].intersects( bounding_boxes_non_AVX[i] ))
                {
                    count++;
                }
            }
        });

        return count;
    };

    BENCHMARK_ADVANCED("Intersects AVX")(Catch::Benchmark::Chronometer meter)
    {
        int count = 0;

        meter.measure([&bounding_boxes_AVX, &count] {
            for (size_t i = 1; i < NUM_ITERATIONS; i++)
            {
                if(bounding_boxes_AVX[i-1].intersects( bounding_boxes_AVX[i] ))
                {
                    count++;
                }
            }
        });

        return count;
    };

    BENCHMARK_ADVANCED("Does not intersect Non-AVX")(Catch::Benchmark::Chronometer meter)
    {
        int count = 0;

        meter.measure([&bounding_boxes_non_AVX, &count] {
            for (size_t i = 1; i < NUM_ITERATIONS; i++)
            {
                if(bounding_boxes_non_AVX[i-1].doesNotIntersect( bounding_boxes_non_AVX[i] ))
                {
                    count++;
                }
            }
        });

        return count;
    };

    BENCHMARK_ADVANCED("Does not intersect AVX")(Catch::Benchmark::Chronometer meter)
    {
        int count = 0;

        meter.measure([&bounding_boxes_AVX, &count] {
            for (size_t i = 1; i < NUM_ITERATIONS; i++)
            {
                if(bounding_boxes_AVX[i-1].doesNotIntersect( bounding_boxes_AVX[i] ))
                {
                    count++;
                }
            }
        });

        return count;
    };

    
    BENCHMARK_ADVANCED("Convex Non-AVX")(Catch::Benchmark::Chronometer meter)
    {
        BBox3D      convex_bbox;

        meter.measure([&bounding_boxes_non_AVX, &convex_bbox] {
            for (size_t i = 0; i < NUM_ITERATIONS; i++)
            {
                convex_bbox.convex( bounding_boxes_non_AVX[i]);

            }
        });

        return convex_bbox;
    };

    BENCHMARK_ADVANCED("Convex AVX")(Catch::Benchmark::Chronometer meter)
    {
        BBox3DAVX      convex_bbox;

        meter.measure([&bounding_boxes_AVX, &convex_bbox] {
            for (size_t i = 0; i < NUM_ITERATIONS; i++)
            {
                convex_bbox.convex( bounding_boxes_AVX[i]);
            }
        });

        return convex_bbox;
    };
}

//  NOLINTEND(bugprone-reserved-identifier, modernize-loop-convert, cppcoreguidelines-avoid-magic-numbers)

