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

#define NUMERIC_PRECISION double

#include "math/vector_3D_template.hpp"

using Vector3D = Cork::Math::Vector3DTemplate<NUMERIC_PRECISION, SIMDInstructionSet::NONE>;
using Vector3DAVX = Cork::Math::Vector3DTemplate<NUMERIC_PRECISION, SIMDInstructionSet::AVX2>;

constexpr int NUM_ITERATIONS = 250000;

template <class T>
bool RandomVectorPerformance(std::array<T, NUM_ITERATIONS>& vectors)
{
    for (auto i = 0; i < vectors.size(); i++)
    {
        vectors[i] = T::randomVector(0, 100);
    }

    return true;
}


TEST_CASE("Vector3D Benchmarks - 250k Vectors", "[primitives]")
{
    BENCHMARK_ADVANCED("Random Vector")(Catch::Benchmark::Chronometer meter)
    {
        std::array<Vector3D, NUM_ITERATIONS> vectors;

        meter.measure([&vectors] { RandomVectorPerformance<Vector3D>(vectors); });
    };

    BENCHMARK_ADVANCED("Random Vector AVX")(Catch::Benchmark::Chronometer meter)
    {
        std::array<Vector3DAVX, NUM_ITERATIONS> vectors;

        meter.measure([&vectors] { RandomVectorPerformance<Vector3DAVX>(vectors); });
    };
}
