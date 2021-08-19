#include <stdlib.h>

#include <catch2/catch_all.hpp>

#define NUMERIC_PRECISION double

#include "math/Vector3DTemplate.h"

typedef Cork::Math::Vector3DTemplate<NUMERIC_PRECISION, SIMDInstructionSet::NONE> Vector3D;
typedef Cork::Math::Vector3DTemplate<NUMERIC_PRECISION, SIMDInstructionSet::AVX2> Vector3DAVX;

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
