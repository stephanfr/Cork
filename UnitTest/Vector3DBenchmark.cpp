#include <catch2/catch_all.hpp>
#include <stdlib.h>

#include "../CorkLib/CorkDefs.h"

#include "../CorkLib/Util/Xoroshiro128Plus.h"

#define NUMERIC_PRECISION double

#include "../CorkLib/Math/Vector3DTemplate.h"

typedef Cork::Math::Vector3DTemplate<NUMERIC_PRECISION,SIMDInstructionSet::NONE>     Vector3D;
typedef Cork::Math::Vector3DTemplate<NUMERIC_PRECISION,SIMDInstructionSet::AVX>      Vector3DAVX;

constexpr int       NUM_ITERATIONS = 100000;

template <class T>
bool RandomVectorPerformance( std::array<T,NUM_ITERATIONS>&     vectors )
{
    for( auto i = 0; i < vectors.size(); i++ )
    {
        vectors[i] = T::randomVector( 0, 100 );
    }

    return true;
}

template <class T>
bool StandaloneRandomTest(std::array<T,NUM_ITERATIONS>&     vectors)
{
    Xoroshiro128Plus        rng(1);

    for( int i = 0; i < vectors.size(); i++ )
    {
        vectors[i] = T( (NUMERIC_PRECISION)rng.next(), (NUMERIC_PRECISION)rng.next(), (NUMERIC_PRECISION)rng.next() );
//        vectors[i] = T( (NUMERIC_PRECISION)rand(), (NUMERIC_PRECISION)rand(), (NUMERIC_PRECISION)rand() );
    }

    return true;
}


TEST_CASE("Vector3D Benchmarks", "[primitives]")
{

    BENCHMARK_ADVANCED("Random Vector")(Catch::Benchmark::Chronometer meter)
    {
        std::array<Vector3D,NUM_ITERATIONS>    vectors;

        meter.measure([&vectors] { RandomVectorPerformance<Vector3D>(vectors); } );
    };

    BENCHMARK_ADVANCED("Random Vector AVX")(Catch::Benchmark::Chronometer meter)
    {
        std::array<Vector3DAVX,NUM_ITERATIONS>    vectors;

        meter.measure([&vectors] { RandomVectorPerformance<Vector3DAVX>(vectors); } );
    };

    BENCHMARK_ADVANCED("Random Vector Standalone")(Catch::Benchmark::Chronometer meter)
    {
        std::array<Vector3D,NUM_ITERATIONS>    vectors;

        meter.measure([&vectors] { StandaloneRandomTest<Vector3D>(vectors); } );
    };
}
