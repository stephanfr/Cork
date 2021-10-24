
#include <catch2/catch_all.hpp>
#include <sstream>

#include "primitives/primitives.hpp"

#include "util/performance_stringstream.h"

//  The pragma below is to disable to false errors flagged by intellisense for Catch2 REQUIRE macros.

#if __INTELLISENSE__
#pragma diag_suppress 2486
#endif



inline SEFUtility::PerformanceOStringStream& operator<<(SEFUtility::PerformanceOStringStream& out_stream,
                                                        Cork::Math::Vector3D& vec)
{
    fmt::format_to(std::move(out_stream.back_insert_iterator()), FMT_COMPILE("{:g} {:g} {:g}"), vec.x(), vec.y(),
                   vec.z());

    return out_stream;
}

inline SEFUtility::PerformanceOStringStream& operator<<(SEFUtility::PerformanceOStringStream& out_stream,
                                                        const Cork::Math::Vector3D& vec)
{
    fmt::format_to(std::move(out_stream.back_insert_iterator()), FMT_COMPILE("{:g} {:g} {:g}"), vec.x(), vec.y(),
                   vec.z());

    return out_stream;
}


constexpr int SMALL_BUFFER_SIZE = 100;
constexpr int NUM_GROW_BUFFER_ITERATIONS = 101;

constexpr int NUM_BENCHMARK_ITERATIONS = 10000;
constexpr int STREAM_BUFFER_PREALLOC_SIZE = 100000;


TEST_CASE("GrowableStreambuf Tests", "[util basics]")
{
    SECTION("Basic Functionality")
    {
        SEFUtility::PerformanceOStringStream test_stream(SMALL_BUFFER_SIZE);

        test_stream << "test";

        REQUIRE(test_stream.str().compare("test") == 0);
        REQUIRE(test_stream.str().capacity() == SMALL_BUFFER_SIZE);

        for (int i = 0; i < NUM_GROW_BUFFER_ITERATIONS; i++)
        {
            test_stream << "test";

            if (test_stream.str().size() <= SMALL_BUFFER_SIZE)
            {
                REQUIRE(test_stream.str().capacity() == SMALL_BUFFER_SIZE);
            }
            else
            {
                REQUIRE(test_stream.str().capacity() > SMALL_BUFFER_SIZE);
            }
        }
    }

    BENCHMARK("PerformanceOStringStream Insert 10k string literals")
    {
        SEFUtility::PerformanceOStringStream test_stream(STREAM_BUFFER_PREALLOC_SIZE);

        for (int i = 0; i < NUM_BENCHMARK_ITERATIONS; i++)
        {
            test_stream << "12345";
        }
    };

    BENCHMARK("std::stringstream Insert 10k string literals")
    {
        std::stringstream test_stream;

        for (int i = 0; i < NUM_BENCHMARK_ITERATIONS; i++)
        {
            test_stream << "12345";
        }
    };

    BENCHMARK("PerformanceOStringStream Insert 10k char* literals")
    {
        SEFUtility::PerformanceOStringStream test_stream(STREAM_BUFFER_PREALLOC_SIZE);

        for (int i = 0; i < NUM_BENCHMARK_ITERATIONS; i++)
        {
            test_stream << (const char*)"12345";
        }
    };

    BENCHMARK("std::stringstream Insert 10k char* literals")
    {
        std::stringstream test_stream;

        for (int i = 0; i < NUM_BENCHMARK_ITERATIONS; i++)
        {
            test_stream << (const char*)"12345";
        }
    };

    BENCHMARK("PerformanceOStringStream Insert 10k std::strings")
    {
        SEFUtility::PerformanceOStringStream test_stream(STREAM_BUFFER_PREALLOC_SIZE);

        std::string test = "12345";

        for (int i = 0; i < NUM_BENCHMARK_ITERATIONS; i++)
        {
            test_stream << test;
        }
    };

    BENCHMARK("std::stringstream Insert 10k std::strings")
    {
        std::stringstream test_stream;

        std::string test = "12345";

        for (int i = 0; i < NUM_BENCHMARK_ITERATIONS; i++)
        {
            test_stream << test;
        }
    };

    BENCHMARK("PerformanceOStringStream Insert 10k int32s")
    {
        SEFUtility::PerformanceOStringStream test_stream(STREAM_BUFFER_PREALLOC_SIZE);

        int32_t test = 123456;                                      //  NOLINT(cppcoreguidelines-avoid-magic-numbers)

        for (int i = 0; i < NUM_BENCHMARK_ITERATIONS; i++)
        {
            test_stream << test;
        }
    };

    BENCHMARK("std::stringstream Insert 10k int32s")
    {
        std::stringstream test_stream;

        int32_t test = 123456;                                      //  NOLINT(cppcoreguidelines-avoid-magic-numbers)

        for (int i = 0; i < NUM_BENCHMARK_ITERATIONS; i++)
        {
            test_stream << test;
        }
    };

    BENCHMARK("PerformanceOStringStream Insert 10k uint32s")
    {
        SEFUtility::PerformanceOStringStream test_stream(STREAM_BUFFER_PREALLOC_SIZE);

        uint32_t test = 123456;                                      //  NOLINT(cppcoreguidelines-avoid-magic-numbers)

        for (int i = 0; i < NUM_BENCHMARK_ITERATIONS; i++)
        {
            test_stream << test;
        }
    };

    BENCHMARK("std::stringstream Insert 10k uint32s")
    {
        std::stringstream test_stream;

        uint32_t test = 123456;                                      //  NOLINT(cppcoreguidelines-avoid-magic-numbers)

        for (int i = 0; i < NUM_BENCHMARK_ITERATIONS; i++)
        {
            test_stream << test;
        }
    };

    BENCHMARK("PerformanceOStringStream Insert 10k doubles")
    {
        SEFUtility::PerformanceOStringStream test_stream(STREAM_BUFFER_PREALLOC_SIZE);

        double test = 123456.789;                                   //  NOLINT(cppcoreguidelines-avoid-magic-numbers)

        for (int i = 0; i < NUM_BENCHMARK_ITERATIONS; i++)
        {
            test_stream << test;
        }
    };

    BENCHMARK("std::stringstream Insert 10k double")
    {
        std::stringstream test_stream;

        double test = 123456.789;                                   //  NOLINT(cppcoreguidelines-avoid-magic-numbers)

        for (int i = 0; i < NUM_BENCHMARK_ITERATIONS; i++)
        {
            test_stream << test;
        }
    };

    BENCHMARK("PerformanceOStringStream Insert 10k Vector3Ds")
    {
        SEFUtility::PerformanceOStringStream test_stream(STREAM_BUFFER_PREALLOC_SIZE);

        Cork::Math::Vector3D test{1.1, 2.2, 3.3};                   //  NOLINT(cppcoreguidelines-avoid-magic-numbers)

        for (int i = 0; i < NUM_BENCHMARK_ITERATIONS; i++)
        {
            test_stream << test;
        }
    };

    BENCHMARK("std::stringstream Insert 10k Vector3Ds")
    {
        std::stringstream test_stream;

        Cork::Math::Vector3D test{1.1, 2.2, 3.3};                   //  NOLINT(cppcoreguidelines-avoid-magic-numbers)

        for (int i = 0; i < NUM_BENCHMARK_ITERATIONS; i++)
        {
            test_stream << test;
        }
    };
}