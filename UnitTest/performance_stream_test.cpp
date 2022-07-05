
#include <catch2/catch_all.hpp>
#include <sstream>

#include "primitives/primitives.hpp"
#include "util/performance_stringstream.hpp"

//  The pragma below is to disable to false errors flagged by intellisense for Catch2 REQUIRE macros.

#if __INTELLISENSE__
#pragma diag_suppress 2486
#endif

inline SEFUtility::PerformanceOStringStream& operator<<(SEFUtility::PerformanceOStringStream& out_stream,
                                                        Cork::Primitives::Vector3D& vec)
{
    fmt::format_to(std::move(out_stream.back_insert_iterator()), FMT_COMPILE("{:g} {:g} {:g}"), vec.x(), vec.y(),
                   vec.z());

    return out_stream;
}

inline SEFUtility::PerformanceOStringStream& operator<<(SEFUtility::PerformanceOStringStream& out_stream,
                                                        const Cork::Primitives::Vector3D& vec)
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
}