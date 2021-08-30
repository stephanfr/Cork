#include "util/performance_stringstream.h"

#include <math/Vector3DTemplate.h>

#include <catch2/catch_all.hpp>
#include <sstream>

//  The pragma below is to disable to false errors flagged by intellisense for Catch2 REQUIRE macros.

#if __INTELLISENSE__
#pragma diag_suppress 2486
#endif

typedef Cork::Math::Vector3DTemplate<double> Vector3D;



TEST_CASE("GrowableStreambuf Tests", "[util basics]")
{
    SECTION("Basic Functionality")
    {
        SEFUtility::PerformanceOStringStream test_stream(100);

        test_stream << "test";

        REQUIRE(test_stream.str().compare("test") == 0);
        REQUIRE(test_stream.str().capacity() == 100);

        for (int i = 0; i < 100; i++)
        {
            test_stream << "test";

            if (test_stream.str().size() <= 100)
            {
                REQUIRE(test_stream.str().capacity() == 100);
            }
            else
            {
                REQUIRE(test_stream.str().capacity() > 100);
            }
        }
    }

    BENCHMARK("PerformanceOStringStream Insert 10k string literals")
    {
        SEFUtility::PerformanceOStringStream test_stream(100000);

        for (int i = 0; i < 10000; i++)
        {
            test_stream << "12345";
        }
    };

    BENCHMARK("std::stringstream Insert 10k string literals")
    {
        std::stringstream test_stream;

        for (int i = 0; i < 10000; i++)
        {
            test_stream << "12345";
        }
    };

    BENCHMARK("PerformanceOStringStream Insert 10k char* literals")
    {
        SEFUtility::PerformanceOStringStream test_stream(100000);

        for (int i = 0; i < 10000; i++)
        {
            test_stream << (const char*)"12345";
        }
    };

    BENCHMARK("std::stringstream Insert 10k char* literals")
    {
        std::stringstream test_stream;

        for (int i = 0; i < 10000; i++)
        {
            test_stream << (const char*)"12345";
        }
    };

    BENCHMARK("PerformanceOStringStream Insert 10k std::strings")
    {
        SEFUtility::PerformanceOStringStream test_stream(100000);

        std::string test = "12345";

        for (int i = 0; i < 10000; i++)
        {
            test_stream << test;
        }
    };

    BENCHMARK("std::stringstream Insert 10k std::strings")
    {
        std::stringstream test_stream;

        std::string test = "12345";

        for (int i = 0; i < 10000; i++)
        {
            test_stream << test;
        }
    };

    BENCHMARK("PerformanceOStringStream Insert 10k int32s")
    {
        SEFUtility::PerformanceOStringStream test_stream(100000);

        int32_t test = 123456;

        for (int i = 0; i < 10000; i++)
        {
            test_stream << test;
        }
    };

    BENCHMARK("std::stringstream Insert 10k int32s")
    {
        std::stringstream test_stream;

        int32_t test = 123456;

        for (int i = 0; i < 10000; i++)
        {
            test_stream << test;
        }
    };

    BENCHMARK("PerformanceOStringStream Insert 10k uint32s")
    {
        SEFUtility::PerformanceOStringStream test_stream(100000);

        uint32_t test = 123456;

        for (int i = 0; i < 10000; i++)
        {
            test_stream << test;
        }
    };

    BENCHMARK("std::stringstream Insert 10k uint32s")
    {
        std::stringstream test_stream;

        uint32_t test = 123456;

        for (int i = 0; i < 10000; i++)
        {
            test_stream << test;
        }
    };

    BENCHMARK("PerformanceOStringStream Insert 10k doubles")
    {
        SEFUtility::PerformanceOStringStream test_stream(100000);

        double test = 123456.789;

        for (int i = 0; i < 10000; i++)
        {
            test_stream << test;
        }
    };

    BENCHMARK("std::stringstream Insert 10k double")
    {
        std::stringstream test_stream;

        double test = 123456.789;

        for (int i = 0; i < 10000; i++)
        {
            test_stream << test;
        }
    };

    BENCHMARK("PerformanceOStringStream Insert 10k Vector3Ds")
    {
        SEFUtility::PerformanceOStringStream test_stream(100000);

        Vector3D test{1.1, 2.2, 3.3};

        for (int i = 0; i < 10000; i++)
        {
            test_stream << test;
        }
    };

    BENCHMARK("std::stringstream Insert 10k Vector3Ds")
    {
        std::stringstream test_stream;

        Vector3D test{1.1, 2.2, 3.3};

        for (int i = 0; i < 10000; i++)
        {
            test_stream << test;
        }
    };
}