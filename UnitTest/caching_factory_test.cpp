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

#include <array>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <thread>

#include "util/caching_factory.hpp"
#include "util/resettable.hpp"

//  The pragma below is to disable to false errors flagged by intellisense for Catch2 REQUIRE macros.

#if __INTELLISENSE__
#pragma diag_suppress 2486
#endif

//  NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers)

using namespace std::literals::chrono_literals;

class NonResettableObject
{
   public:
    NonResettableObject() = default;

    NonResettableObject(const NonResettableObject&) = delete;
    NonResettableObject(NonResettableObject&&) = delete;

    ~NonResettableObject() = default;

    NonResettableObject& operator=(const NonResettableObject&) = delete;
    NonResettableObject& operator=(NonResettableObject&&) = delete;

    [[nodiscard]] long getValue() const { return (m_value); }

    void setValue(long value) { m_value = value; }

   private:
    long m_value{0};
};

class ResettableObject : SEFUtility::Resettable
{
   public:
    ResettableObject() = default;

    ResettableObject(const ResettableObject&) = delete;
    ResettableObject(ResettableObject&&) = delete;

    virtual ~ResettableObject() = default;

    ResettableObject& operator=(const ResettableObject&) = delete;
    ResettableObject& operator=(ResettableObject&&) = delete;

    void reset() override { m_value = 0; }

    [[nodiscard]] long getValue() const { return (m_value); }

    void setValue(long value) { m_value = value; }

   private:
    long m_value{0};
};

void NonResettableThreadMain(SEFUtility::CachingFactoryCacheOrDestroy cacheOrDestroy, long numLoops)
{
    SEFUtility::CachingFactory<NonResettableObject> factory;

    for (int i = 0; i < numLoops; i++)
    {
        SEFUtility::CachingFactory<NonResettableObject>::UniquePtr object = factory.GetInstance(cacheOrDestroy);

        object->setValue(object->getValue() + 1);

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void ResettableThreadMain(SEFUtility::CachingFactoryCacheOrDestroy cacheOrDestroy, long numLoops)
{
    SEFUtility::CachingFactory<ResettableObject> factory;

    for (int i = 0; i < numLoops; i++)
    {
        SEFUtility::CachingFactory<ResettableObject>::UniquePtr object = factory.GetInstance(cacheOrDestroy);

        object->setValue(object->getValue() + 1);

        std::this_thread::sleep_for(std::chrono::microseconds(50));
    }
}

constexpr int NUM_THREADS = 200;
constexpr int NUM_LOOPS = 10000;

TEST_CASE("Caching Factory Tests", "[cork-base]")
{
    SECTION("Non Resetting Caching Factory Test")
    {
        SEFUtility::CachingFactory<NonResettableObject> nonResettableFactory;

        REQUIRE(SEFUtility::CachingFactory<NonResettableObject>::numObjectsInCache() == 0);

        {
            SEFUtility::CachingFactory<NonResettableObject>::UniquePtr object1 =
                nonResettableFactory.GetInstance(SEFUtility::CachingFactoryCacheOrDestroy::CACHE);

            REQUIRE(SEFUtility::CachingFactory<NonResettableObject>::numObjectsInCache() == 0);

            object1->setValue(1);
        }

        REQUIRE(SEFUtility::CachingFactory<NonResettableObject>::numObjectsInCache() == 1);

        {
            SEFUtility::CachingFactory<NonResettableObject>::UniquePtr object1 =
                nonResettableFactory.GetInstance(SEFUtility::CachingFactoryCacheOrDestroy::CACHE);

            REQUIRE(SEFUtility::CachingFactory<NonResettableObject>::numObjectsInCache() == 0);
            REQUIRE(object1->getValue() == 1);

            SEFUtility::CachingFactory<NonResettableObject>::UniquePtr object2 =
                nonResettableFactory.GetInstance(SEFUtility::CachingFactoryCacheOrDestroy::CACHE);

            REQUIRE(SEFUtility::CachingFactory<NonResettableObject>::numObjectsInCache() == 0);
            REQUIRE(object2->getValue() == 0);

            object2->setValue(2);
        }

        REQUIRE(SEFUtility::CachingFactory<NonResettableObject>::numObjectsInCache() == 2);

        {
            SEFUtility::CachingFactory<NonResettableObject>::UniquePtr object1 =
                nonResettableFactory.GetInstance(SEFUtility::CachingFactoryCacheOrDestroy::DESTROY);

            REQUIRE(SEFUtility::CachingFactory<NonResettableObject>::numObjectsInCache() == 1);
            REQUIRE(object1->getValue() == 2);

            SEFUtility::CachingFactory<NonResettableObject>::UniquePtr object2 =
                nonResettableFactory.GetInstance(SEFUtility::CachingFactoryCacheOrDestroy::CACHE);

            REQUIRE(SEFUtility::CachingFactory<NonResettableObject>::numObjectsInCache() == 0);
            REQUIRE(object2->getValue() == 1);

            SEFUtility::CachingFactory<NonResettableObject>::UniquePtr object3 =
                nonResettableFactory.GetInstance(SEFUtility::CachingFactoryCacheOrDestroy::CACHE);

            REQUIRE(SEFUtility::CachingFactory<NonResettableObject>::numObjectsInCache() == 0);
            REQUIRE(object3->getValue() == 0);

            object3->setValue(3);
        }

        REQUIRE(SEFUtility::CachingFactory<NonResettableObject>::numObjectsInCache() == 2);

        SEFUtility::CachingFactory<NonResettableObject>::clear();

        REQUIRE(SEFUtility::CachingFactory<NonResettableObject>::numObjectsInCache() == 0);
    }

    SECTION("Resetting Caching Factory Test")
    {
        SEFUtility::CachingFactory<ResettableObject> resettableFactory;

        REQUIRE(SEFUtility::CachingFactory<ResettableObject>::numObjectsInCache() == 0);

        {
            SEFUtility::CachingFactory<ResettableObject>::UniquePtr object1 =
                resettableFactory.GetInstance(SEFUtility::CachingFactoryCacheOrDestroy::CACHE);

            REQUIRE(SEFUtility::CachingFactory<ResettableObject>::numObjectsInCache() == 0);

            object1->setValue(1);
        }

        REQUIRE(SEFUtility::CachingFactory<ResettableObject>::numObjectsInCache() == 1);

        {
            SEFUtility::CachingFactory<ResettableObject>::UniquePtr object1 =
                resettableFactory.GetInstance(SEFUtility::CachingFactoryCacheOrDestroy::CACHE);

            REQUIRE(SEFUtility::CachingFactory<ResettableObject>::numObjectsInCache() == 0);
            REQUIRE(object1->getValue() == 0);

            SEFUtility::CachingFactory<ResettableObject>::UniquePtr object2 =
                resettableFactory.GetInstance(SEFUtility::CachingFactoryCacheOrDestroy::CACHE);

            REQUIRE(SEFUtility::CachingFactory<ResettableObject>::numObjectsInCache() == 0);
            REQUIRE(object2->getValue() == 0);

            object2->setValue(2);
        }

        REQUIRE(SEFUtility::CachingFactory<ResettableObject>::numObjectsInCache() == 2);

        {
            SEFUtility::CachingFactory<ResettableObject>::UniquePtr object1 =
                resettableFactory.GetInstance(SEFUtility::CachingFactoryCacheOrDestroy::DESTROY);

            REQUIRE(SEFUtility::CachingFactory<ResettableObject>::numObjectsInCache() == 1);
            REQUIRE(object1->getValue() == 0);

            SEFUtility::CachingFactory<ResettableObject>::UniquePtr object2 =
                resettableFactory.GetInstance(SEFUtility::CachingFactoryCacheOrDestroy::CACHE);

            REQUIRE(SEFUtility::CachingFactory<ResettableObject>::numObjectsInCache() == 0);
            REQUIRE(object2->getValue() == 0);

            SEFUtility::CachingFactory<ResettableObject>::UniquePtr object3 =
                resettableFactory.GetInstance(SEFUtility::CachingFactoryCacheOrDestroy::CACHE);

            REQUIRE(SEFUtility::CachingFactory<ResettableObject>::numObjectsInCache() == 0);
            REQUIRE(object3->getValue() == 0);

            object3->setValue(3);
        }

        REQUIRE(SEFUtility::CachingFactory<ResettableObject>::numObjectsInCache() == 2);

        SEFUtility::CachingFactory<ResettableObject>::clear();

        REQUIRE(SEFUtility::CachingFactory<ResettableObject>::numObjectsInCache() == 0);
    }

    SECTION("Caching Factory Threading Test")
    {
        std::array<std::thread, NUM_THREADS> resettableThreads;
        std::array<std::thread, NUM_THREADS> nonResettableThreads;

        for (int i = 0; i < NUM_THREADS; i++)
        {
            resettableThreads[i] =
                std::thread(ResettableThreadMain, SEFUtility::CachingFactoryCacheOrDestroy::CACHE, NUM_LOOPS);
            nonResettableThreads[i] =
                std::thread(NonResettableThreadMain, SEFUtility::CachingFactoryCacheOrDestroy::CACHE, NUM_LOOPS);
        }

        for (int i = 0; i < NUM_THREADS; i++)
        {
            resettableThreads[i].join();
            nonResettableThreads[i].join();
        }

        REQUIRE(SEFUtility::CachingFactory<ResettableObject>::numObjectsInCache() == NUM_THREADS);
        REQUIRE(SEFUtility::CachingFactory<NonResettableObject>::numObjectsInCache() == NUM_THREADS);

        //	This is a resettable test, so all the objects should be at zero.

        long resettableTotalCount = 0;
        long nonResettableTotalCount = 0;

        for (int i = 0; i < NUM_THREADS; i++)
        {
            SEFUtility::CachingFactory<ResettableObject>::UniquePtr resettableObject =
                SEFUtility::CachingFactory<ResettableObject>::GetInstance(
                    SEFUtility::CachingFactoryCacheOrDestroy::DESTROY);
            SEFUtility::CachingFactory<NonResettableObject>::UniquePtr nonResettableObject =
                SEFUtility::CachingFactory<NonResettableObject>::GetInstance(
                    SEFUtility::CachingFactoryCacheOrDestroy::DESTROY);

            resettableTotalCount += resettableObject->getValue();
            nonResettableTotalCount += nonResettableObject->getValue();
        }

        REQUIRE(resettableTotalCount == 0);
        REQUIRE(nonResettableTotalCount == NUM_THREADS * NUM_LOOPS);

        REQUIRE(SEFUtility::CachingFactory<ResettableObject>::numObjectsInCache() == 0);
        REQUIRE(SEFUtility::CachingFactory<NonResettableObject>::numObjectsInCache() == 0);

        for (int i = 0; i < NUM_THREADS; i++)
        {
            resettableThreads[i] = std::thread(ResettableThreadMain,
                                               (i % 2) == 0 ? SEFUtility::CachingFactoryCacheOrDestroy::CACHE
                                                            : SEFUtility::CachingFactoryCacheOrDestroy::DESTROY,
                                               10000);
        }

        for (int i = 0; i < NUM_THREADS; i++)
        {
            resettableThreads[i].join();
        }
    }
}

//  NOLINTEND(cppcoreguidelines-avoid-magic-numbers)
