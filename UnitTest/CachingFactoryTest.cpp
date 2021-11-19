
#include <array>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <thread>

#include "util/CachingFactory.h"
#include "util/resettable.hpp"

//  The pragma below is to disable to false errors flagged by intellisense for Catch2 REQUIRE macros.

#if __INTELLISENSE__
#pragma diag_suppress 2486
#endif

class NonResettableObject
{
   public:
    NonResettableObject() : m_value(0) {}

    ~NonResettableObject() {}

    long getValue() const { return (m_value); }

    void setValue(long value) { m_value = value; }

   private:
    long m_value;
};

class ResettableObject : SEFUtility::Resettable
{
   public:
    ResettableObject() : m_value(0) {}

    ~ResettableObject() {}

    void reset() { m_value = 0; }

    long getValue() const { return (m_value); }

    void setValue(long value) { m_value = value; }

   private:
    long m_value;
};

TEST_CASE("Test Non Resetting Caching Factory", "[cork-base]")
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

TEST_CASE("Test Resetting Caching Factory", "[cork-base]")
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

TEST_CASE("Test Caching Factory Threading", "[cork-base]")
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
