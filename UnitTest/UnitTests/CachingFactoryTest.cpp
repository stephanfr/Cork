
#include "pch.h"

#include <array>
#include <thread>
#include <chrono>

#include "Resettable.h"
#include "CachingFactory.h"


class NonResettableObject
{
public :

	NonResettableObject()
		: m_value( 0 )
	{}

	~NonResettableObject()
	{}


	long			getValue() const
	{
		return(m_value);
	}

	void			setValue(long		value)
	{
		m_value = value;
	}

private :

	long			m_value;
};




class ResettableObject : SEFUtility::Resettable
{
public:

	ResettableObject()
		: m_value(0)
	{}

	~ResettableObject()
	{}


	void			reset()
	{
		m_value = 0;
	}

	long			getValue() const
	{
		return(m_value);
	}

	void			setValue( long		value )
	{
		m_value = value;
	}

private:

	long			m_value;
};




SEFUtility::CachingFactory<NonResettableObject>::CacheType SEFUtility::CachingFactory<NonResettableObject>::m_cache;


bool	TestNonResettingCachingFactory()
{
	SEFUtility::CachingFactory<NonResettableObject>					nonResettableFactory;

	EXPECT_EQ(0, SEFUtility::CachingFactory<NonResettableObject>::numObjectsInCache());

	{
		SEFUtility::CachingFactory<NonResettableObject>::UniquePtr		object1 = nonResettableFactory.GetInstance(SEFUtility::CachingFactoryCacheOrDestroy::CACHE);

		EXPECT_EQ(0, SEFUtility::CachingFactory<NonResettableObject>::numObjectsInCache());

		object1->setValue ( 1 );
	}

	EXPECT_EQ(1, SEFUtility::CachingFactory<NonResettableObject>::numObjectsInCache());

	{
		SEFUtility::CachingFactory<NonResettableObject>::UniquePtr		object1 = nonResettableFactory.GetInstance(SEFUtility::CachingFactoryCacheOrDestroy::CACHE);

		EXPECT_EQ(0, SEFUtility::CachingFactory<NonResettableObject>::numObjectsInCache());
		EXPECT_EQ(1, object1->getValue());
		
		SEFUtility::CachingFactory<NonResettableObject>::UniquePtr		object2 = nonResettableFactory.GetInstance(SEFUtility::CachingFactoryCacheOrDestroy::CACHE);

		EXPECT_EQ(0, SEFUtility::CachingFactory<NonResettableObject>::numObjectsInCache());
		EXPECT_EQ(0, object2->getValue());

		object2->setValue( 2 );
	}

	EXPECT_EQ(2, SEFUtility::CachingFactory<NonResettableObject>::numObjectsInCache());


	{
		SEFUtility::CachingFactory<NonResettableObject>::UniquePtr		object1 = nonResettableFactory.GetInstance(SEFUtility::CachingFactoryCacheOrDestroy::DESTROY);

		EXPECT_EQ(1, SEFUtility::CachingFactory<NonResettableObject>::numObjectsInCache());
		EXPECT_EQ(2, object1->getValue());

		SEFUtility::CachingFactory<NonResettableObject>::UniquePtr		object2 = nonResettableFactory.GetInstance(SEFUtility::CachingFactoryCacheOrDestroy::CACHE);

		EXPECT_EQ(0, SEFUtility::CachingFactory<NonResettableObject>::numObjectsInCache());
		EXPECT_EQ(1, object2->getValue());

		SEFUtility::CachingFactory<NonResettableObject>::UniquePtr		object3 = nonResettableFactory.GetInstance(SEFUtility::CachingFactoryCacheOrDestroy::CACHE);

		EXPECT_EQ(0, SEFUtility::CachingFactory<NonResettableObject>::numObjectsInCache());
		EXPECT_EQ(0, object3->getValue());

		object3->setValue( 3 );
	}

	EXPECT_EQ(2, SEFUtility::CachingFactory<NonResettableObject>::numObjectsInCache());
	
	SEFUtility::CachingFactory<NonResettableObject>::clear();

	EXPECT_EQ(0, SEFUtility::CachingFactory<NonResettableObject>::numObjectsInCache());


	return(true);
}




SEFUtility::CachingFactory<ResettableObject>::CacheType SEFUtility::CachingFactory<ResettableObject>::m_cache;


bool	TestResettingCachingFactory()
{
	SEFUtility::CachingFactory<ResettableObject>					resettableFactory;

	EXPECT_EQ(0, SEFUtility::CachingFactory<ResettableObject>::numObjectsInCache());

	{
		SEFUtility::CachingFactory<ResettableObject>::UniquePtr		object1 = resettableFactory.GetInstance(SEFUtility::CachingFactoryCacheOrDestroy::CACHE);

		EXPECT_EQ(0, SEFUtility::CachingFactory<ResettableObject>::numObjectsInCache());

		object1->setValue( 1 );
	}

	EXPECT_EQ(1, SEFUtility::CachingFactory<ResettableObject>::numObjectsInCache());

	{
		SEFUtility::CachingFactory<ResettableObject>::UniquePtr		object1 = resettableFactory.GetInstance(SEFUtility::CachingFactoryCacheOrDestroy::CACHE);

		EXPECT_EQ(0, SEFUtility::CachingFactory<ResettableObject>::numObjectsInCache());
		EXPECT_EQ(0, object1->getValue());

		SEFUtility::CachingFactory<ResettableObject>::UniquePtr		object2 = resettableFactory.GetInstance(SEFUtility::CachingFactoryCacheOrDestroy::CACHE);

		EXPECT_EQ(0, SEFUtility::CachingFactory<ResettableObject>::numObjectsInCache());
		EXPECT_EQ(0, object2->getValue());

		object2->setValue( 2 );
	}

	EXPECT_EQ(2, SEFUtility::CachingFactory<ResettableObject>::numObjectsInCache());


	{
		SEFUtility::CachingFactory<ResettableObject>::UniquePtr		object1 = resettableFactory.GetInstance(SEFUtility::CachingFactoryCacheOrDestroy::DESTROY);

		EXPECT_EQ(1, SEFUtility::CachingFactory<ResettableObject>::numObjectsInCache());
		EXPECT_EQ(0, object1->getValue());

		SEFUtility::CachingFactory<ResettableObject>::UniquePtr		object2 = resettableFactory.GetInstance(SEFUtility::CachingFactoryCacheOrDestroy::CACHE);

		EXPECT_EQ(0, SEFUtility::CachingFactory<ResettableObject>::numObjectsInCache());
		EXPECT_EQ(0, object2->getValue());

		SEFUtility::CachingFactory<ResettableObject>::UniquePtr		object3 = resettableFactory.GetInstance(SEFUtility::CachingFactoryCacheOrDestroy::CACHE);

		EXPECT_EQ(0, SEFUtility::CachingFactory<ResettableObject>::numObjectsInCache());
		EXPECT_EQ(0, object3->getValue());

		object3->setValue( 3 );
	}

	EXPECT_EQ(2, SEFUtility::CachingFactory<ResettableObject>::numObjectsInCache());

	SEFUtility::CachingFactory<ResettableObject>::clear();

	EXPECT_EQ(0, SEFUtility::CachingFactory<ResettableObject>::numObjectsInCache());



	return(true);
}


void	NonResettableThreadMain( SEFUtility::CachingFactoryCacheOrDestroy				cacheOrDestroy,
							     long													numLoops)
{
	SEFUtility::CachingFactory<NonResettableObject>			factory;
		
	for (int i = 0; i < numLoops; i++)
	{
		SEFUtility::CachingFactory<NonResettableObject>::UniquePtr  object = factory.GetInstance(cacheOrDestroy);

		object->setValue(object->getValue() + 1);

		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
}

void	ResettableThreadMain( SEFUtility::CachingFactoryCacheOrDestroy			cacheOrDestroy,
							  long												numLoops )
{
	SEFUtility::CachingFactory<ResettableObject>		factory;

	for (int i = 0; i < numLoops; i++)
	{
		SEFUtility::CachingFactory<ResettableObject>::UniquePtr  object = factory.GetInstance(cacheOrDestroy);

		object->setValue(object->getValue() + 1);

		std::this_thread::sleep_for(std::chrono::microseconds(50));
	}
}


#define NUM_THREADS 200
#define NUM_LOOPS 10000

bool	TestMultiThreading()
{
	std::array<std::thread, NUM_THREADS>			resettableThreads;
	std::array<std::thread, NUM_THREADS>			nonResettableThreads;

	for (int i = 0; i < NUM_THREADS; i++)
	{
		resettableThreads[i] = std::thread(ResettableThreadMain, SEFUtility::CachingFactoryCacheOrDestroy::CACHE, NUM_LOOPS);
		nonResettableThreads[i] = std::thread(NonResettableThreadMain, SEFUtility::CachingFactoryCacheOrDestroy::CACHE, NUM_LOOPS);
	}

	for (int i = 0; i < NUM_THREADS; i++)
	{
		resettableThreads[i].join();
		nonResettableThreads[i].join();
	}

	EXPECT_EQ(NUM_THREADS, SEFUtility::CachingFactory<ResettableObject>::numObjectsInCache());
	EXPECT_EQ(NUM_THREADS, SEFUtility::CachingFactory<NonResettableObject>::numObjectsInCache());

	//	This is a resettable test, so all the objects should be at zero.

	long	resettableTotalCount = 0;
	long	nonResettableTotalCount = 0;

	for (int i = 0; i < NUM_THREADS; i++)
	{
		SEFUtility::CachingFactory<ResettableObject>::UniquePtr  resettableObject = SEFUtility::CachingFactory<ResettableObject>::GetInstance(SEFUtility::CachingFactoryCacheOrDestroy::DESTROY);
		SEFUtility::CachingFactory<NonResettableObject>::UniquePtr  nonResettableObject = SEFUtility::CachingFactory<NonResettableObject>::GetInstance(SEFUtility::CachingFactoryCacheOrDestroy::DESTROY);

		resettableTotalCount += resettableObject->getValue();
		nonResettableTotalCount += nonResettableObject->getValue();
	}

	EXPECT_EQ(0, resettableTotalCount);
	EXPECT_EQ(NUM_THREADS * NUM_LOOPS, nonResettableTotalCount);

	EXPECT_EQ(0, SEFUtility::CachingFactory<ResettableObject>::numObjectsInCache());
	EXPECT_EQ(0, SEFUtility::CachingFactory<NonResettableObject>::numObjectsInCache());

	for (int i = 0; i < NUM_THREADS; i++)
	{
		resettableThreads[i] = std::thread(ResettableThreadMain, (i % 2) == 0 ? SEFUtility::CachingFactoryCacheOrDestroy::CACHE : SEFUtility::CachingFactoryCacheOrDestroy::DESTROY, 10000);
	}

	for (int i = 0; i < NUM_THREADS; i++)
	{
		resettableThreads[i].join();
	}

	return(true);
}


TEST(CachingFactoy, BasicTest)
{
	EXPECT_TRUE(TestNonResettingCachingFactory());
	EXPECT_TRUE(TestResettingCachingFactory());
	EXPECT_TRUE(TestMultiThreading());
}

