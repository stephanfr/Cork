
#pragma once


#include <tbb/mutex.h>
#include <tbb/spin_mutex.h>



template<class Container, class... Args>
auto threadSafeEmplaceBack(Container& c, tbb::mutex&	lock, Args&&... args) -> decltype (c.back())
{
	lock.lock();
	c.emplace_back(std::forward<Args>(args)...);
	decltype (c.back())	newElement( c.back() );
	lock.unlock();

	return( newElement );
}


template<class Container, class... Args>
auto threadSafeEmplaceBack(Container& c, tbb::spin_mutex&	lock, Args&&... args) -> decltype (c.back())
{
	lock.lock();
	c.emplace_back(std::forward<Args>(args)...);
	decltype (c.back())	newElement( c.back() );
	lock.unlock();

	return( newElement );
}


