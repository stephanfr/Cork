/*
Copyright (c) 2013 Stephan Friedl

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

Except as contained in this notice, the name(s) of the above copyright holders
shall not be used in advertising or otherwise to promote the sale, use or other
dealings in this Software without prior written authorization.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
 */


#pragma once


#include <memory>

#include <boost\ptr_container\ptr_list.hpp>




template <class T>
class CachingFactory
{

	static void			CacheInstance( T*		instance )
	{
		m_cache.push_front( instance );
	}

	static void			DestroyInstance( T*		instance )
	{
		delete instance;
	}
		
public :

	typedef boost::ptr_list<T>										CacheType;
	typedef std::unique_ptr<T, decltype(&CacheInstance)>			UniquePtr;

	typedef enum class CacheOrDestroy { CACHE, DESTROY };


	static UniquePtr			GetInstance( CacheOrDestroy		cacheOrDestroy = CacheOrDestroy::CACHE )
	{
		decltype(&CacheInstance)		destroyFunction = cacheOrDestroy == CacheOrDestroy::CACHE ? CacheInstance : DestroyInstance;
		
		if( !m_cache.empty() )
		{
			return( UniquePtr( m_cache.pop_front().release(), destroyFunction ) );
		}
		
		return( UniquePtr( new T, destroyFunction ) );
	}

private :


	static boost::ptr_list<T>			m_cache;
};





