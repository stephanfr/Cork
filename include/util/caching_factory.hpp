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

#include <deque>
#include <future>
#include <memory>
#include <type_traits>

#include "boost/smart_ptr/detail/spinlock.hpp"
#include "resettable.hpp"

namespace SEFUtility
{
    enum class CachingFactoryCacheOrDestroy
    {
        CACHE,
        DESTROY
    };

    template <class T>
    class CachingFactory
    {
        //	We have some SFINAE action below.  If the object has the Resettable interface,
        //		then compile a version of CacheInstanceInternal that calls reset before caching.
        //		If the object does not have the Resettable interface, then compile a version that
        //		does not call reset before recaching.

        template <bool cond, typename U>
        using resolvedType = typename std::enable_if<cond, U>::type;

        template <class Q = T>
        static void CacheInstanceInternal(resolvedType<std::is_base_of<Resettable, Q>::value, Q*> instance)
        {
            instance->reset();

            std::lock_guard<boost::detail::spinlock> guard(cache_.lock());
            cache_.push_back(instance);
        }

        template <class Q = T>
        static void CacheInstanceInternal(resolvedType<!std::is_base_of<Resettable, Q>::value, Q*> instance)
        {
            std::lock_guard<boost::detail::spinlock> guard(cache_.lock());
            cache_.push_back(instance);
        }

        static void CacheInstance(T* instance) { CacheInstanceInternal(instance); }

        static void DestroyInstance(T* instance) { delete instance; }

       public:
        //	The CacheType is a deque of pointers with a destructor that destroys all the pointers.
        //		The class also contains a spin lock to make the class itself thread re-entrant.

        template <class T1>
        class CacheType : public std::deque<T1*>
        {
           public:
            ~CacheType() { clear(); }

            void clear()
            {
                std::lock_guard<boost::detail::spinlock> guard(m_lock);

                while (!std::deque<T1*>::empty())
                {
                    delete std::deque<T1*>::front();
                    std::deque<T1*>::pop_front();
                };
            }

            boost::detail::spinlock& lock() { return (m_lock); }

           private:
            boost::detail::spinlock m_lock;
        };

        //	The UniquePtr is simply a std::unique_ptr<T1> with a custom destructor based
        //		on whether the cache or destroy the object

        typedef std::unique_ptr<T, decltype(&CacheInstance)> UniquePtr;

        //	GetInstance() returns an instance of the managed class either from the cache or by creating
        //		it new and associates the correct destructor to either cache or destroy the class when
        //		the UniquePtr is destroyed.

        static UniquePtr GetInstance(CachingFactoryCacheOrDestroy cacheOrDestroy = CachingFactoryCacheOrDestroy::CACHE)
        {
            decltype(&CacheInstance) destroyFunction =
                cacheOrDestroy == CachingFactoryCacheOrDestroy::CACHE ? CacheInstance : DestroyInstance;

            std::lock_guard<boost::detail::spinlock> guard(cache_.lock());

            if (!cache_.empty())
            {
                UniquePtr returnValue(cache_.front(), destroyFunction);

                cache_.pop_front();

                return (returnValue);
            }

            return (UniquePtr(new T, destroyFunction));
        }

        //	Clear the cache

        static void clear() { cache_.clear(); }

        //	Return the number of elements in the cache.

        static size_t numObjectsInCache() { return (cache_.size()); }

       private:
        inline static CacheType<T> cache_;
    };

};  // namespace SEFUtility
