// +-------------------------------------------------------------------------
// | ManagedIntrusiveList.h
// |
// | Author: Stephan Friedl
// +-------------------------------------------------------------------------
// | COPYRIGHT:
// |    Copyright Stephan Friedl 2013
// |    See the included COPYRIGHT file for further details.
// |
// |    This file is part of the Cork library.
// |
// |    Cork is free software: you can redistribute it and/or modify
// |    it under the terms of the GNU Lesser General Public License as
// |    published by the Free Software Foundation, either version 3 of
// |    the License, or (at your option) any later version.
// |
// |    Cork is distributed in the hope that it will be useful,
// |    but WITHOUT ANY WARRANTY; without even the implied warranty of
// |    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// |    GNU Lesser General Public License for more details.
// |
// |    You should have received a copy
// |    of the GNU Lesser General Public License
// |    along with Cork.  If not, see <http://www.gnu.org/licenses/>.
// +-------------------------------------------------------------------------
#pragma once

#include <type_traits>

#include "boost/intrusive/list.hpp"
#include "boost/intrusive/unordered_set.hpp"
#include "boost/noncopyable.hpp"
#include "tbb/concurrent_vector.h"
#include "tbb/spin_mutex.h"
#include "type_safe/integer.hpp"

namespace hidden
{
    template <typename T>
    class PointerOnlyListElement
        : public boost::intrusive::list_base_hook<boost::intrusive::link_mode<boost::intrusive::normal_link>>
    {
       public:
        explicit PointerOnlyListElement(T* pointer) : m_pointer(pointer) {}

        operator T*() { return (m_pointer); }

        operator const T*() const { return (m_pointer); }

        T* operator->() { return (m_pointer); }

        const T* operator->() const { return (m_pointer); }

        T* pointer() { return (m_pointer); }

        T* m_pointer;
    };

}  // namespace hidden

template <typename T, typename TI = size_t>
class ContiguousStorage : public boost::noncopyable
{
   public:
    ContiguousStorage(size_t starting_size = 10000)
        : current_size_(starting_size), end_(0), block_(static_cast<T*>(malloc(sizeof(T) * (starting_size + 1))))
    {
    }

    ~ContiguousStorage() { free(block_); }

    void reserve(size_t new_minimum_size)
    {
        if (new_minimum_size > current_size_)
        {
            std::cout << "Growing ContiguousStorage from: " << current_size_ << " to " << new_minimum_size << std::endl;

            current_size_ = new_minimum_size;
            free(block_);
            block_ = static_cast<T*>(malloc(sizeof(T) * (current_size_ + 1)));
        }

        end_ = 0;
    }

    void clear() { end_ = 0; }

    T& operator[](TI index)
    {
        assert(index < end_);

        return *(block_ + static_cast<uint32_t>(index));
    }

    const T& operator[](TI index) const
    {
        assert(index < end_);

        return *(block_ + static_cast<uint32_t>(index));
    }

    T& back()
    {
        assert(end_ > 0);

        return *(block_ + (end_ - 1));
    }

    const T& back() const
    {
        assert(end_ > 0);

        return *(block_ + (end_ - 1));
    }

    template <class... _Valty>
    T* emplace_back(_Valty&&... _Val)
    {
        assert(end_ < current_size_);
        return new (block_ + end_++) T(std::forward<_Valty>(_Val)...);
    }

   private:
    size_t current_size_;
    size_t end_;
    T* block_;
};

template <typename T>
class ManagedIntrusivePointerList : public boost::noncopyable,
                                    protected boost::intrusive::list<hidden::PointerOnlyListElement<T>>
{
   private:
    typedef boost::intrusive::list<hidden::PointerOnlyListElement<T>> BaseType;

   public:
    typedef tbb::concurrent_vector<hidden::PointerOnlyListElement<T>> PoolType;

    explicit ManagedIntrusivePointerList(PoolType& pool) : m_pool(pool) {}

    ManagedIntrusivePointerList(const ManagedIntrusivePointerList&) = delete;

    using BaseType::iterator;

    using BaseType::begin;
    using BaseType::end;
    using BaseType::front;

    using BaseType::clear;
    using BaseType::empty;
    using BaseType::erase;
    using BaseType::size;

    void push_back(T* pointerToAdd) { BaseType::push_back(*(m_pool.emplace_back(pointerToAdd))); }

   protected:
    PoolType& m_pool;
};

typedef boost::intrusive::list_base_hook<> IntrusiveListHook;
typedef boost::intrusive::list_base_hook<boost::intrusive::link_mode<boost::intrusive::normal_link>>
    IntrusiveListHookNoDestructorOnElements;

//  For instances where we have strongly typed index types - we have the ValuePool below which will
//      conditionally compile support for type_safe::integer<uint32_t> indices.  The list will default
//      to uint32_t indices with an appropriate pool.

template <typename T, typename TI>
class ValuePool : public tbb::concurrent_vector<T>
{
   public:
    T& operator[](size_t) = delete;
    const T& operator[](size_t) const = delete;

    T& operator[](TI index)
    {
        if constexpr (std::is_base_of<type_safe::integer<uint32_t>, TI>::value)
        {
            return tbb::concurrent_vector<T>::operator[](typename TI::integer_type(index));
        }
        else
        {
            return tbb::concurrent_vector<T>::operator[](index);
        }
    }

    const T& operator[](TI index) const
    {
        if constexpr (std::is_base_of<type_safe::integer<uint32_t>, TI>::value)
        {
            return tbb::concurrent_vector<T>::operator[](typename TI::integer_type(index));
        }
        else
        {
            return tbb::concurrent_vector<T>::operator[](index);
        }
    }
};

template <typename T, typename TI = uint32_t, typename TP = ValuePool<T, TI>>
class ManagedIntrusiveValueList : public boost::noncopyable,
                                  protected boost::intrusive::list<T, boost::intrusive::constant_time_size<true>>
{
   private:
    typedef boost::intrusive::list<T, boost::intrusive::constant_time_size<true>> BaseType;

   public:
    //    typedef tbb::concurrent_vector<T> PoolType;
    using PoolType = TP;

    explicit ManagedIntrusiveValueList(PoolType& pool) : m_pool(pool) {}

    using BaseType::begin;
    using BaseType::clear;
    using BaseType::end;
    using BaseType::size;

    template <class... _Valty>
    T* emplace_back(_Valty&&... _Val)
    {
        T& newValue = *(m_pool.emplace_back(std::forward<_Valty>(_Val)...));

        BaseType::push_back(newValue);

        return (&newValue);
    }

    template <class... _Valty>
    T* emplace_back_unindexed(_Valty&&... _Val)
    {
        T& newValue = *(m_pool.emplace_back(std::forward<_Valty>(_Val)...));

        return (&newValue);
    }

    bool isCompact() const { return (m_pool.size() == size()); }

    PoolType& getPool() { return (m_pool); }

    void FixupList()
    {
        BaseType::clear();

        for (auto itrElement = m_pool.begin(); itrElement != m_pool.end(); itrElement++)
        {
            BaseType::push_back(*itrElement);
        }
    }

    void free(T& valueToFree)
    {
        BaseType::erase(
            boost::intrusive::list<T, boost::intrusive::constant_time_size<true>>::iterator_to(valueToFree));
    }

    void free(const T& valueToFree)
    {
        BaseType::erase(
            boost::intrusive::list<T, boost::intrusive::constant_time_size<true>>::iterator_to(valueToFree));
    }

    void free(T* valueToFree)
    {
        BaseType::erase(
            boost::intrusive::list<T, boost::intrusive::constant_time_size<true>>::iterator_to(*valueToFree));
    }

    void free(const T* valueToFree)
    {
        BaseType::erase(
            boost::intrusive::list<T, boost::intrusive::constant_time_size<true>>::iterator_to(*valueToFree));
    }

   private:
    PoolType& m_pool;
};
