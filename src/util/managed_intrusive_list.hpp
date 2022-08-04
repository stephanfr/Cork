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
        explicit PointerOnlyListElement(T* pointer) : pointer_(pointer) {}

        operator T*() { return (pointer_); }  //  NOLINT(hicpp-explicit-conversions, google-explicit-constructor)

        operator const T*() const
        {
            return (pointer_);
        }  //  NOLINT(hicpp-explicit-conversions, google-explicit-constructor)

        T* operator->() { return (pointer_); }

        const T* operator->() const { return (pointer_); }

        T* pointer() { return (pointer_); }

        T* pointer_;
    };

}  // namespace hidden

template <typename T>
class ManagedIntrusivePointerList : public boost::noncopyable,
                                    protected boost::intrusive::list<hidden::PointerOnlyListElement<T>>
{
   private:
    using BaseType = boost::intrusive::list<hidden::PointerOnlyListElement<T>>;

   public:
    using PoolType = tbb::concurrent_vector<hidden::PointerOnlyListElement<T>>;

    ManagedIntrusivePointerList() = delete;

    explicit ManagedIntrusivePointerList(PoolType& pool) : pool_(pool) {}

    ManagedIntrusivePointerList(const ManagedIntrusivePointerList&) = delete;
    ManagedIntrusivePointerList(ManagedIntrusivePointerList&&) = delete;

    virtual ~ManagedIntrusivePointerList() = default;

    ManagedIntrusivePointerList& operator=(const ManagedIntrusivePointerList&) = delete;
    ManagedIntrusivePointerList& operator=(ManagedIntrusivePointerList&&) = delete;

    using BaseType::iterator;

    using BaseType::begin;
    using BaseType::end;
    using BaseType::front;

    using BaseType::clear;
    using BaseType::empty;
    using BaseType::erase;
    using BaseType::size;

    void push_back(T* pointerToAdd) { BaseType::push_back(*(pool_.emplace_back(pointerToAdd))); }

   protected:
    PoolType& pool_;
};

using IntrusiveListHook = boost::intrusive::list_base_hook<>;
using IntrusiveListHookNoDestructorOnElements =
    boost::intrusive::list_base_hook<boost::intrusive::link_mode<boost::intrusive::normal_link>>;

template <typename T, typename TI>
class ValuePool : public tbb::concurrent_vector<T>
{
   public:
    //    T& operator[](size_t) = delete;
    //    const T& operator[](size_t) const = delete;

    T& operator[](TI index) { return tbb::concurrent_vector<T>::operator[](static_cast<size_t>(index)); }

    const T& operator[](TI index) const { return tbb::concurrent_vector<T>::operator[](static_cast<size_t>(index)); }
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

    explicit ManagedIntrusiveValueList(PoolType& pool) : pool_(pool) {}

    using BaseType::begin;
    using BaseType::clear;
    using BaseType::end;
    using BaseType::size;

    template <class... _Valty>
    T* emplace_back(_Valty&&... _Val)
    {
        T& newValue = *(pool_.emplace_back(std::forward<_Valty>(_Val)...));

        BaseType::push_back(newValue);

        return (&newValue);
    }

    template <class... _Valty>
    T* emplace_back_unindexed(_Valty&&... _Val)
    {
        T& newValue = *(pool_.emplace_back(std::forward<_Valty>(_Val)...));

        return (&newValue);
    }

    [[nodiscard]] bool isCompact() const { return (pool_.size() == size()); }

    PoolType& getPool() { return (pool_); }
    const PoolType& getPool() const { return (pool_); }

    void FixupList()
    {
        BaseType::clear();

        for (auto itrElement = pool_.begin(); itrElement != pool_.end(); itrElement++)
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
    PoolType& pool_;
};
