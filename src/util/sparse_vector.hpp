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

#include <boost/container/static_vector.hpp>
#include <boost/integer/static_min_max.hpp>
#include <boost/iterator/iterator_facade.hpp>
#include <functional>
#include <memory>
#include <set>
#include <unordered_map>

#include "resettable.hpp"

namespace SEFUtility
{
    class SparseVectorEntry
    {
       public:
        explicit SparseVectorEntry(size_t index) : index_(index) {}

        [[nodiscard]] size_t index() const { return (index_); }

       private:
        size_t index_;
    };

    template <class T>
    class UnorderedMapPool
    {
       public:
        virtual std::unordered_map<size_t, T>* new_map(size_t initial_size) = 0;
        virtual void release_map(std::unordered_map<size_t, T>* map) = 0;
    };

    template <class T>
    class DefaultMapPool : public UnorderedMapPool<T>
    {
        std::unordered_map<size_t, T>* new_map(size_t initial_size)
        {
            return new std::unordered_map<size_t, T>(initial_size);
        }

        void release_map(std::unordered_map<size_t, T>* map) { delete map; }
    };

    template <class T, size_t CUTOVER_SIZE>
    class SparseVector : public Resettable
    {
       private:
        using EntryVector = boost::container::static_vector<T, CUTOVER_SIZE>;
        using EntryVectorIterator = typename boost::container::static_vector<T, CUTOVER_SIZE>::iterator;
        using EntryVectorConstIterator = typename boost::container::static_vector<T, CUTOVER_SIZE>::const_iterator;

        using EntryMap = std::unordered_map<size_t, T>;
        using EntryMapIterator = typename EntryMap::iterator;

       public:
        using MapPoolType = UnorderedMapPool<T>;

        class iterator : public boost::iterator_facade<iterator, T*, boost::forward_traversal_tag, T*>
        {
           protected:
            friend class SparseVector;

            explicit iterator(const EntryVectorIterator& vectorIterator) : cut_over_(false)
            {
                vector_iterator_ = vectorIterator;
            }

            explicit iterator(const EntryMapIterator& setIterator) : cut_over_(true) { set_iterator_ = setIterator; }

            friend class boost::iterator_core_access;

            void increment()
            {
                if (cut_over_)
                {
                    set_iterator_++;
                }
                else
                {
                    vector_iterator_++;
                }
            }

            bool equal(iterator const& other) const
            {
                if (cut_over_)
                {
                    return (set_iterator_ == other.set_iterator_);
                }

                return (vector_iterator_ == other.vector_iterator_);
            }

            T* dereference() const
            {
                if (cut_over_)
                {
                    return (&(set_iterator_->second));
                }

                return (&*vector_iterator_);
            }

            bool cut_over_;

            EntryVectorIterator vector_iterator_;
            EntryMapIterator set_iterator_;
        };

        class const_iterator : public boost::iterator_facade<const_iterator, T*, boost::forward_traversal_tag, const T*>
        {
           protected:
            friend class SparseVector;

            explicit const_iterator(const EntryVectorIterator& vectorIterator) : cut_over_(false)
            {
                vector_iterator_ = vectorIterator;
            }

            explicit const_iterator(const EntryMapIterator& setIterator) : cut_over_(true)
            {
                set_iterator_ = setIterator;
            }

            friend class boost::iterator_core_access;

            void increment()
            {
                if (cut_over_)
                {
                    set_iterator_++;
                }
                else
                {
                    vector_iterator_++;
                }
            }

            bool equal(iterator const& other) const
            {
                if (cut_over_)
                {
                    return (set_iterator_ == other.m_setIterator);
                }

                return (vector_iterator_ == other.m_vectorIterator);
            }

            T* dereference() const
            {
                if (cut_over_)
                {
                    return (&(set_iterator_->second));
                }

                return (vector_iterator_);
            }

            bool cut_over_;

            EntryVectorIterator vector_iterator_;
            EntryMapIterator set_iterator_;
        };

        SparseVector()
            : cut_over_(false),
              inserter_(&SparseVector<T, CUTOVER_SIZE>::insertIntoArray),
              find_or_add_(&SparseVector<T, CUTOVER_SIZE>::findOrAddArray),
              map_pool_(*(default_pool_.get())),
              map_(NULL)
        {
        }

        explicit SparseVector(MapPoolType& map_pool)
            : cut_over_(false),
              inserter_(&SparseVector<T, CUTOVER_SIZE>::insertIntoArray),
              find_or_add_(&SparseVector<T, CUTOVER_SIZE>::findOrAddArray),
              map_pool_(map_pool),
              map_(NULL)
        {
        }

        SparseVector(SparseVector&&) = delete;

        //	Need the copy constructor to keep the compiler quiet about being unable to copy fixed_vectors,
        //		but we should never call it - thus the assert.

        //  NOLINTNEXTLINE
        explicit SparseVector(const SparseVector& vectorToCopy) { assert(false); }

        ~SparseVector()
        {
            if (map_ != NULL)
            {
                map_pool_.release_map(map_);
            }
        }

        //	Need the assignment operator to keep the compiler quiet about being unable to copy fixed_vectors,
        //		but we should never call it - thus the assert.

        //  NOLINTNEXTLINE
        SparseVector& operator=(const SparseVector& vectorToCopy) { assert(false); }

        SparseVector& operator=(SparseVector&& vectorToCopy) = delete;

        void reset() override
        {
            if (map_ != NULL)
            {
                map_pool_.release_map(map_);
            }

            cut_over_ = false;
            array_.clear();
            inserter_ = &SparseVector<T, CUTOVER_SIZE>::insertIntoArray;
            find_or_add_ = &SparseVector<T, CUTOVER_SIZE>::findOrAddArray;
            map_ = NULL;
        }

        [[nodiscard]] size_t size() const
        {
            if (!cut_over_)
            {
                return (array_.size());
            }

            return (map_->size());
        }

        [[nodiscard]] bool empty() const
        {
            if (!cut_over_)
            {
                return (array_.size() == 0);
            }

            return (map_->empty());
        }

        iterator begin()
        {
            if (!cut_over_)
            {
                return (iterator(array_.begin()));
            }

            return (iterator(map_->begin()));
        }

        const_iterator begin() const
        {
            if (!cut_over_)
            {
                return (const_iterator(array_.begin()));
            }

            return (const_iterator(map_->begin()));
        }

        iterator end()
        {
            if (!cut_over_)
            {
                return (iterator(array_.end()));
            }

            return (iterator(map_->end()));
        }

        const_iterator end() const
        {
            if (!cut_over_)
            {
                return (const_iterator(array_.end()));
            }

            return (const_iterator(map_->end()));
        }

        T& operator[](size_t index)
        {
            if (!cut_over_)
            {
                for (unsigned int i = 0; i < array_.size(); i++)
                {
                    if (array_[i].index() == index)
                    {
                        return (array_[i]);
                    }
                }

                //	We did not find the entry so there is no choice but assert

                assert(false);
            }

            EntryMapIterator itrEntry;

            itrEntry = map_->find(index);

            if (itrEntry == map_->end())
            {
                //	We did not find the entry so there is no choice but assert

                assert(false);
            }

            return (itrEntry->second);
        }

        const T& operator[](size_t index) const
        {
            if (!cut_over_)
            {
                for (unsigned int i = 0; i < array_.size(); i++)
                {
                    if (array_[i].index() == index)
                    {
                        return (array_[i]);
                    }
                }

                //	We did not find the entry so there is no choice but assert

                assert(false);
            }

            EntryMapIterator itrEntry;

            itrEntry = map_->find(index);

            if (itrEntry == map_->end())
            {
                //	We did not find the entry so there is no choice but assert

                assert(false);
            }

            return (itrEntry->second);
        }

        T& find_or_add(size_t index) { return ((this->*find_or_add_)(index)); }

        inline void for_each(std::function<void(T& entry)> action)
        {
            if (!cut_over_)
            {
                for (unsigned int i = 0; i < array_.size(); i++)
                {
                    action(array_[i]);
                }
            }
            else
            {
                for (auto& currentEntry : *map_)
                {
                    action(currentEntry.second);
                }
            }
        }

        void elements(std::vector<T*>& elementVector)
        {
            if (!cut_over_)
            {
                for (unsigned int i = 0; i < array_.size(); i++)
                {
                    elementVector.push_back(array_ + i);
                }
            }
            else
            {
                for (auto& currentEntry : *map_)
                {
                    elementVector.push_back(&(currentEntry.second));
                }
            }
        }

       private:
        static std::unique_ptr<DefaultMapPool<T>> default_pool_;

        using InsertFunctionPointer = T& (SparseVector<T, CUTOVER_SIZE>::*)(size_t);
        using FindOrAddFunctionPointer = T& (SparseVector<T, CUTOVER_SIZE>::*)(size_t);

        bool cut_over_;

        InsertFunctionPointer inserter_;
        FindOrAddFunctionPointer find_or_add_;

        EntryVector array_;

        EntryMap* map_;

        MapPoolType& map_pool_;

        T& insertIntoArray(size_t index)
        {
            if (array_.size() < CUTOVER_SIZE)
            {
                array_.emplace_back(index);

                return (array_.back());
            }

            map_ = map_pool_.new_map(CUTOVER_SIZE * 4);

            for (unsigned int i = 0; i < array_.size(); i++)
            {
                map_->emplace(array_[i].index(), array_[i]);
            }

            cut_over_ = true;

            inserter_ = &SparseVector<T, CUTOVER_SIZE>::insertIntoMap;
            find_or_add_ = &SparseVector<T, CUTOVER_SIZE>::findOrAddMap;

            return (map_->emplace(index, index).first->second);
        }

        T& insertIntoMap(size_t index) { return (map_->emplace(index, index).first->second); }

        T& findOrAddArray(size_t index)
        {
            for (unsigned int i = 0; i < array_.size(); i++)
            {
                if (array_[i].index() == index)
                {
                    return (array_[i]);
                }
            }

            return ((this->*inserter_)(index));
        }

        T& findOrAddMap(size_t index)
        {
            EntryMapIterator itrEntry;

            itrEntry = map_->find(index);

            if (itrEntry != map_->end())
            {
                return (itrEntry->second);
            }

            return ((this->*inserter_)(index));
        }
    };

    template <class T, size_t CUTOVER_SIZE>
    std::unique_ptr<DefaultMapPool<T>> SparseVector<T, CUTOVER_SIZE>::default_pool_(new DefaultMapPool<T>());

    template <class T>
    class PointerSetPool
    {
       public:
        virtual std::set<T*>* new_set() = 0;
        virtual void release_set(std::set<T*>* set) = 0;
    };

    template <class T>
    class DefaultPointerSetPool : public PointerSetPool<T>
    {
        std::set<T*>* new_set() { return new std::set<T*>(); }

        void release_set(std::set<T*>* set) { delete set; }
    };

    template <class T, size_t CUTOVER_SIZE>
    class SearchablePointerList
    {
       private:
        using EntryVector = boost::container::static_vector<T*, CUTOVER_SIZE>;
        using EntryVectorIterator = typename boost::container::static_vector<T*, CUTOVER_SIZE>::iterator;
        using EntryVectorConstIterator = typename boost::container::static_vector<T*, CUTOVER_SIZE>::const_iterator;

        using EntrySet = std::set<T*>;
        using EntrySetIterator = typename EntrySet::iterator;
        using EntrySetConstIterator = typename EntrySet::const_iterator;

       public:
        using SetPoolType = PointerSetPool<T>;

        class iterator : public boost::iterator_facade<iterator, T*, boost::forward_traversal_tag, T*>
        {
           protected:
            friend class SearchablePointerList;

            explicit iterator(const EntryVectorIterator& vectorIterator) : cut_over_(false)
            {
                vector_iterator_ = new (buffer_) EntryVectorIterator(vectorIterator);
            }

            explicit iterator(const EntrySetIterator& setIterator) : cut_over_(true)
            {
                set_iterator_ = new (buffer_) EntrySetIterator(setIterator);
            }

            friend class boost::iterator_core_access;

            void increment()
            {
                if (cut_over_)
                {
                    (*set_iterator_)++;
                }
                else
                {
                    (*vector_iterator_)++;
                }
            }

            bool equal(iterator const& other) const
            {
                if (cut_over_)
                {
                    return (*set_iterator_ == *(other.set_iterator_));
                }

                return (*vector_iterator_ == *(other.vector_iterator_));
            }

            T* dereference() const
            {
                if (cut_over_)
                {
                    return (*(*set_iterator_));
                }

                return (*(*vector_iterator_));
            }

            static constexpr size_t BUFFER_SIZE =
                boost::static_unsigned_max<sizeof(EntryVectorIterator), sizeof(EntrySetIterator)>::value;

            std::array<unsigned char, BUFFER_SIZE> buffer_;

            bool cut_over_;

            union
            {
                EntryVectorIterator* vector_iterator_;
                EntrySetIterator* set_iterator_;
            };
        };

        class const_iterator : public boost::iterator_facade<const_iterator, T*, boost::forward_traversal_tag, T* const>
        {
           protected:
            friend class SearchablePointerList;

            explicit const_iterator(const EntryVectorConstIterator& vectorIterator) : cut_over_(false)
            {
                vector_iterator_ = new (buffer_) EntryVectorConstIterator(vectorIterator);
            }

            explicit const_iterator(const EntrySetConstIterator& setIterator) : cut_over_(true)
            {
                set_iterator_ = new (buffer_) EntrySetConstIterator(setIterator);
            }

            friend class boost::iterator_core_access;

            void increment()
            {
                if (cut_over_)
                {
                    (*set_iterator_)++;
                }
                else
                {
                    (*vector_iterator_)++;
                }
            }

            bool equal(const_iterator const& other) const
            {
                if (cut_over_)
                {
                    return (*set_iterator_ == *(other.set_iterator_));
                }

                return (*vector_iterator_ == *(other.vector_iterator_));
            }

            T* dereference() const
            {
                if (cut_over_)
                {
                    return (*(*set_iterator_));
                }

                return (*(*vector_iterator_));
            }

            bool cut_over_;

            union
            {
                EntryVectorConstIterator* vector_iterator_;
                EntrySetConstIterator* set_iterator_;
            };

            static constexpr size_t BUFFER_SIZE =
                boost::static_unsigned_max<sizeof(EntryVectorConstIterator), sizeof(EntrySetConstIterator)>::value;

            std::array<unsigned char, BUFFER_SIZE> buffer_;
        };

        SearchablePointerList()
            : cut_over_(false),
              inserter_(&SearchablePointerList<T, CUTOVER_SIZE>::insertIntoArray),
              map_(NULL),
              set_pool_(*(default_pool_.get()))
        {
        }

        explicit SearchablePointerList(SetPoolType& set_pool)
            : cut_over_(false),
              inserter_(&SearchablePointerList<T, CUTOVER_SIZE>::insertIntoArray),
              map_(NULL),
              set_pool_(set_pool)
        {
        }

        //	Need the copy constructor to keep the compiler quiet about being unable to copy fixed_vectors,
        //		but we should never call it - thus the assert.

        //  NOLINTNEXTLINE
        SearchablePointerList(const SearchablePointerList& vectorToCopy) { assert(false); }

        SearchablePointerList(SearchablePointerList&&) = delete;

        ~SearchablePointerList()
        {
            if (map_ != NULL)
            {
                set_pool_.release_set(map_);
            }
        }

        //	Need the assignment operator to keep the compiler quiet about being unable to copy fixed_vectors,
        //		but we should never call it - thus the assert.

        //  NOLINTNEXTLINE
        SearchablePointerList& operator=(const SearchablePointerList& vectorToCopy) { assert(false); }

        SearchablePointerList& operator=(SearchablePointerList&&) = delete;

        [[nodiscard]] size_t size() const
        {
            if (!cut_over_)
            {
                return (array_.size());
            }

            return (map_->size());
        }

        [[nodiscard]] bool empty() const
        {
            if (!cut_over_)
            {
                return (array_.empty());
            }

            return (map_->empty());
        }

        T* front() const
        {
            if (!cut_over_)
            {
                return (array_.front());
            }

            return (*(map_->begin()));
        }

        iterator begin()
        {
            if (!cut_over_)
            {
                return (iterator(array_.begin()));
            }

            return (iterator(map_->begin()));
        }

        const_iterator begin() const
        {
            if (!cut_over_)
            {
                return (const_iterator(array_.begin()));
            }

            return (const_iterator(map_->begin()));
        }

        iterator end()
        {
            if (!cut_over_)
            {
                return (iterator(array_.end()));
            }

            return (iterator(map_->end()));
        }

        const_iterator end() const
        {
            if (!cut_over_)
            {
                return (const_iterator(array_.end()));
            }

            return (const_iterator(map_->end()));
        }

        T* operator[](unsigned int index)
        {
            if (!cut_over_)
            {
                for (T entry : array_)
                {
                    if (entry->index() == index)
                    {
                        return (entry);
                    }
                }
            }

            EntrySetIterator itrEntry;

            itrEntry = map_->find(index);

            if (itrEntry != map_->end())
            {
                return (itrEntry);
            }

            //	We should never get here, so assert.

            assert(false);
        }

        void insert(T* newValue) { (this->*inserter_)(newValue); }

        void erase(T* index)
        {
            if (!cut_over_)
            {
                for (EntryVectorIterator itrElement = array_.begin(); itrElement != array_.end(); itrElement++)
                {
                    if (*itrElement == index)
                    {
                        array_.erase(itrElement);
                        break;
                    }
                }
            }
            else
            {
                map_->erase(index);
            }
        }

        inline void for_each(std::function<void(T& entry)> action)
        {
            if (!cut_over_)
            {
                for (auto& currentEntry : array_)
                {
                    action(currentEntry);
                }
            }
            else
            {
                for (auto& currentEntry : *map_)
                {
                    action(currentEntry);
                }
            }
        }

       protected:
        typedef void (SearchablePointerList<T, CUTOVER_SIZE>::*InsertFunctionPointer)(T*);

        static std::unique_ptr<DefaultPointerSetPool<T>> default_pool_;

        bool cut_over_;

        InsertFunctionPointer inserter_;

        EntryVector array_;

        EntrySet* map_;

        SetPoolType& set_pool_;

        void insertIntoArray(T* newValue)
        {
            if (array_.size() < CUTOVER_SIZE)
            {
                array_.push_back(newValue);
            }
            else
            {
                map_ = set_pool_.new_set();

                for (T* value : array_)
                {
                    map_->insert(value);
                }

                map_->insert(newValue);

                cut_over_ = true;

                inserter_ = &SearchablePointerList<T, CUTOVER_SIZE>::insertIntoMap;
            }
        }

        void insertIntoMap(T* newValue) { map_->insert(newValue); }

        friend class iterator;
    };

    template <class T, size_t CUTOVER_SIZE>
    std::unique_ptr<DefaultPointerSetPool<T>> SearchablePointerList<T, CUTOVER_SIZE>::default_pool_(
        new DefaultPointerSetPool<T>());

}  // namespace SEFUtility
