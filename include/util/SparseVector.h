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

#include <tbb/concurrent_unordered_map.h>
#include <tbb/concurrent_unordered_set.h>

#include <boost/container/static_vector.hpp>
#include <boost/integer/static_min_max.hpp>
#include <boost/iterator/iterator_facade.hpp>
#include <functional>
#include <set>
#include <unordered_map>

#include "Resettable.h"

namespace SEFUtility
{
    class SparseVectorEntry
    {
       public:
        SparseVectorEntry(size_t index) : m_index(index) {}

        size_t index() const { return (m_index); }

       private:
        size_t m_index;
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

    template <class T, long CUTOVER_SIZE>
    class SparseVector : public Resettable
    {
       private:
        typedef boost::container::static_vector<T, CUTOVER_SIZE> EntryVector;
        typedef typename boost::container::static_vector<T, CUTOVER_SIZE>::iterator EntryVectorIterator;
        typedef typename boost::container::static_vector<T, CUTOVER_SIZE>::const_iterator EntryVectorConstIterator;

        typedef std::unordered_map<size_t, T> EntryMap;
        typedef typename EntryMap::iterator EntryMapIterator;

       public:
        using MapPoolType = UnorderedMapPool<T>;

        class iterator : public boost::iterator_facade<iterator, T*, boost::forward_traversal_tag, T*>
        {
           protected:
            friend class SparseVector;

            iterator(const EntryVectorIterator& vectorIterator) : m_cutOver(false)
            {
                m_vectorIterator = vectorIterator;
            }

            iterator(const EntryMapIterator& setIterator) : m_cutOver(true) { m_setIterator = setIterator; }

            friend class boost::iterator_core_access;

            void increment()
            {
                if (m_cutOver)
                {
                    m_setIterator++;
                }
                else
                {
                    m_vectorIterator++;
                }
            }

            bool equal(iterator const& other) const
            {
                if (m_cutOver)
                {
                    return (m_setIterator == other.m_setIterator);
                }
                else
                {
                    return (m_vectorIterator == other.m_vectorIterator);
                }
            }

            T* dereference() const
            {
                if (m_cutOver)
                {
                    return (&(m_setIterator->second));
                }
                else
                {
                    return (&*m_vectorIterator);
                }
            }

            bool m_cutOver;

            EntryVectorIterator m_vectorIterator;
            EntryMapIterator m_setIterator;
        };

        class const_iterator : public boost::iterator_facade<const_iterator, T*, boost::forward_traversal_tag, const T*>
        {
           protected:
            friend class SparseVector;

            const_iterator(const EntryVectorIterator& vectorIterator) : m_cutOver(false)
            {
                m_vectorIterator = vectorIterator;
            }

            const_iterator(const EntryMapIterator& setIterator) : m_cutOver(true) { m_setIterator = setIterator; }

            friend class boost::iterator_core_access;

            void increment()
            {
                if (m_cutOver)
                {
                    m_setIterator++;
                }
                else
                {
                    m_vectorIterator++;
                }
            }

            bool equal(iterator const& other) const
            {
                if (m_cutOver)
                {
                    return (m_setIterator == other.m_setIterator);
                }
                else
                {
                    return (m_vectorIterator == other.m_vectorIterator);
                }
            }

            T* dereference() const
            {
                if (m_cutOver)
                {
                    return (&(m_setIterator->second));
                }
                else
                {
                    return (m_vectorIterator);
                }
            }

            bool m_cutOver;

            EntryVectorIterator m_vectorIterator;
            EntryMapIterator m_setIterator;
        };

        SparseVector()
            : m_cutover(false),
              m_inserter(&SparseVector<T, CUTOVER_SIZE>::insertIntoArray),
              m_findOrAdd(&SparseVector<T, CUTOVER_SIZE>::findOrAddArray),
              map_pool_(*(default_pool_.get())),
              m_map(NULL)
        {
        }

        SparseVector(MapPoolType& map_pool)
            : m_cutover(false),
              m_inserter(&SparseVector<T, CUTOVER_SIZE>::insertIntoArray),
              m_findOrAdd(&SparseVector<T, CUTOVER_SIZE>::findOrAddArray),
              map_pool_(map_pool),
              m_map(NULL)
        {
        }

        //	Need the copy constructor to keep the compiler quiet about being unable to copy fixed_vectors,
        //		but we should never call it - thus the assert.

        SparseVector(const SparseVector& vectorToCopy) { assert(false); }

        ~SparseVector()
        {
            if (m_map != NULL)
            {
                map_pool_.release_map(m_map);
            }
        }

        //	Need the assignment operator to keep the compiler quiet about being unable to copy fixed_vectors,
        //		but we should never call it - thus the assert.

        SparseVector& operator=(const SparseVector& vectorToCopy) { assert(false); }

        void reset()
        {
            if (m_map != NULL)
            {
                map_pool_.release_map(m_map);
            }

            m_cutover = false;
            m_array.clear();
            m_inserter = &SparseVector<T, CUTOVER_SIZE>::insertIntoArray;
            m_findOrAdd = &SparseVector<T, CUTOVER_SIZE>::findOrAddArray;
            m_map = NULL;
        }

        size_t size() const
        {
            if (!m_cutover)
            {
                return (m_array.size());
            }

            return (m_map->size());
        }

        bool empty() const
        {
            if (!m_cutover)
            {
                return (m_array.size() == 0);
            }

            return (m_map->empty());
        }

        iterator begin()
        {
            if (!m_cutover)
            {
                return (iterator(m_array.begin()));
            }

            return (iterator(m_map->begin()));
        }

        const_iterator begin() const
        {
            if (!m_cutover)
            {
                return (const_iterator(m_array.begin()));
            }

            return (const_iterator(m_map->begin()));
        }

        iterator end()
        {
            if (!m_cutover)
            {
                return (iterator(m_array.end()));
            }

            return (iterator(m_map->end()));
        }

        const_iterator end() const
        {
            if (!m_cutover)
            {
                return (const_iterator(m_array.end()));
            }

            return (const_iterator(m_map->end()));
        }

        T& operator[](size_t index)
        {
            if (!m_cutover)
            {
                for (unsigned int i = 0; i < m_array.size(); i++)
                {
                    if (m_array[i].index() == index)
                    {
                        return (m_array[i]);
                    }
                }

                //	We did not find the entry so there is no choice but assert

                assert(false);
            }

            EntryMapIterator itrEntry;

            itrEntry = m_map->find(index);

            if (itrEntry == m_map->end())
            {
                //	We did not find the entry so there is no choice but assert

                assert(false);
            }

            return (itrEntry->second);
        }

        const T& operator[](size_t index) const
        {
            if (!m_cutover)
            {
                for (unsigned int i = 0; i < m_array.size(); i++)
                {
                    if (m_array[i].index() == index)
                    {
                        return (m_array[i]);
                    }
                }

                //	We did not find the entry so there is no choice but assert

                assert(false);
            }

            EntryMapIterator itrEntry;

            itrEntry = m_map->find(index);

            if (itrEntry == m_map->end())
            {
                //	We did not find the entry so there is no choice but assert

                assert(false);
            }

            return (itrEntry->second);
        }

        T& find_or_add(size_t index) { return ((this->*m_findOrAdd)(index)); }

        inline void for_each(std::function<void(T& entry)> action)
        {
            if (!m_cutover)
            {
                for (unsigned int i = 0; i < m_array.size(); i++)
                {
                    action(m_array[i]);
                }
            }
            else
            {
                for (auto& currentEntry : *m_map)
                {
                    action(currentEntry.second);
                }
            }
        }

        void elements(std::vector<T*>& elementVector)
        {
            if (!m_cutover)
            {
                for (unsigned int i = 0; i < m_array.size(); i++)
                {
                    elementVector.push_back(m_array + i);
                }
            }
            else
            {
                for (auto& currentEntry : *m_map)
                {
                    elementVector.push_back(&(currentEntry.second));
                }
            }
        }

       private:
        static std::unique_ptr<DefaultMapPool<T>> default_pool_;

        typedef T& (SparseVector<T, CUTOVER_SIZE>::*InsertFunctionPointer)(size_t);
        typedef T& (SparseVector<T, CUTOVER_SIZE>::*FindOrAddFunctionPointer)(size_t);

        bool m_cutover;

        InsertFunctionPointer m_inserter;
        FindOrAddFunctionPointer m_findOrAdd;

        EntryVector m_array;

        EntryMap* m_map;

        MapPoolType& map_pool_;

        T& insertIntoArray(size_t index)
        {
            if (m_array.size() < CUTOVER_SIZE)
            {
                m_array.emplace_back(index);

                return (m_array.back());
            }

            //            m_map = new EntryMap(CUTOVER_SIZE * 4);
            m_map = map_pool_.new_map(CUTOVER_SIZE * 4);

            for (unsigned int i = 0; i < m_array.size(); i++)
            {
                m_map->emplace(m_array[i].index(), m_array[i]);
            }

            m_cutover = true;

            m_inserter = &SparseVector<T, CUTOVER_SIZE>::insertIntoMap;
            m_findOrAdd = &SparseVector<T, CUTOVER_SIZE>::findOrAddMap;

            return (m_map->emplace(index, index).first->second);
        }

        T& insertIntoMap(size_t index) { return (m_map->emplace(index, index).first->second); }

        T& findOrAddArray(size_t index)
        {
            for (unsigned int i = 0; i < m_array.size(); i++)
            {
                if (m_array[i].index() == index)
                {
                    return (m_array[i]);
                }
            }

            return ((this->*m_inserter)(index));
        }

        T& findOrAddMap(size_t index)
        {
            EntryMapIterator itrEntry;

            itrEntry = m_map->find(index);

            if (itrEntry != m_map->end())
            {
                return (itrEntry->second);
            }

            return ((this->*m_inserter)(index));
        }
    };

    template <class T, long CUTOVER_SIZE>
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

    template <class T, long CUTOVER_SIZE>
    class SearchablePointerList
    {
       private:
        typedef boost::container::static_vector<T*, CUTOVER_SIZE> EntryVector;
        typedef typename boost::container::static_vector<T*, CUTOVER_SIZE>::iterator EntryVectorIterator;
        typedef typename boost::container::static_vector<T*, CUTOVER_SIZE>::const_iterator EntryVectorConstIterator;

        typedef std::set<T*> EntrySet;
        typedef typename EntrySet::iterator EntrySetIterator;
        typedef typename EntrySet::const_iterator EntrySetConstIterator;

       public:
        using SetPoolType = PointerSetPool<T>;

        class iterator : public boost::iterator_facade<iterator, T*, boost::forward_traversal_tag, T*>
        {
           protected:
            friend class SearchablePointerList;

            iterator(const EntryVectorIterator& vectorIterator) : m_cutOver(false)
            {
                m_vectorIterator = new (m_buffer) EntryVectorIterator(vectorIterator);
            }

            iterator(const EntrySetIterator& setIterator) : m_cutOver(true)
            {
                m_setIterator = new (m_buffer) EntrySetIterator(setIterator);
            }

            friend class boost::iterator_core_access;

            void increment()
            {
                if (m_cutOver)
                {
                    (*m_setIterator)++;
                }
                else
                {
                    (*m_vectorIterator)++;
                }
            }

            bool equal(iterator const& other) const
            {
                if (m_cutOver)
                {
                    return (*m_setIterator == *(other.m_setIterator));
                }
                else
                {
                    return (*m_vectorIterator == *(other.m_vectorIterator));
                }
            }

            T* dereference() const
            {
                if (m_cutOver)
                {
                    return (*(*m_setIterator));
                }
                else
                {
                    return (*(*m_vectorIterator));
                }
            }

            unsigned char
                m_buffer[boost::static_unsigned_max<sizeof(EntryVectorIterator), sizeof(EntrySetIterator)>::value];

            bool m_cutOver;

            union
            {
                EntryVectorIterator* m_vectorIterator;
                EntrySetIterator* m_setIterator;
            };
        };

        class const_iterator : public boost::iterator_facade<const_iterator, T*, boost::forward_traversal_tag, T* const>
        {
           protected:
            friend class SearchablePointerList;

            const_iterator(const EntryVectorConstIterator& vectorIterator) : m_cutOver(false)
            {
                m_vectorIterator = new (m_buffer) EntryVectorConstIterator(vectorIterator);
            }

            const_iterator(const EntrySetConstIterator& setIterator) : m_cutOver(true)
            {
                m_setIterator = new (m_buffer) EntrySetConstIterator(setIterator);
            }

            friend class boost::iterator_core_access;

            void increment()
            {
                if (m_cutOver)
                {
                    (*m_setIterator)++;
                }
                else
                {
                    (*m_vectorIterator)++;
                }
            }

            bool equal(const_iterator const& other) const
            {
                if (m_cutOver)
                {
                    return (*m_setIterator == *(other.m_setIterator));
                }
                else
                {
                    return (*m_vectorIterator == *(other.m_vectorIterator));
                }
            }

            T* dereference() const
            {
                if (m_cutOver)
                {
                    return (*(*m_setIterator));
                }
                else
                {
                    return (*(*m_vectorIterator));
                }
            }

            unsigned char m_buffer[boost::static_unsigned_max<sizeof(EntryVectorConstIterator),
                                                              sizeof(EntrySetConstIterator)>::value];

            bool m_cutOver;

            union
            {
                EntryVectorConstIterator* m_vectorIterator;
                EntrySetConstIterator* m_setIterator;
            };
        };

        SearchablePointerList()
            : m_cutover(false),
              m_inserter(&SearchablePointerList<T, CUTOVER_SIZE>::insertIntoArray),
              m_map(NULL),
              set_pool_(*(default_pool_.get()))
        {
        }

        SearchablePointerList( SetPoolType&      set_pool )
            : m_cutover(false),
              m_inserter(&SearchablePointerList<T, CUTOVER_SIZE>::insertIntoArray),
              m_map(NULL),
              set_pool_(set_pool)
        {
        }

        //	Need the copy constructor to keep the compiler quiet about being unable to copy fixed_vectors,
        //		but we should never call it - thus the assert.

        SearchablePointerList(const SearchablePointerList& vectorToCopy) { assert(false); }

        ~SearchablePointerList()
        {
            if (m_map != NULL)
            {
                set_pool_.release_set( m_map );
            }
        }

        //	Need the assignment operator to keep the compiler quiet about being unable to copy fixed_vectors,
        //		but we should never call it - thus the assert.

        SearchablePointerList& operator=(const SearchablePointerList& vectorToCopy) { assert(false); }

        size_t size() const
        {
            if (!m_cutover)
            {
                return (m_array.size());
            }

            return (m_map->size());
        }

        bool empty() const
        {
            if (!m_cutover)
            {
                return (m_array.empty());
            }

            return (m_map->empty());
        }

        T* front() const
        {
            if (!m_cutover)
            {
                return (m_array.front());
            }

            return (*(m_map->begin()));
        }

        iterator begin()
        {
            if (!m_cutover)
            {
                return (iterator(m_array.begin()));
            }

            return (iterator(m_map->begin()));
        }

        const_iterator begin() const
        {
            if (!m_cutover)
            {
                return (const_iterator(m_array.begin()));
            }

            return (const_iterator(m_map->begin()));
        }

        iterator end()
        {
            if (!m_cutover)
            {
                return (iterator(m_array.end()));
            }

            return (iterator(m_map->end()));
        }

        const_iterator end() const
        {
            if (!m_cutover)
            {
                return (const_iterator(m_array.end()));
            }

            return (const_iterator(m_map->end()));
        }

        T* operator[](unsigned int index)
        {
            if (!m_cutover)
            {
                for (T& entry : m_array)
                {
                    if (entry.index() == index)
                    {
                        return (entry);
                    }
                }
            }

            EntrySetIterator itrEntry;

            itrEntry = m_map->find(index);

            if (itrEntry != m_map->end())
            {
                return (itrEntry);
            }

            //	We should never get here, so assert.

            assert(false);
        }

        void insert(T* newValue) { (this->*m_inserter)(newValue); }

        void erase(T* index)
        {
            if (!m_cutover)
            {
                for (EntryVectorIterator itrElement = m_array.begin(); itrElement != m_array.end(); itrElement++)
                {
                    if (*itrElement == index)
                    {
                        m_array.erase(itrElement);
                        break;
                    }
                }
            }
            else
            {
                m_map->erase(index);
            }
        }

        inline void for_each(std::function<void(T& entry)> action)
        {
            if (!m_cutover)
            {
                for (auto& currentEntry : m_array)
                {
                    action(currentEntry);
                }
            }
            else
            {
                for (auto& currentEntry : *m_map)
                {
                    action(currentEntry);
                }
            }
        }

       protected:
        typedef void (SearchablePointerList<T, CUTOVER_SIZE>::*InsertFunctionPointer)(T*);

        static std::unique_ptr<DefaultPointerSetPool<T>> default_pool_;

        bool m_cutover;

        InsertFunctionPointer m_inserter;

        EntryVector m_array;

        EntrySet* m_map;

        SetPoolType& set_pool_;

        void insertIntoArray(T* newValue)
        {
            if (m_array.size() < CUTOVER_SIZE)
            {
                m_array.push_back(newValue);
            }
            else
            {
                m_map = set_pool_.new_set();

                for (T* value : m_array)
                {
                    m_map->insert(value);
                }

                m_map->insert(newValue);

                m_cutover = true;

                m_inserter = &SearchablePointerList<T, CUTOVER_SIZE>::insertIntoMap;
            }
        }

        void insertIntoMap(T* newValue) { m_map->insert(newValue); }

        friend class iterator;
    };

    template <class T, long CUTOVER_SIZE>
    std::unique_ptr<DefaultPointerSetPool<T>> SearchablePointerList<T, CUTOVER_SIZE>::default_pool_(
        new DefaultPointerSetPool<T>());

}  // namespace SEFUtility
