
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

#include <atomic>
#include <tuple>
#include <vector>

#include "Xoshiro256Plus.h"
#include "cork_defs.hpp"

typedef SEFUtility::RNG::Xoshiro256Plus<g_SIMD_Level> Xoroshiro256Plus;

class IUnionFind
{
   public:
    [[nodiscard]] virtual size_t size() const = 0;

    [[nodiscard]] virtual size_t find(size_t i) = 0;
    virtual void unite(size_t i, size_t j) = 0;

    [[nodiscard]] std::vector<size_t> dump()
    {
        std::vector<size_t> result(size());

        for (size_t i = 0; i < size(); i++)
        {
            result[i] = find(i);
        }

        return (result);
    }
};

//
//	Straightforward Serial Ranked Weighted UnionFind with path compression
//

class RankWeightedSerialUnionFind : public IUnionFind
{
   public:
    explicit RankWeightedSerialUnionFind(size_t N) : rank_(N, 0)
    {
        ids_.reserve(N);

        for (size_t i = 0; i < N; i++)
        {
            ids_.emplace_back(i);
        }
    }

    [[nodiscard]] size_t size() const override { return (ids_.size()); }

    [[nodiscard]] size_t find(size_t i) override
    {
        size_t id = i;

        while (ids_[id] != id)
        {
            id = ids_[id];
        }

        ids_[i] = id;  // path compression optimization

        return (id);
    }

    void unite(size_t i, size_t j) override
    {
        size_t iid = find(i);
        size_t jid = find(j);

        if (iid == jid)  // no need to merge the same tree with itself
        {
            return;
        }

        //	Attempt to rank balance

        if (rank_[iid] > rank_[jid])
        {
            ids_[jid] = iid;
        }
        else if (rank_[iid] < rank_[jid])
        {
            ids_[iid] = jid;
        }
        else
        {  // equal ranks
            rank_[jid]++;
            ids_[jid] = iid;
        }
    }

   private:
    std::vector<size_t> ids_;
    std::vector<size_t> rank_;
};

//
//	Serial Union Find implementation based on:
//
//  A Randomized Concurrent Algorithm for Disjoint Set Union
//		Siddhartha V.Jayanti and Robert E.Tarjan
//
//	The key differences in this implementation from traditional UnionFinds are:
//
//		1)	Using random values to determine linking.  This eliminates any changes to the
//				rank values so only the id values change
//		2)	Splitting for path compression
//
//	This implementation is single threaded but fast and efficient.
//

class RandomWeightedSerialUnionFind : public IUnionFind
{
   public:
    RandomWeightedSerialUnionFind() = delete;

    explicit RandomWeightedSerialUnionFind(size_t N) : size_(N)
    {
        parents_ = new size_t[size_];
        rank_ = new size_t[size_];

        //	The choice of Xororshiro128+ here is all about speed.  We absolutely do not
        //		need the statistical purity it provides - we just need random values to
        //		determine the ranking of entries.

        Xoroshiro256Plus randomGenerator(N);

        //	Make each element it's own parent to start.  We can use the relaxed memory ordering here
        //		as there should not be any memory read issues - we are just storing values.

        for (size_t i = 0; i < size_; i++)
        {
            parents_[i] = i;
            rank_[i] = randomGenerator.next();
        }
    }

    RandomWeightedSerialUnionFind(const RandomWeightedSerialUnionFind&) = delete;
    RandomWeightedSerialUnionFind(RandomWeightedSerialUnionFind&&) = delete;

    ~RandomWeightedSerialUnionFind()
    {
        delete[] parents_;
        delete[] rank_;
    }

    RandomWeightedSerialUnionFind& operator=(const RandomWeightedSerialUnionFind&) = delete;
    RandomWeightedSerialUnionFind& operator=(RandomWeightedSerialUnionFind&&) = delete;

    [[nodiscard]] size_t size() const override { return (size_); }

    [[nodiscard]] size_t find(size_t i) override
    {
        size_t id = i;

        while (true)
        {
            size_t parent = parents_[id];
            size_t grandParent = parents_[parent];

            if (id == parent)
            {
                return (parent);
            }

            parents_[id] = grandParent;
            id = parent;
        }
    }

    void unite(size_t i, size_t j) override
    {
        //	The while loop is to handle failures of the CAS operation that updates the parent.

        size_t iid = find(i);
        size_t jid = find(j);

        if (iid == jid)  // no need to merge the same tree with itself
        {
            return;
        }

        //	Balance based on the random values

        if (rank_[iid] < rank_[jid])
        {
            parents_[jid] = iid;
        }
        else
        {
            parents_[iid] = jid;
        }
    }

   private:
    size_t size_;

    size_t* parents_;
    size_t* rank_;
};

class RandomWeightedParallelUnionFind : public IUnionFind
{
   public:
    RandomWeightedParallelUnionFind() = delete;

    explicit RandomWeightedParallelUnionFind(size_t N) : size_(N)
    {
        parents_ = new std::atomic<size_t>[size_];
        rank_ = new size_t[size_];

        //	The choice of Xororshiro256+ here is all about speed.  We absolutely do not
        //		need the statistical purity it provides - we just need random values to
        //		determine the ranking of entries.

        Xoroshiro256Plus randomGenerator(N);

        //	Make each element it's own parent to start.  We can use the relaxed memory ordering here
        //		as there should not be any memory read issues - we are just storing values.

        for (size_t i = 0; i < size_; i++)
        {
            parents_[i] = i;
            rank_[i] = randomGenerator.next();
        }
    }

    RandomWeightedParallelUnionFind(const RandomWeightedParallelUnionFind&) = delete;
    RandomWeightedParallelUnionFind(RandomWeightedParallelUnionFind&&) = delete;

    ~RandomWeightedParallelUnionFind()
    {
        delete[] parents_;
        delete[] rank_;
    }

    RandomWeightedParallelUnionFind& operator=(const RandomWeightedParallelUnionFind&) = delete;
    RandomWeightedParallelUnionFind& operator=(RandomWeightedParallelUnionFind&&) = delete;

    [[nodiscard]] size_t size() const override { return (size_); }

    [[nodiscard]] size_t find(size_t i) override
    {
        size_t id = i;
        size_t initialParent = parents_[i];

        while (parents_[id] != id)
        {
            id = parents_[id];
        }

        parents_[i].compare_exchange_strong(initialParent, id, std::memory_order_relaxed);

        return (id);
    }

    void unite(size_t i, size_t j) override
    {
        //	The while loop is to handle failures of the CAS operation that updates the parent.

        while (true)
        {
            size_t iid = find(i);
            size_t jid = find(j);

            if (iid == jid)  // no need to merge the same tree with itself
            {
                return;
            }

            //	Balance based on the random values

            if (rank_[iid] < rank_[jid])
            {
                if (parents_[jid].compare_exchange_strong(j, iid, std::memory_order_relaxed))
                {
                    return;
                }
            }
            else
            {
                if (parents_[iid].compare_exchange_strong(i, jid, std::memory_order_relaxed))
                {
                    return;
                }
            }
        }
    }

   private:
    size_t size_;

    std::atomic<size_t>* parents_;
    size_t* rank_;
};

class RandomWeightedParallelUnionFind1 : public IUnionFind
{
   public:
    explicit RandomWeightedParallelUnionFind1(size_t N) : size_(N)
    {
        parents_ = new std::atomic<size_t>[size_];
        rank_ = new size_t[size_];

        //	The choice of Xororshiro256+ here is all about speed.  We absolutely do not
        //		need the statistical purity it provides - we just need random values to
        //		determine the ranking of entries.

        Xoroshiro256Plus randomGenerator(N);

        //	Make each element it's own parent to start.  We can use the relaxed memory ordering here
        //		as there should not be any memory read issues - we are just storing values.

        for (size_t i = 0; i < size_; i++)
        {
            parents_[i] = i;
            rank_[i] = randomGenerator.next();
        }
    }

    ~RandomWeightedParallelUnionFind1()
    {
        delete[] parents_;
        delete[] rank_;
    }

    [[nodiscard]] size_t size() const override { return (size_); }

    [[nodiscard]] size_t find(size_t i) override
    {
        size_t id = i;

        while (true)
        {
            size_t parent = parents_[id];
            size_t grandParent = parents_[parent];

            if (id == parent)
            {
                return (parent);
            }

            parents_[id] = grandParent;
            id = parent;
        }
    }

    void unite(size_t i, size_t j) override
    {
        //	The while loop is to handle failures of the CAS operation that updates the parent.

        while (true)
        {
            std::pair<size_t, size_t> iid = findUnoptimized(i);
            std::pair<size_t, size_t> jid = findUnoptimized(j);

            if (iid.first == jid.first)
            {
                return;
            }

            //	Balance based on the random values

            if (rank_[iid.first] < rank_[jid.first])
            {
                parents_[i].compare_exchange_weak(iid.second, iid.first, std::memory_order_relaxed);
                parents_[j].compare_exchange_weak(jid.second, jid.first, std::memory_order_relaxed);

                if (parents_[jid.first].compare_exchange_strong(j, iid.first, std::memory_order_relaxed))
                {
                    return;
                }
            }
            else
            {
                parents_[i].compare_exchange_weak(iid.second, iid.first, std::memory_order_relaxed);
                parents_[j].compare_exchange_weak(jid.second, jid.first, std::memory_order_relaxed);

                if (parents_[iid.first].compare_exchange_strong(i, jid.first, std::memory_order_relaxed))
                {
                    return;
                }
            }
        }
    }

   private:
    size_t size_;

    std::atomic<size_t>* parents_;
    size_t* rank_;

    inline std::pair<size_t, size_t> findUnoptimized(size_t i)
    {
        size_t id = i;
        size_t initialParent = parents_[i];

        while (parents_[id] != id)
        {
            id = parents_[id];
        }

        return (std::make_pair(id, initialParent));
    }
};
