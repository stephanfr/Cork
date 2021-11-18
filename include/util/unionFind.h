
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

#include "Xoshiro256Plus.h"


typedef SEFUtility::RNG::Xoshiro256Plus<g_SIMD_Level> Xoroshiro256Plus;


class IUnionFind
{
public:

	virtual size_t			size() const = 0;

	virtual size_t			find(size_t		i) = 0;
	virtual void			unite(size_t i, size_t j) = 0;

	std::vector<size_t> dump()
	{
		std::vector<size_t> result(size());

		for (size_t i = 0; i < size(); i++)
		{
			result[i] = find(i);
		}

		return(result);
	}
};



//
//	Straightforward Serial Ranked Weighted UnionFind with path compression
//


class RankWeightedSerialUnionFind : public IUnionFind
{
public:

	RankWeightedSerialUnionFind(size_t   N)
		: m_rank(N, 0)
	{
		m_ids.reserve(N);

		for (size_t i = 0; i < N; i++)
		{
			m_ids.emplace_back(i);
		}
	}


	size_t			size() const
	{
		return(m_ids.size());
	}

	size_t find(size_t	i)
	{
		size_t id = i;

		while (m_ids[id] != id)
		{
			id = m_ids[id];
		}

		m_ids[i] = id;					// path compression optimization

		return(id);
	}

	void unite(size_t i, size_t j)
	{
		size_t iid = find(i);
		size_t jid = find(j);

		if (iid == jid) // no need to merge the same tree with itself
		{
			return;
		}

		//	Attempt to rank balance

		if (m_rank[iid] > m_rank[jid])
		{
			m_ids[jid] = iid;
		}
		else if (m_rank[iid] < m_rank[jid])
		{
			m_ids[iid] = jid;
		}
		else
		{ // equal ranks
			m_rank[jid]++;
			m_ids[jid] = iid;
		}
	}

private:

	std::vector<size_t>		m_ids;
	std::vector<size_t>		m_rank;
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

	RandomWeightedSerialUnionFind(size_t   N)
		: m_size(N)
	{
		m_parents = new size_t[m_size];
		m_rank = new size_t[m_size];

		//	The choice of Xororshiro128+ here is all about speed.  We absolutely do not
		//		need the statistical purity it provides - we just need random values to
		//		determine the ranking of entries.

		Xoroshiro256Plus		randomGenerator(N);

		//	Make each element it's own parent to start.  We can use the relaxed memory ordering here
		//		as there should not be any memory read issues - we are just storing values.

		for (size_t i = 0; i < m_size; i++)
		{
			m_parents[i] = i;
			m_rank[i] = randomGenerator.next();
		}
	}

	~RandomWeightedSerialUnionFind()
	{
		delete[] m_parents;
		delete[] m_rank;
	}


	size_t			size() const
	{
		return(m_size);
	}


	size_t find(size_t	i)
	{
		size_t id = i;

		while (true)
		{
			size_t parent = m_parents[id];
			size_t grandParent = m_parents[parent];

			if (id == parent)
			{
				return(parent);
			}

			m_parents[id] = grandParent;
			id = parent;
		}
	}

	void unite(size_t i, size_t j)
	{
		//	The while loop is to handle failures of the CAS operation that updates the parent.

		size_t iid = find(i);
		size_t jid = find(j);

		if (iid == jid) // no need to merge the same tree with itself
		{
			return;
		}

		//	Balance based on the random values

		if (m_rank[iid] < m_rank[jid])
		{
			m_parents[jid] = iid;
		}
		else
		{
			m_parents[iid] = jid;
		}
	}

private:

	size_t				m_size;

	size_t*				m_parents;
	size_t*				m_rank;
};




class RandomWeightedParallelUnionFind : public IUnionFind
{
public:

		RandomWeightedParallelUnionFind(size_t   N)
		: m_size( N )
	{
		m_parents = new std::atomic<size_t>[m_size];
		m_rank = new size_t[m_size];

		//	The choice of Xororshiro256+ here is all about speed.  We absolutely do not
		//		need the statistical purity it provides - we just need random values to
		//		determine the ranking of entries.

		Xoroshiro256Plus		randomGenerator(N);

		//	Make each element it's own parent to start.  We can use the relaxed memory ordering here
		//		as there should not be any memory read issues - we are just storing values.

		for (size_t i = 0; i < m_size; i++)
		{
			m_parents[i] = i;
			m_rank[i] = randomGenerator.next();
		}
	}

	~RandomWeightedParallelUnionFind()
	{
		delete[] m_parents;
		delete[] m_rank;
	}



	size_t			size() const
	{
		return(m_size);
	}

	size_t find(size_t	i)
	{
		size_t id = i;
		size_t initialParent = m_parents[i];

/*
		while( true )
		{
			size_t parent = m_parents[id]._My_val;
			size_t grandParent = m_parents[parent]._My_val;

			if (id == parent)
			{
				return(parent);
			}

			m_parents[id].compare_exchange_weak(parent, grandParent, std::memory_order_relaxed);
			id = parent;
		}
*/

		while (m_parents[id] != id)
		{
			id = m_parents[id];
		}

		m_parents[i].compare_exchange_strong(initialParent, id,std::memory_order_relaxed);

		return(id);
	}

	void unite(size_t i, size_t j)
	{
		//	The while loop is to handle failures of the CAS operation that updates the parent.

		while (true)
		{
			size_t iid = find(i);
			size_t jid = find(j);

			if (iid == jid) // no need to merge the same tree with itself
			{
				return;
			}

			//	Balance based on the random values

			if (m_rank[iid] < m_rank[jid])
			{
				if (m_parents[jid].compare_exchange_strong(j, iid, std::memory_order_relaxed))
				{
					return;
				}
			}
			else
			{
				if (m_parents[iid].compare_exchange_strong(i, jid, std::memory_order_relaxed))
				{
					return;
				}
			}
		}
	}


private:

	size_t							m_size;

	std::atomic<size_t>*			m_parents;
	size_t*							m_rank;
};





class RandomWeightedParallelUnionFind1 : public IUnionFind
{
public:

	RandomWeightedParallelUnionFind1(size_t   N)
		: m_size(N)
	{
		m_parents = new std::atomic<size_t>[m_size];
		m_rank = new size_t[m_size];

		//	The choice of Xororshiro256+ here is all about speed.  We absolutely do not
		//		need the statistical purity it provides - we just need random values to
		//		determine the ranking of entries.

		Xoroshiro256Plus		randomGenerator(N);

		//	Make each element it's own parent to start.  We can use the relaxed memory ordering here
		//		as there should not be any memory read issues - we are just storing values.

		for (size_t i = 0; i < m_size; i++)
		{
			m_parents[i] = i;
			m_rank[i] = randomGenerator.next();
		}
	}

	~RandomWeightedParallelUnionFind1()
	{
		delete[] m_parents;
		delete[] m_rank;
	}



	size_t			size() const
	{
		return(m_size);
	}

	size_t find(size_t	i)
	{
		size_t id = i;

		while (true)
		{
			size_t parent = m_parents[id];
			size_t grandParent = m_parents[parent];

			if (id == parent)
			{
				return(parent);
			}

			m_parents[id] = grandParent;
			id = parent;
		}
	}

	void unite(size_t i, size_t j)
	{
		//	The while loop is to handle failures of the CAS operation that updates the parent.

		while (true)
		{
//			size_t iid = find(i);
//			size_t jid = find(j);

			std::pair<size_t, size_t>	iid = findUnoptimized(i);
			std::pair<size_t, size_t>	jid = findUnoptimized(j);

//			if (iid == jid) // no need to merge the same tree with itself
			if( iid.first == jid.first )
			{
				return;
			}

			//	Balance based on the random values

			if (m_rank[iid.first] < m_rank[jid.first])
			{
				m_parents[i].compare_exchange_weak(iid.second, iid.first, std::memory_order_relaxed);
				m_parents[j].compare_exchange_weak(jid.second, jid.first, std::memory_order_relaxed);

				if (m_parents[jid.first].compare_exchange_strong(j, iid.first, std::memory_order_relaxed))
				{
					return;
				}
			}
			else
			{
				m_parents[i].compare_exchange_weak(iid.second, iid.first, std::memory_order_relaxed);
				m_parents[j].compare_exchange_weak(jid.second, jid.first, std::memory_order_relaxed);

				if (m_parents[iid.first].compare_exchange_strong(i, jid.first, std::memory_order_relaxed))
				{
					return;
				}
			}
		}
	}



private:

	size_t							m_size;

	std::atomic<size_t>*			m_parents;
	size_t*							m_rank;

	inline
	std::pair<size_t, size_t> findUnoptimized(size_t	i)
	{
		size_t id = i;
		size_t initialParent = m_parents[i];
		/*
		while( true )
		{
		size_t parent = m_parents[id].load(std::memory_order_acquire);
		size_t grandParent = m_parents[parent].load(std::memory_order_acquire);

		if (id == parent)
		{
		return(parent);
		}

		m_parents[id].compare_exchange_weak(parent, grandParent);
		id = parent;
		}
		*/

		while (m_parents[id] != id)
		{
			id = m_parents[id];
		}

		//		m_parents[i].compare_exchange_strong(initialParent, id, std::memory_order_relaxed);

		return(std::make_pair(id, initialParent));

		//		return(id);

	}

};
