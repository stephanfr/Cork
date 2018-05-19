// +-------------------------------------------------------------------------
// | unionFind.h
// | 
// | Author: Gilbert Bernstein
// +-------------------------------------------------------------------------
// | COPYRIGHT:
// |    Copyright Gilbert Bernstein 2013
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

#include "prelude.h"

class UnionFind
{
public:

	explicit
	UnionFind( size_t   N )
		: m_rank(N, 0)
	{
		m_ids.reserve(N);

		for ( size_t i = 0; i < N; i++)
		{
			m_ids.emplace_back( i );
		}
    }
    
    size_t find( size_t	i )
	{
        size_t id = i;
        
		while (m_ids[id] != id)
		{
			id = m_ids[id];
		}
        
		m_ids[i] = id; // path compression optimization
        
		return( id );
    }

    size_t unionIds( size_t i, size_t j)
	{
        size_t iid = find(i);
        size_t jid = find(j);
        
		if (iid == jid) // no need to merge the same tree with itself
		{
			return( iid );
		}

        //	Attempt to rank balance

        if(m_rank[iid] > m_rank[jid])
		{
            return( m_ids[jid] = iid );
        }
		else if(m_rank[iid] < m_rank[jid])
		{
            return( m_ids[iid] = jid );
        }
		else
		{ // equal ranks
            m_rank[jid]++;
            return( m_ids[jid] = iid );
        }
    }

    std::vector<size_t> dump()
	{
        std::vector<size_t> result(m_ids.size());
    
		for (size_t i = 0; i < m_ids.size(); i++)
		{
			result[i] = find(i);
		}

		return( result );
    }
    
private:

    std::vector<size_t>		m_ids;
    std::vector<size_t>		m_rank;
};
