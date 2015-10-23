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


#include "boost\noncopyable.hpp"
#include "boost\intrusive\list.hpp"
#include "boost\intrusive\unordered_set.hpp"
#include "tbb\concurrent_vector.h"




namespace hidden
{
	template <typename T>
	class PointerOnlyListElement : public boost::intrusive::list_base_hook<boost::intrusive::link_mode<boost::intrusive::normal_link>>
	{
	public:
		
		explicit
		PointerOnlyListElement( T*		pointer )
			: m_pointer(pointer)
		{}

		operator		T*()
		{
			return(m_pointer);
		}

		operator		const T*( ) const
		{
			return( m_pointer );
		}

		T* operator ->( )
		{
			return( m_pointer );
		}

		const T* operator ->( ) const
		{
			return( m_pointer );
		}

		T*				pointer()
		{
			return(m_pointer);
		}


		T*			m_pointer;
	};

}



template <typename T>
class ManagedIntrusivePointerList : public boost::noncopyable, protected boost::intrusive::list<hidden::PointerOnlyListElement<T>>
{
private:

	typedef boost::intrusive::list<hidden::PointerOnlyListElement<T>>		BaseType;

public:

	typedef tbb::concurrent_vector<hidden::PointerOnlyListElement<T>>		PoolType;


	explicit
	ManagedIntrusivePointerList(PoolType&		pool)
		: m_pool( pool )
	{}

	ManagedIntrusivePointerList( const ManagedIntrusivePointerList& ) = delete;



	using BaseType::iterator;

	using BaseType::front;
	using BaseType::begin;
	using BaseType::end;

	using BaseType::clear;
	using BaseType::empty;
	using BaseType::erase;
	using BaseType::size;



	void		push_back(T*		pointerToAdd)
	{
		BaseType::push_back(*(m_pool.emplace_back(pointerToAdd)));
	}

protected :

	PoolType&				m_pool;
};





typedef boost::intrusive::list_base_hook<>																	IntrusiveListHook;
typedef boost::intrusive::list_base_hook<boost::intrusive::link_mode<boost::intrusive::normal_link>>		IntrusiveListHookNoDestructorOnElements;



template <typename T>
class ManagedIntrusiveValueList : public boost::noncopyable, protected boost::intrusive::list<T,boost::intrusive::constant_time_size<true>>
{
private:

	typedef boost::intrusive::list<T, boost::intrusive::constant_time_size<true>>		BaseType;

public:

	typedef tbb::concurrent_vector<T>													PoolType;

	explicit
	ManagedIntrusiveValueList(PoolType&		pool)
		: m_pool(pool)
	{}

	using BaseType::begin;
	using BaseType::end;
	using BaseType::size;
	using BaseType::clear;


	template<class... _Valty>
	T*		emplace_back(_Valty&&... _Val)
	{
		T&		newValue = *(m_pool.emplace_back(std::forward<_Valty>(_Val)...));
		
		BaseType::push_back(newValue);

		return( &newValue );
	}



	bool		isCompact() const
	{
		return( m_pool.size() == size() );
	}

	PoolType&	getPool()
	{
		return( m_pool );
	}

	void		FixupList()
	{
		BaseType::clear();

		for( auto itrElement = m_pool.begin(); itrElement != m_pool.end(); itrElement++ )
		{
			BaseType::push_back( *itrElement );
		}
	}


	void		free(T&	valueToFree)
	{
		BaseType::erase(iterator_to(valueToFree));
	}

	void		free( const T&		valueToFree )
	{
		BaseType::erase( iterator_to( valueToFree ) );
	}

	void		free(T*	valueToFree)
	{
		BaseType::erase(iterator_to(*valueToFree));
	}

	void		free( const T*		valueToFree )
	{
		BaseType::erase( iterator_to( *valueToFree ) );
	}


private:

	PoolType&				m_pool;
};





