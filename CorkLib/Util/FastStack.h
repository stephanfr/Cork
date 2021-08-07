// +-------------------------------------------------------------------------
// | FastStack.h
// | 
// | Author: Stephan Friedl
// +-------------------------------------------------------------------------
// | COPYRIGHT:
// |    Copyright Stephan Friedl 2015
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


#include <memory>

#include <boost/align.hpp>




template <typename T, size_t INITIAL_SIZE>
class FastStack
{
public :

	FastStack()
		: m_usingInitialStorage( true ),
		  m_storage( (T*)m_initialStorage ),
		  m_storage_size( (sizeof(T) + 16) *(INITIAL_SIZE+4) ),
		  m_size( INITIAL_SIZE + 4 ),
		  m_limit( INITIAL_SIZE ),
		  m_nextEmptyElement( 0 )
	{
		size_t		space = sizeof(m_initialStorage);
		
		assert( boost::alignment::align(__alignof(T), sizeof(T), (void*&)m_storage, space ) != nullptr );
	}

	~FastStack()
	{
		if( !m_usingInitialStorage )
		{
			free( m_storage );
		}
	}


	bool		isEmpty() const
	{
		return( m_nextEmptyElement == 0 );
	}


	void		reset()
	{
		m_nextEmptyElement = 0;
	}

	void		push( const T&		newElement )
	{
		m_storage[m_nextEmptyElement++] = newElement;
	
		if( m_nextEmptyElement >= m_limit )
		{
			growStorage();
		}
	}

	void		push2( const T&		newElement1,
					   const T&		newElement2 )
	{
		m_storage[m_nextEmptyElement++] = newElement1;
		m_storage[m_nextEmptyElement++] = newElement2;
	
		if( m_nextEmptyElement >= m_limit )
		{
			growStorage();
		}
	}


	bool		pop( T&			poppedElement )
	{
		if( m_nextEmptyElement != 0 )
		{
			poppedElement = m_storage[--m_nextEmptyElement];

			return( true );
		}

		return( false );
	}


private :

	unsigned int		m_size;
	unsigned int		m_limit;
	
	T*					m_storage;
	unsigned int		m_storage_size;

	bool				m_usingInitialStorage;
	unsigned char		m_initialStorage[(sizeof(T) + 16) *(INITIAL_SIZE+4)];			//	Should be safely way overallocated 

	unsigned int		m_nextEmptyElement;



	void				growStorage()
	{
		unsigned int	new_storage_size = sizeof(T) * ( m_limit * 2 ) + 2;
		T*		newStorage = (T*)aligned_alloc( new_storage_size, __alignof(T) );

		if( !m_usingInitialStorage )
		{
			memcpy( newStorage, m_storage, m_storage_size );
			free( m_storage );
		}
		else
		{
			memcpy( newStorage, m_storage, m_storage_size );
			m_usingInitialStorage = false;
		}

		m_storage = newStorage;
		m_storage_size = new_storage_size;
		m_limit *= 2;
		m_size = m_limit + 2;
	}

};
