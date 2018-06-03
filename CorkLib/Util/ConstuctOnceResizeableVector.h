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


#include <boost\iterator\iterator_facade.hpp>

#include "Resettable.h"



namespace SEFUtility
{

	template<class T>
	class ConstructOnceResizeableVector : public Resettable
	{
	public :

		class iterator : public boost::iterator_facade<iterator, T, boost::forward_traversal_tag, T&>
		{

		protected:

			friend class ConstructOnceResizeableVector;


			iterator( T*		vectorIterator,
					  T*		end )
			{
				m_vectorIterator = vectorIterator;
				m_end = end;
			}


			friend class boost::iterator_core_access;

			void increment()
			{
				m_vectorIterator < m_end ? m_vectorIterator++ : m_vectorIterator = m_end;
			}

			bool equal( iterator const& other ) const
			{
				return( m_vectorIterator == other.m_vectorIterator);
			}


			T&		dereference() const
			{
				return( *m_vectorIterator );
			}


			T*			m_vectorIterator;
			
			T*			m_end;
		};

		class const_iterator : public boost::iterator_facade<const_iterator, T, boost::forward_traversal_tag, const T&>
		{
		public :

			const_iterator( const iterator&			vectorIterator)
			{
				m_vectorIterator = vectorIterator.m_vectorIterator;
				m_end = vectorIterator.m_end;
			}

		protected:

			friend class ConstructOnceResizeableVector;


			const_iterator( const T*			vectorIterator,
							const T*			end )
			{
				m_vectorIterator = vectorIterator;
				m_end = end;
			}


			friend class boost::iterator_core_access;

			void increment()
			{
				m_vectorIterator < m_end ? m_vectorIterator++ : m_vectorIterator = m_end;
			}

			bool equal( const_iterator const& other ) const
			{
				return( m_vectorIterator == other.m_vectorIterator);
			}

			const T&		dereference() const
			{
				return( *m_vectorIterator );
			}


			const T*	m_vectorIterator;

			const T*	m_end;
		};



		ConstructOnceResizeableVector( const size_t		initialReservation = 10 )
			: m_size( 0 ),
			  m_elementsArraySize( initialReservation )
		{
			m_elements = new T[initialReservation];
		}

		~ConstructOnceResizeableVector()
		{
			delete [] m_elements;
		}


		void		resize( const size_t		newSize )
		{
			//	Resize invalidates everything and resets all elements.

			if( m_elementsArraySize < newSize )
			{
				delete [] m_elements;

				m_elements = new T[newSize];
				m_elementsArraySize = newSize;
			}
			else
			{
				for (size_t i = 0; i < m_size; i++)
				{
					static_cast<Resettable&>(m_elements[i]).reset();
				}
			}

			m_size = newSize;

			m_end = m_elements + m_size;
			m_constEnd = m_elements + m_size;
		}


		void		reset()
		{
			for( int i = 0; i < m_size; i++ )
			{
				m_elements[i].reset();
			}

			m_size = 0;

			m_end = m_elements;
			m_constEnd = m_elements;
		}



		const size_t		size() const
		{
			return( m_size );
		}

		const size_t		reservedSize() const
		{
			return(m_elementsArraySize);
		}


		T&		operator[]( const size_t		index )
		{
			return( m_elements[index] );
		}

		const T&	operator[]( const size_t	index ) const
		{
			return( m_elements[index] );
		}


		iterator			begin()
		{
			return( iterator( m_elements, m_end ) );
		}

		const_iterator		begin() const
		{
			return( const_iterator( m_elements, m_constEnd ) );
		}

		iterator			end()
		{
			return( iterator(m_end, m_end ) );
		}

		const_iterator		end() const
		{
			return( const_iterator(m_constEnd, m_constEnd ) );
		}


	private :

		size_t						m_size;

		T*							m_end;
		const T*					m_constEnd;

		T*							m_elements;
		size_t						m_elementsArraySize;
	};

}



