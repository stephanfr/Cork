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


#include <functional>
#include <unordered_map>
#include <set>

#include <boost\integer\static_min_max.hpp>
#include <boost\iterator\iterator_facade.hpp>
#include <boost\container\static_vector.hpp>

#include <tbb\concurrent_unordered_map.h>
#include <tbb\concurrent_unordered_set.h>




namespace SEFUtility
{
	class SparseVectorEntry
	{
	public :

		SparseVectorEntry( size_t		index )
			: m_index( index )
		{}


		size_t		index() const
		{
			return( m_index );
		}


	private :

		size_t		m_index;
	};



    template<class T,long CUTOVER_SIZE>
	class SparseVector
	{
	private :

		typedef boost::container::static_vector<T, CUTOVER_SIZE>								EntryVector;
		typedef typename boost::container::static_vector<T, CUTOVER_SIZE>::iterator				EntryVectorIterator;
		typedef typename boost::container::static_vector<T, CUTOVER_SIZE>::const_iterator		EntryVectorConstIterator;

		typedef std::unordered_map<size_t, T>													EntryMap;
		typedef typename EntryMap::iterator														EntryMapIterator;

	public :

		class iterator : public boost::iterator_facade<iterator, T*, boost::forward_traversal_tag, T*>
		{

		protected:

			friend class SparseVector;


			iterator( const EntryVectorIterator&			vectorIterator )
				: m_cutOver(false)
			{
				m_vectorIterator = vectorIterator;
			}

			iterator( const EntryMapIterator&		setIterator )
				: m_cutOver(true)
			{
				m_setIterator = setIterator;
			}


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

			bool equal( iterator const& other ) const
			{
				if (m_cutOver)
				{
					return( m_setIterator == other.m_setIterator );
				}
				else
				{
					return( m_vectorIterator == other.m_vectorIterator );
				}
			}

			T*		dereference() const
			{
				if (m_cutOver)
				{
					return( &(m_setIterator->second) );
				}
				else
				{
					return( &*m_vectorIterator );
				}
			}


			bool					m_cutOver;

			EntryVectorIterator		m_vectorIterator;
			EntryMapIterator		m_setIterator;
		};

		class const_iterator : public boost::iterator_facade<const_iterator, T*, boost::forward_traversal_tag, const T*>
		{

		protected:

			friend class SparseVector;


			const_iterator( const EntryVectorIterator&			vectorIterator )
				: m_cutOver(false)
			{
				m_vectorIterator = vectorIterator;
			}

			const_iterator( const EntryMapIterator&		setIterator )
				: m_cutOver(true)
			{
				m_setIterator = setIterator;
			}


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

			bool equal( iterator const& other ) const
			{
				if (m_cutOver)
				{
					return( m_setIterator == other.m_setIterator );
				}
				else
				{
					return( m_vectorIterator == other.m_vectorIterator );
				}
			}

			T*		dereference() const
			{
				if (m_cutOver)
				{
					return( &(m_setIterator->second) );
				}
				else
				{
					return( m_vectorIterator );
				}
			}


			bool					m_cutOver;

			EntryVectorIterator		m_vectorIterator;
			EntryMapIterator		m_setIterator;
		};



		SparseVector()
			: m_cutover( false ),
			  m_inserter( &SparseVector<T,10>::insertIntoArray ),
			  m_map( NULL )
		{}

		//	Need the copy constructor to keep the compiler quiet about being unable to copy fixed_vectors,
		//		but we should never call it - thus the assert.

		SparseVector( const SparseVector&		vectorToCopy )
		{
			assert( false );
		}

		~SparseVector()
		{
			if( m_map != NULL )
			{
				delete m_map;
			}
		}

		//	Need the assignment operator to keep the compiler quiet about being unable to copy fixed_vectors,
		//		but we should never call it - thus the assert.

		SparseVector&			operator=( const SparseVector&		vectorToCopy )
		{
			assert( false );
		}


		unsigned int			size() const
		{
			if( !m_cutover )
			{
				return( m_arraySize );
			}

			return( m_map->size() );
		}

		bool					empty() const
		{
			if( !m_cutover )
			{
				return( m_arraySize == 0 );
			}

			return( m_map->empty() );
		}

		
		iterator			begin()
		{
			if( !m_cutover )
			{
				return( iterator( m_array.begin() ) );
			}

			return( iterator( m_map->begin() ) );
		}

		const_iterator			begin() const
		{
			if (!m_cutover)
			{
				return( const_iterator( m_array.begin() ) );
			}

			return( const_iterator( m_map->begin() ) );
		}

		iterator			end()
		{
			if (!m_cutover)
			{
				return( iterator( m_array.end() ) );
			}

			return( iterator( m_map->end() ) );
		}

		const_iterator			end() const
		{
			if (!m_cutover)
			{
				return( const_iterator( m_array.end() ) );
			}

			return( const_iterator( m_map->end() ) );
		}



		T&		operator[]( size_t		index )
		{
			if( !m_cutover )
			{
				for( unsigned int i = 0; i < m_array.size(); i++ )
				{
					if( m_array[i].index() == index )
					{
						return( m_array[i] );
					}
				}

				//	We did not find the entry so there is no choice but assert	
				
				assert( false );
			}

			EntryMapIterator		itrEntry;

			itrEntry = m_map->find( index );

			if( itrEntry == m_map->end() )
			{
				//	We did not find the entry so there is no choice but assert	
				
				assert( false );
			}

			return( itrEntry->second );
		}



		T&			find_or_add( size_t		index )
		{
			if( !m_cutover )
			{
				for( unsigned int i = 0; i < m_array.size(); i++ )
				{
					if( m_array[i].index() == index )
					{
						return( m_array[i] );
					}
				}
			}
			else
			{
				EntryMapIterator		itrEntry;

				itrEntry = m_map->find( index );

				if( itrEntry != m_map->end() )
				{
					return( itrEntry->second );
				}
			}

			return((this->*m_inserter)( index ));
		}



		void			erase( unsigned int		index )
		{
			if (!m_cutover)
			{
				for ( unsigned int i = 0; i < m_array.size(); i++ )
				{
					if ( m_array[i].index() == index)
					{
						if( i < m_arraySize - 1 )
						{
							m_array[i] = m_array[m_arraySize--];
						}
					}
				}
			}
			else
			{
				itrEntry = m_map->erase( index );
			}
		}




		inline void for_each( std::function<void( T &entry )> action )
		{
			if( !m_cutover )
			{
				for( unsigned int i = 0; i < m_array.size(); i++ )
				{
					action( m_array[i] );
				}
			}
			else
			{
				for( auto& currentEntry : *m_map )
				{
					action( currentEntry.second );
				}
			}
		}


		void		elements( std::vector<T*>&		elementVector )
		{
			if( !m_cutover )
			{
				for( unsigned int i = 0; i < m_array.size(); i++ )
				{
					elementVector.push_back( m_array + i );
				}
			}
			else
			{
				for( auto& currentEntry : *m_map )
				{
					elementVector.push_back( &(currentEntry.second) );
				}
			}
		}


	private :

		typedef T& (SparseVector<T,10>::*InsertFunctionPointer)( size_t );	


		bool						m_cutover;
		InsertFunctionPointer		m_inserter;

		EntryVector					m_array;

		EntryMap*					m_map;


		T&						insertIntoArray( size_t	index )
		{
			if( m_array.size() < CUTOVER_SIZE )
			{
				m_array.emplace_back( index );
				
				return( m_array.back() );
			}

			m_map = new EntryMap( CUTOVER_SIZE * 4 );
				
			for( unsigned int i = 0; i < m_array.size(); i++ )
			{
				m_map->emplace( m_array[i].index(), m_array[i] );
			}

			m_cutover = true;
			m_inserter = &SparseVector<T,CUTOVER_SIZE>::insertIntoMap;

			return( m_map->emplace( index, index ).first->second );
		}

		T&						insertIntoMap( size_t	index )
		{
			return( m_map->emplace( index, index ).first->second );
		}

	};



	

	template<class T,long CUTOVER_SIZE>
	class SearchablePointerList
	{
	private:

		typedef boost::container::static_vector<T*, CUTOVER_SIZE>									EntryVector;
		typedef typename boost::container::static_vector<T*, CUTOVER_SIZE>::iterator				EntryVectorIterator;
		typedef typename boost::container::static_vector<T*, CUTOVER_SIZE>::const_iterator			EntryVectorConstIterator;

		typedef std::set<T*>																		EntrySet;
		typedef typename EntrySet::iterator															EntrySetIterator;
		typedef typename EntrySet::const_iterator													EntrySetConstIterator;


	public:


		class iterator : public boost::iterator_facade<iterator, T*, boost::forward_traversal_tag, T*>
		{

		protected:

			friend class SearchablePointerList;


			iterator( const EntryVectorIterator&	vectorIterator )
				: m_cutOver(false)
			{
				m_vectorIterator = new( m_buffer )EntryVectorIterator( vectorIterator );
			}

			iterator( const EntrySetIterator&		setIterator )
				: m_cutOver(true)
			{
				m_setIterator = new( m_buffer )EntrySetIterator( setIterator );
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

			bool equal( iterator const& other ) const
			{
				if (m_cutOver)
				{
					return( *m_setIterator == *(other.m_setIterator) );
				}
				else
				{
					return( *m_vectorIterator == *(other.m_vectorIterator) );
				}
			}

			T*		dereference() const
			{
				if (m_cutOver)
				{
					return( *(*m_setIterator) );
				}
				else
				{
					return( *(*m_vectorIterator ));
				}
			}


			unsigned char		m_buffer[boost::static_unsigned_max< sizeof( EntryVectorIterator ), sizeof( EntrySetIterator ) >::value];

			bool				m_cutOver;

			union
			{
				EntryVectorIterator*		m_vectorIterator;
				EntrySetIterator*			m_setIterator;
			};
		};


		class const_iterator : public boost::iterator_facade<const_iterator, T*, boost::forward_traversal_tag, T* const>
		{

		protected:

			friend class SearchablePointerList;


			const_iterator( const EntryVectorConstIterator&	vectorIterator )
				: m_cutOver( false )
			{
				m_vectorIterator = new(m_buffer)EntryVectorConstIterator( vectorIterator );
			}

			const_iterator( const EntrySetConstIterator&		setIterator )
				: m_cutOver( true )
			{
				m_setIterator = new(m_buffer)EntrySetConstIterator( setIterator );
			}


			friend class boost::iterator_core_access;

			void increment()
			{
				if (m_cutOver)
				{
					( *m_setIterator )++;
				}
				else
				{
					( *m_vectorIterator )++;
				}
			}

			bool equal( const_iterator const& other ) const
			{
				if (m_cutOver)
				{
					return( *m_setIterator == *( other.m_setIterator ) );
				}
				else
				{
					return( *m_vectorIterator == *( other.m_vectorIterator ) );
				}
			}

			T*		dereference() const
			{
				if (m_cutOver)
				{
					return( *( *m_setIterator ) );
				}
				else
				{
					return( *( *m_vectorIterator ) );
				}
			}


			unsigned char		m_buffer[boost::static_unsigned_max< sizeof( EntryVectorConstIterator ), sizeof( EntrySetConstIterator ) >::value];

			bool				m_cutOver;

			union
			{
				EntryVectorConstIterator*		m_vectorIterator;
				EntrySetConstIterator*			m_setIterator;
			};
		};






		SearchablePointerList()
			: m_cutover( false ),
			  m_inserter( &SearchablePointerList<T,10>::insertIntoArray ),
			  m_map( NULL )
		{}

		//	Need the copy constructor to keep the compiler quiet about being unable to copy fixed_vectors,
		//		but we should never call it - thus the assert.

		SearchablePointerList( const SearchablePointerList&		vectorToCopy )
		{
			assert( false );
		}

		~SearchablePointerList()
		{
			if (m_map != NULL)
			{
				delete m_map;
			}
		}

		//	Need the assignment operator to keep the compiler quiet about being unable to copy fixed_vectors,
		//		but we should never call it - thus the assert.

		SearchablePointerList&		operator=( const SearchablePointerList&		vectorToCopy )
		{
			assert( false );
		}



		size_t					size() const
		{
			if( !m_cutover )
			{
				return( m_array.size() );
			}

			return( m_map->size() );
		}

		bool					empty() const
		{
			if( !m_cutover )
			{
				return( m_array.empty() );
			}

			return( m_map->empty() );
		}

		T*						front() const
		{
			if( !m_cutover )
			{
				return( m_array.front() );
			}

			return( *(m_map->begin()) );
		}



		iterator			begin()
		{
			if( !m_cutover )
			{
				return( iterator( m_array.begin() ) );
			}

			return( iterator( m_map->begin() ) );
		}

		const_iterator			begin() const
		{
			if (!m_cutover)
			{
				return( const_iterator( m_array.begin() ) );
			}

			return( const_iterator( m_map->begin() ) );
		}

		iterator			end()
		{
			if (!m_cutover)
			{
				return( iterator( m_array.end() ) );
			}

			return( iterator( m_map->end() ) );
		}

		const_iterator			end() const
		{
			if (!m_cutover)
			{
				return( const_iterator( m_array.end() ) );
			}

			return( const_iterator( m_map->end() ) );
		}

		


		T*		operator[]( unsigned int		index )
		{
			if (!m_cutover)
			{
				for (T& entry : m_array)
				{
					if (entry.index() == index)
					{
						return( entry );
					}
				}
			}

			EntryMapIterator		itrEntry;

			itrEntry = m_map->find( index );

			if (itrEntry != m_map->end())
			{
				return( itrEntry );
			}

			//	We should never get here, so assert.

			assert( false );
		}


		void			insert( T*		newValue )
		{
			(this->*m_inserter)( newValue );
/*
			if (!m_cutover)
			{
				if (m_array.size() < CUTOVER_SIZE)
				{
					m_array.push_back( newValue );
				}
				else
				{
					m_map = new EntrySet();

					for (T* value : m_array)
					{
						m_map->insert( value );
					}

					m_map->insert( newValue );

					m_cutover = true;
				}
			}
			else
			{
				m_map->insert( newValue );
			}
*/
		}



		void			erase( T*		index )
		{
			if (!m_cutover)
			{
				for (EntryVectorIterator itrElement = m_array.begin(); itrElement != m_array.end(); itrElement++)
				{
					if (*itrElement == index)
					{
						m_array.erase( itrElement );
						break;
					}
				}
			}
			else
			{
				m_map->erase( index );
			}
		}




		inline void for_each( std::function<void( T &entry )> action )
		{
			if (!m_cutover)
			{
				for (auto& currentEntry : m_array)
				{
					action( currentEntry );
				}
			}
			else
			{
				for (auto& currentEntry : *m_map)
				{
					action( currentEntry );
				}
			}
		}


	protected:

		typedef void (SearchablePointerList<T,10>::*InsertFunctionPointer)( T* );	


		bool						m_cutover;

		InsertFunctionPointer		m_inserter;

		EntryVector					m_array;

		EntrySet*					m_map;




		void			insertIntoArray( T*		newValue )
		{
			if (m_array.size() < CUTOVER_SIZE)
			{
				m_array.push_back( newValue );
			}
			else
			{
				m_map = new EntrySet();

				for (T* value : m_array)
				{
					m_map->insert( value );
				}

				m_map->insert( newValue );

				m_cutover = true;

				m_inserter = &SearchablePointerList<T,CUTOVER_SIZE>::insertIntoMap;
			}
		}

		void			insertIntoMap( T*	newValue )
		{
			m_map->insert( newValue );
		}


		friend class iterator;
	};





}	//	namespace SEFUtility

