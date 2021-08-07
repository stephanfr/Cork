// +-------------------------------------------------------------------------
// | Vector2DTemplate.h
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


#include <cmath>
#include <cstdlib>
#include <iostream>

#include "../Util/prelude.h"



namespace Cork
{
	namespace Math
	{


		template <typename T>
		class Vector2DTemplate final
		{
		private: // names

			union
			{
				T		v[2];
    
				struct
				{
					T	m_x;
					T	m_y;
				};
        
				struct
				{
					T	m_s;
					T	m_t;
				};
			};

		// +---------------------------------
		public: // constructors

			Vector2DTemplate()
			{}
			
			Vector2DTemplate(const T &n0, const T &n1)
				: m_x(n0),
				  m_y(n1)
			{}
			
			// allows implicit conversion based on generics...

			Vector2DTemplate(const Vector2DTemplate &cp)
				: m_x(cp.m_x),
				  m_y(cp.m_y)
			{}
			
			// build from short array in memory
			explicit
			Vector2DTemplate(const T *ns)
				: m_x(ns[0]),
				  m_y(ns[1])
			{}

		// +---------------------------------

			T		x() const		{ return(m_x); }
			T		y() const		{ return(m_y); }
		// +---------------------------------

			T& operator[](unsigned int i)       { return v[i]; }
			T  operator[](unsigned int i) const { return v[i]; }
		// +---------------------------------

			// destructive arithmetic
			inline
			Vector2DTemplate&	operator+=( const Vector2DTemplate&	rhs )
			{
				m_x += rhs.m_x;
				m_y += rhs.m_y;
				return *this;
			}

			inline
			Vector2DTemplate&	operator-=( const Vector2DTemplate&	rhs )
			{
				m_x -= rhs.m_x;
				m_y -= rhs.m_y;
				return *this;
			}

			inline
			Vector2DTemplate&	operator*=( const T&	rhs )
			{
				m_x *= rhs;
				m_y *= rhs;
				return *this;
			}

			inline
			Vector2DTemplate&	operator/=( const T&	rhs )
			{
				m_x /= rhs;
				m_y /= rhs;
				return *this;
			}


			//	Comparison operators

			bool operator==( const Vector2DTemplate<T>&		rhs )
			{
				return(( m_x == rhs.m_x ) && ( m_y == rhs.m_y ));
			}

			bool operator!=( const Vector2DTemplate<T>&		rhs )
			{
				return(( m_x != rhs.m_x ) || ( m_y != rhs.m_y ));
			}


			//	Non destructive arithmetic

			Vector2DTemplate<T> operator/( const float&		rhs	) const
			{
				Vector2DTemplate<T> res( *this );
				return( res /= rhs );
			}

		};
		// +---------------------------------

		template <typename T>
		inline
		std::ostream& operator<<(std::ostream &out, const Vector2DTemplate<T> &vec)
		{
			return out << '[' << vec.m_x << ',' << vec.m_y << ']';
		}



		// +---------------------------------
		// comparison operators

		/*
		template <typename T>
		inline
		bool operator==(const Vector2DTemplate<T> &lhs, const Vector2DTemplate<T> &rhs)
		{
			return lhs.m_x == rhs.m_x && lhs.m_y == rhs.m_y;
		}
		template <typename T>
		inline
		bool operator!=(const Vector2DTemplate<T> &lhs, const Vector2DTemplate<T> &rhs)
		{
			return lhs.m_x != rhs.m_x || lhs.m_y != rhs.m_y;
		}
		*/

		// +---------------------------------
		// non-destructive arithmetic

		template <typename T>
		inline
		Vector2DTemplate<T> operator+(const Vector2DTemplate<T> &lhs, const Vector2DTemplate<T> &rhs)
		{
		   Vector2DTemplate<T> res(lhs);
			return res += rhs;
		}

		template <typename T>
		inline
		Vector2DTemplate<T> operator-(const Vector2DTemplate<T> &lhs, const Vector2DTemplate<T> &rhs)
		{
		   Vector2DTemplate<T> res(lhs);
			return res -= rhs;
		}

		template <typename T>
		inline
		Vector2DTemplate<T> operator*(const Vector2DTemplate<T> &lhs, const T&	rhs)
		{
		   Vector2DTemplate<T> res(lhs);
			return res *= rhs;
		}

		template <typename T>
		inline
		Vector2DTemplate<T> operator*(const T&	lhs, const Vector2DTemplate<T> &rhs)
		{
		   Vector2DTemplate<T> res(rhs);
			return res *= lhs;
		}

		//template <typename T>
		//inline
		//Vector2DTemplate<T> operator/(const Vector2DTemplate<T> &lhs, const float&	rhs)
		//{
		//   Vector2DTemplate<T> res(lhs);
		//    return res /= rhs;
		//}

		template <typename T>
		inline
		Vector2DTemplate<T> operator-(const Vector2DTemplate<T> &vec)
		{
			return Vector2DTemplate<T>(-vec.m_x, -vec.m_y);
		}

		template <typename T>
		inline
		Vector2DTemplate<T> abs(const Vector2DTemplate<T>&	vec)
		{
			return Vector2DTemplate<T>(fabs(vec.m_x), fabs(vec.m_y));
		}


		// +---------------------------------
		// component-wise max/min

		template <typename T>
		inline
		Vector2DTemplate<T> max(const Vector2DTemplate<T> &lhs, const Vector2DTemplate<T> &rhs)
		{
			return Vector2DTemplate<T>(std::max(lhs.m_x, rhs.m_x),
				std::max(lhs.m_y, rhs.m_y));
		}

		template <typename T>
		inline
		Vector2DTemplate<T> min( const Vector2DTemplate<T>&		lhs, const Vector2DTemplate<T>&		rhs )
		{
			return Vector2DTemplate<T>(std::min(lhs.m_x, rhs.m_x),	std::min(lhs.m_y, rhs.m_y));
		}

		// dimension logic for projections

		template <typename T>
		inline
		unsigned int maxDim(	const Vector2DTemplate<T>&		vec	)
		{
			return (vec.m_x >= vec.m_y) ? 0 : 1;
		}

		template <typename T>
		inline
		unsigned int minDim( const Vector2DTemplate<T>&		vec )
		{
			return (vec.m_x <= vec.m_y) ? 0 : 1;
		}

		template <typename T>
		inline
		T proj(unsigned int dim, const Vector2DTemplate<T>&		vec )
		{
			return (dim == 0) ? vec.m_y : vec.m_x;
		}

		// more sophisticated vector arithmetic

		template <typename T>
		inline
		T		dot( const Vector2DTemplate<T>&		lhs,
					 const Vector2DTemplate<T>&		rhs )
		{
			return lhs.m_x*rhs.m_x + lhs.m_y*rhs.m_y;
		}

		// determinant of 2 vec2s

		template <typename T>
		inline
		T		det( const Vector2DTemplate<T>&		lhs,
					 const Vector2DTemplate<T>&		rhs )
		{
			return( lhs.m_x*rhs.m_y - lhs.m_y*rhs.m_x );
		}



		// collapsing measurements

		template <typename T>
		inline
		T	len2(const Vector2DTemplate<T> &vec)
		{
			return vec.m_x*vec.m_x + vec.m_y*vec.m_y;
		}

		template <typename T>
		inline
		T	len(const Vector2DTemplate<T> &vec)
		{
			return std::sqrt(len2(vec)); 
		}

		template <typename T>
		inline
		T	max(const Vector2DTemplate<T> &vec)
		{
			return std::max(vec.m_x, vec.m_y);
		}

		template <typename T>
		inline
		T	min(const Vector2DTemplate<T> &vec)
		{
			return std::min(vec.m_x, vec.m_y);
		}



		// normalization functions

		template <typename T>
		inline
		void normalize( Vector2DTemplate<T>&					vec )
		{
			vec /= len( vec );
		}

		template <typename T>
		inline
		Vector2DTemplate<T> normalized( const Vector2DTemplate<T>&		vec )
		{
			return( vec / len( vec ));
		}

	}	//	namespace Math
}		//	namespace Cork

