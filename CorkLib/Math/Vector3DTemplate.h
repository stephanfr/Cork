// +-------------------------------------------------------------------------
// | Vector3DTemplate.h
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


#ifdef __CORK_AVX__
#include <immintrin.h>
#endif

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <array>

#include "..\util\prelude.h"



namespace Cork
{
	namespace Math
	{

		// **************************************************************************
		// *  Vector3DTemplate stores 3-dimensional vectors using whatever base type you choose
		// **************************************************************************

		template<class N>
		class Vector3DTemplate final
		{
		protected: // names
    
				union
			{
				N m_v[4];

				struct
				{
					N m_x;
					N m_y;
					N m_z;
					N m_spare;
				};
        
#ifdef __CORK_AVX__
				__m256d		m_ymm;
#endif
			};

		// +---------------------------------
		public :
			//	Constructors
    
			Vector3DTemplate()
			: m_spare(0.0)
			{}

			Vector3DTemplate(const N &n0, const N &n1, const N &n2)
				: m_x(n0),
				  m_y(n1),
				  m_z(n2),
				  m_spare(0.0)
			{}
    
			// build from short array in memory

			explicit
			Vector3DTemplate(const N*	ns)
				: m_x( ns[0] ),
				  m_y( ns[1] ),
				  m_z( ns[2] ),
				  m_spare( 0.0 )
			{}


			template <typename T>
			Vector3DTemplate( const std::array<T,3>&	threeTuple )
				: m_x( (NUMERIC_PRECISION)threeTuple[0] ),
				  m_y( (NUMERIC_PRECISION)threeTuple[1] ),
				  m_z( (NUMERIC_PRECISION)threeTuple[2] ),
				  m_spare( 0.0 )
			{}


#ifdef __CORK_AVX__
			// Constructor to convert from type __m256d used in intrinsics:
			Vector3DTemplate( __m256d const & x )
			{
				m_ymm = x;
			}
			// Assignment operator to convert from type __m256d used in intrinsics:
			Vector3DTemplate & operator = ( __m256d const & x )
			{
				m_ymm = x;
				return *this;
			}
			// Type cast operator to convert to __m256d used in intrinsics
			operator __m256d( ) const
			{
				return m_ymm;
			}
			// Member function to load from array (unaligned)
			Vector3DTemplate & load( double const * p )
			{
				ymm = _mm256_loadu_pd( p );
				return *this;
			}
			// Member function to load from array, aligned by 32
			// You may use load_a instead of load if you are certain that p points to an address
			// divisible by 32
			Vector3DTemplate & load_a( double const * p )
			{
				ymm = _mm256_load_pd( p );
				return *this;
			}
#endif
	
			static Vector3DTemplate		randomVector( N min, N max )
			{
				return( Vector3DTemplate( (NUMERIC_PRECISION)drand( min, max ), (NUMERIC_PRECISION)drand( min, max ), (NUMERIC_PRECISION)drand( min, max ) ));
			}


			// +---------------------------------
		public: // discrete components
			N		x() const		{ return(m_x); }
			N		y() const		{ return(m_y); }
			N		z() const		{ return(m_z); }

		// +---------------------------------
		public: // indexing
			N& operator[](uint i)       { return m_v[i]; }
			N  operator[](uint i) const { return m_v[i]; }
		// +---------------------------------
		public: // destructive arithmetic
			inline Vector3DTemplate<N>& operator+=(const Vector3DTemplate<N> &rhs);
			inline Vector3DTemplate<N>& operator-=(const Vector3DTemplate<N> &rhs);
			inline Vector3DTemplate<N>& operator*=(const N &rhs);
			inline Vector3DTemplate<N>& operator/=(const N &rhs);
  
			bool operator <( const Vector3DTemplate<N>&		vertexToCompare ) const
			{
				//	Equality is by x, then y and finally z

				if ( m_x < vertexToCompare.m_x )
				{
					return(true);
				}

				if ( m_x > vertexToCompare.m_x )
				{
					return(false);
				}

				//	X values are equal

				if ( m_y < vertexToCompare.m_y )
				{
					return(true);
				}

				if ( m_y > vertexToCompare.m_y )
				{
					return(false);
				}

				//	X and Y values are equal

				if ( m_z < vertexToCompare.m_z )
				{
					return(true);
				}

				if ( m_z > vertexToCompare.m_z )
				{
					return(false);
				}

				//	The only way we should end up down here is if the two vertices are equal

				return(false);
			}

		};
		// +---------------------------------
		// comparison operators
		template<class N>
		inline bool operator==(const Vector3DTemplate<N> &lhs, const Vector3DTemplate<N> &rhs);
		template<class N>
		inline bool operator!=(const Vector3DTemplate<N> &lhs, const Vector3DTemplate<N> &rhs);
		// +---------------------------------
		// non-destructive arithmetic
		template<class N>
		inline Vector3DTemplate<N> operator+(const Vector3DTemplate<N> &lhs, const Vector3DTemplate<N> &rhs);
		template<class N>
		inline Vector3DTemplate<N> operator-(const Vector3DTemplate<N> &lhs, const Vector3DTemplate<N> &rhs);
		template<class N>
		inline Vector3DTemplate<N> operator*(const Vector3DTemplate<N> &lhs, const N &rhs);
		template<class N>
		inline Vector3DTemplate<N> operator*(const N &lhs, const Vector3DTemplate<N> &rhs);
		template<class N>
		inline Vector3DTemplate<N> operator/(const Vector3DTemplate<N> &lhs, const N &rhs);
		template<class N>
		inline Vector3DTemplate<N> operator-(const Vector3DTemplate<N> &vec);
		template<class N>
		inline Vector3DTemplate<N> abs(const Vector3DTemplate<N> &vec);
		// +---------------------------------
		// component-wise max/min
		template<class N>
		inline Vector3DTemplate<N> max(const Vector3DTemplate<N> &lhs, const Vector3DTemplate<N> &rhs);
		template<class N>
		inline Vector3DTemplate<N> max(const Vector3DTemplate<N> &vec1, const Vector3DTemplate<N> &vec2, const Vector3DTemplate<N> &vec3 );
		template<class N>
		inline Vector3DTemplate<N> min(const Vector3DTemplate<N> &lhs, const Vector3DTemplate<N> &rhs);
		template<class N>
		inline Vector3DTemplate<N> min(const Vector3DTemplate<N> &vec1, const Vector3DTemplate<N> &vec2, const Vector3DTemplate<N> &vec3 );
		// +---------------------------------
		// dimension logic for projections
		template<class N>
		inline uint maxDim(const Vector3DTemplate<N> &vec);
		template<class N>
		inline uint minDim(const Vector3DTemplate<N> &vec);
		template<class N>
		inline Vector2DTemplate<N> proj(uint dim, const Vector3DTemplate<N> &vec);
		// +---------------------------------
		// more sophisticated vector arithmetic
		template<class N>
		inline N dot(const Vector3DTemplate<N> &lhs, const Vector3DTemplate<N> &rhs);
		template<class N>
		inline Vector3DTemplate<N> cross(const Vector3DTemplate<N> &lhs, const Vector3DTemplate<N> &rhs);
		// determinant of 3 Vector3DTemplates
		template<class N>
		inline N det(const Vector3DTemplate<N> &v0, const Vector3DTemplate<N> &v1, const Vector3DTemplate<N> &v2);
		// +---------------------------------
		// collapsing measurements
		template<class N>
		inline N len2(const Vector3DTemplate<N> &vec);
		template<class N>
		inline N len(const Vector3DTemplate<N> &vec);
		template<class N>
		inline N max(const Vector3DTemplate<N> &vec);
		template<class N>
		inline N min(const Vector3DTemplate<N> &vec);
		// +---------------------------------
		// normalization functions
		template<class N>
		inline void normalize(Vector3DTemplate<N> &vec);
		template<class N>
		inline Vector3DTemplate<N> normalized(const Vector3DTemplate<N> &vec);
		// +---------------------------------
		// output/input stream operators
		template<class N>
		inline std::ostream& operator<<(std::ostream &out, const Vector3DTemplate<N> &vec) {
			return out << '[' << vec.m_x << ',' << vec.m_y << ',' << vec.m_z << ']';
		}


		// destructive arithmetic
		template<class N>
		inline Vector3DTemplate<N>& Vector3DTemplate<N>::operator+=(const Vector3DTemplate<N> &rhs) {
			m_x += rhs.m_x;
			m_y += rhs.m_y;
			m_z += rhs.m_z;
			return *this;
		}

		template<class N>
		inline
		Vector3DTemplate<N>& Vector3DTemplate<N>::operator-=(const Vector3DTemplate<N> &rhs)
		{
			m_x -= rhs.m_x;
			m_y -= rhs.m_y;
			m_z -= rhs.m_z;
			return *this;
		}

		template<class N>
		inline
		Vector3DTemplate<N>& Vector3DTemplate<N>::operator*=(const N &rhs)
		{
			m_x *= rhs;
			m_y *= rhs;
			m_z *= rhs;
			return *this;
		}

		template<class N>
		inline
		Vector3DTemplate<N>& Vector3DTemplate<N>::operator/=(const N &rhs)
		{
			m_x /= rhs;
			m_y /= rhs;
			m_z /= rhs;
			return *this;
		}

		// +---------------------------------
		// comparison operators

		template<class N>
		inline bool operator==(const Vector3DTemplate<N> &lhs, const Vector3DTemplate<N> &rhs)
		{
			return lhs.m_x == rhs.m_x && lhs.m_y == rhs.m_y && lhs.m_z == rhs.m_z;
		}

		template<class N>
		inline bool operator!=(const Vector3DTemplate<N> &lhs, const Vector3DTemplate<N> &rhs)
		{
			return lhs.m_x != rhs.m_x || lhs.m_y != rhs.m_y || lhs.m_z != rhs.m_z;
		}

		// +---------------------------------
		// non-destructive arithmetic

		template<class N>
		inline
		Vector3DTemplate<N> operator+(const Vector3DTemplate<N> &lhs, const Vector3DTemplate<N> &rhs)
		{
			Vector3DTemplate<N> res(lhs);
			return res += rhs;
		}

		template<class N>
		inline
		Vector3DTemplate<N> operator-(const Vector3DTemplate<N> &lhs, const Vector3DTemplate<N> &rhs)
		{
			Vector3DTemplate<N> res(lhs);
			return res -= rhs;
		}

		template<class N>
		inline
		Vector3DTemplate<N> operator*(const Vector3DTemplate<N> &lhs, const N &rhs)
		{
			Vector3DTemplate<N> res(lhs);
			return res *= rhs;
		}

		template<class N>
		inline
		Vector3DTemplate<N> operator*(const N &lhs, const Vector3DTemplate<N> &rhs)
		{
			Vector3DTemplate<N> res(rhs);
			return res *= lhs;
		}

		template<class N>
		inline
		Vector3DTemplate<N> operator/(const Vector3DTemplate<N> &lhs, const N &rhs)
		{
			Vector3DTemplate<N> res(lhs);
			return res /= rhs;
		}

		template<class N>
		inline
		Vector3DTemplate<N> operator-(const Vector3DTemplate<N> &vec)
		{
			return Vector3DTemplate<N>(-vec.m_x, -vec.m_y, -vec.m_z);
		}

		template<class N>
		inline
		Vector3DTemplate<N> abs(const Vector3DTemplate<N> &vec)
		{
			return Vector3DTemplate<N>(fabs(vec.m_x), fabs(vec.m_y), fabs(vec.m_z));
		}

		//template<>
		//inline Vector3DTemplate<float> abs(const Vector3DTemplate<float> &vec) {
		//    return Vector3DTemplate<float>(fabs(vec.m_x), fabs(vec.m_y), fabs(vec.m_z));
		//}

		// +---------------------------------
		// component-wise max/min

		template<class N>
		inline
		Vector3DTemplate<N> max(const Vector3DTemplate<N> &lhs, const Vector3DTemplate<N> &rhs)
		{
#ifdef __CORK_AVX__
			return _mm256_max_pd( lhs, rhs );
#else
			return Vector3DTemplate<N>(std::max(lhs.m_x, rhs.m_x),
						   std::max(lhs.m_y, rhs.m_y),
						   std::max(lhs.m_z, rhs.m_z));
#endif
		}

		template<class N>
		inline
		Vector3DTemplate<N> max(const Vector3DTemplate<N>&	vec1, const Vector3DTemplate<N>&	vec2, const Vector3DTemplate<N>&	vec3)
		{
#ifdef __CORK_AVX__
			return _mm256_max_pd( vec1, _mm256_max_pd( vec2, vec3 ));
#else
			return Vector3DTemplate<N>( std::max( vec1.m_x, std::max( vec2.m_x, vec3.m_x )),
										std::max( vec1.m_y, std::max( vec2.m_y, vec3.m_y )),
										std::max( vec1.m_z, std::max( vec2.m_z, vec3.m_z )) );
#endif
		}

		template<class N>
		inline
		Vector3DTemplate<N> min(const Vector3DTemplate<N> &lhs, const Vector3DTemplate<N> &rhs) {
#ifdef __CORK_AVX__
			return _mm256_min_pd( lhs, rhs );
#else
			return Vector3DTemplate<N>(std::min(lhs.m_x, rhs.m_x),
						   std::min(lhs.m_y, rhs.m_y),
						   std::min(lhs.m_z, rhs.m_z));
#endif
		}

		template<class N>
		inline
		Vector3DTemplate<N> min(const Vector3DTemplate<N>&	vec1, const Vector3DTemplate<N>&	vec2, const Vector3DTemplate<N>&	vec3)
		{
#ifdef __CORK_AVX__
			return _mm256_min_pd( vec1, _mm256_min_pd( vec2, vec3 ) );
#else
			return Vector3DTemplate<N>( std::min( vec1.m_x, std::min( vec2.m_x, vec3.m_x )),
										std::min( vec1.m_y, std::min( vec2.m_y, vec3.m_y )),
										std::min( vec1.m_z, std::min( vec2.m_z, vec3.m_z )) );
#endif
		}

		// +---------------------------------
		// dimension logic for projections
		template<class N>
		inline uint maxDim(const Vector3DTemplate<N> &vec) {
			return (vec.m_x >= vec.m_y)? ((vec.m_x >= vec.m_z)? 0 : 2) :
									 ((vec.m_y >= vec.m_z)? 1 : 2);
		}
		template<class N>
		inline uint minDim(const Vector3DTemplate<N> &vec) {
			return (vec.m_x <= vec.m_y)? ((vec.m_x <= vec.m_z)? 0 : 2) :
									 ((vec.m_y <= vec.m_z)? 1 : 2);
		}
		template<class N>
		inline Vector2DTemplate<N> proj(uint dim, const Vector3DTemplate<N> &vec) {
			return Vector2DTemplate<N>( vec.v[(dim+1)%3], vec.v[(dim+2)%3] );
		}
		// +---------------------------------
		// more sophisticated vector arithmetic
		template<class N>
		inline N dot(const Vector3DTemplate<N> &lhs, const Vector3DTemplate<N> &rhs) {
			return lhs.m_x*rhs.m_x + lhs.m_y*rhs.m_y + lhs.m_z*rhs.m_z;
		}
		template<class N>
		inline Vector3DTemplate<N> cross(const Vector3DTemplate<N> &lhs, const Vector3DTemplate<N> &rhs) {
			return Vector3DTemplate<N>(lhs.m_y*rhs.m_z - lhs.m_z*rhs.m_y,
						   lhs.m_z*rhs.m_x - lhs.m_x*rhs.m_z,
						   lhs.m_x*rhs.m_y - lhs.m_y*rhs.m_x);
		}
		// determinant of 3 Vector3DTemplates
		template<class N>
		inline N det(const Vector3DTemplate<N> &v0, const Vector3DTemplate<N> &v1, const Vector3DTemplate<N> &v2) {
			N xy = v0.m_x*v1.m_y - v0.m_y*v1.m_x;
			N xz = v0.m_x*v1.m_z - v0.m_z*v1.m_x;
			N yz = v0.m_y*v1.m_z - v0.m_z*v1.m_y;
			return xy*v2.m_z - xz*v2.m_y + yz*v2.m_x;
		}
		// +---------------------------------
		// collapsing measurements
		template<class N>
		inline N len2(const Vector3DTemplate<N> &vec) {
			return vec.m_x*vec.m_x + vec.m_y*vec.m_y + vec.m_z*vec.m_z;
		}
		template<class N>
		inline N len(const Vector3DTemplate<N> &vec) {
			return std::sqrt(len2(vec));
		}
		template<class N>
		inline N max(const Vector3DTemplate<N> &vec) {
			return std::max(vec.m_x, std::max(vec.m_y, vec.m_z));
		}
		template<class N>
		inline N min(const Vector3DTemplate<N> &vec) {
			return std::min(vec.m_x, std::min(vec.m_y, vec.m_z));
		}
		// +---------------------------------
		// normalization functions
		template<class N>
		inline void normalize(Vector3DTemplate<N> &vec) {
			vec /= len(vec);
		}
		template<class N>
		inline Vector3DTemplate<N> normalized(const Vector3DTemplate<N> &vec) {
			return vec / len(vec);
		}


	}	//	namespace Math
}		//	namespace Cok


