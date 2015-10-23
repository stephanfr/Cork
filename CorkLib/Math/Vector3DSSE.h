// +-------------------------------------------------------------------------
// | Vector3DSSE.h
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


#include <xmmintrin.h>


#include "..\util\prelude.h"

#include "SSERng.h"



namespace Cork
{
	namespace Math
	{

		__declspec(align(16))
		class Vector3DSSE final
		{
		private: // names
    
			union
			{
				float		m_v[4];

				struct
				{
					float m_x;
					float m_y;
					float m_z;
				};
        
				struct
				{
					float m_r;
					float m_g;
					float m_b;
					float m_spare;
				};

				__m128		m_SSEVector;

			};

		public :

			Vector3DSSE()
				: m_spare(0.0)
			{}

			Vector3DSSE( const Vector3DSSE&		cp )
				: m_SSEVector( cp.m_SSEVector )
			{}

			explicit
			Vector3DSSE( const __m128&			SSEValue )
				: m_SSEVector( SSEValue ),
				  m_spare(0.0)
			{}

			Vector3DSSE( const float&		n0,
					     const float&		n1,
					     const float&		n2 )
				: m_x( n0 ),
				  m_y( n1 ),
				  m_z( n2 ),
				  m_spare(0.0)
			{}
    

			// build from short array in memory
			explicit
			Vector3DSSE(const float *ns)
				: m_x(ns[0]),
				  m_y(ns[1]),
				  m_z(ns[2]),
				  m_spare(0.0)
			{}



			static Vector3DSSE		randomVector( float min, float max )
			{
				__m128		result;

				drand_sse( result, min, max );

				result.m128_f32[3] = 0.0f;
		
				return( Vector3DSSE( result ));
			}

			//	Accessors and Mutators

			float		x() const
			{
				return( m_x );
			}
	
			float		y() const
			{
				return(m_y);
			}

			float		z() const
			{
				return(m_z);
			}


			float& operator[]( uint		i )
			{
				return m_v[i];
			}

			float  operator[]( uint		i ) const
			{
				return m_v[i];
			}

	
			//	Conversion operator for SSE __m128 values
	
			operator __m128&()
			{
				return( m_SSEVector );
			}

			operator const __m128&() const
			{
				return( m_SSEVector );
			}


			explicit operator	float*()
			{
				return( m_v );
			}


			//	Destructive arithmetic

			Vector3DSSE& Vector3DSSE::operator+=(const Vector3DSSE &rhs)
			{
				m_x += rhs.m_x;
				m_y += rhs.m_y;
				m_z += rhs.m_z;

				return( *this );
			}

			Vector3DSSE& Vector3DSSE::operator-=(const Vector3DSSE &rhs)
			{
				m_x -= rhs.m_x;
				m_y -= rhs.m_y;
				m_z -= rhs.m_z;

				return( *this );
			}

			Vector3DSSE& Vector3DSSE::operator*=(const float &rhs)
			{
				m_x *= rhs;
				m_y *= rhs;
				m_z *= rhs;

				return( *this );
			}

			Vector3DSSE& Vector3DSSE::operator/=(const float &rhs)
			{
				m_x /= rhs;
				m_y /= rhs;
				m_z /= rhs;
				return *this;
			}



			//	Friend operators and functions

			// comparison operators

			friend bool operator==(const Vector3DSSE &lhs, const Vector3DSSE &rhs);
			friend bool operator!=(const Vector3DSSE &lhs, const Vector3DSSE &rhs);

			// non-destructive arithmetic

			friend Vector3DSSE operator+(const Vector3DSSE &lhs, const Vector3DSSE &rhs);
			friend Vector3DSSE operator-(const Vector3DSSE &lhs, const Vector3DSSE &rhs);
			friend Vector3DSSE operator*(const Vector3DSSE &lhs, const float &rhs);
			friend Vector3DSSE operator*(const float &lhs, const Vector3DSSE &rhs);
			friend Vector3DSSE operator/(const Vector3DSSE &lhs, const float &rhs);
			friend Vector3DSSE operator-(const Vector3DSSE &vec);
			friend Vector3DSSE abs(const Vector3DSSE &vec);

			friend Vector3DSSE max(const Vector3DSSE &lhs, const Vector3DSSE &rhs);
			friend Vector3DSSE max(const Vector3DSSE &vec1, const Vector3DSSE &vec2, const Vector3DSSE &vec3);

			friend Vector3DSSE min(const Vector3DSSE &lhs, const Vector3DSSE &rhs);
			friend Vector3DSSE min(const Vector3DSSE &vec1, const Vector3DSSE &vec2, const Vector3DSSE &vec3);

			friend uint maxDim(const Vector3DSSE &vec);
			friend uint minDim(const Vector3DSSE &vec);

			friend float dot(const Vector3DSSE &lhs, const Vector3DSSE &rhs);
			friend Vector3DSSE cross(const Vector3DSSE &lhs, const Vector3DSSE &rhs);

			friend float det(const Vector3DSSE &v0, const Vector3DSSE &v1, const Vector3DSSE &v2);

			friend float len2(const Vector3DSSE &vec);
			friend float len(const Vector3DSSE &vec);
			friend float max(const Vector3DSSE &vec);
			friend float min(const Vector3DSSE &vec);

			friend void normalize(Vector3DSSE &vec);
			friend Vector3DSSE normalized(const Vector3DSSE &vec);

			friend std::ostream& operator<<(std::ostream &out, const Vector3DSSE &vec);


		};





		// comparison operators

		inline bool operator==( const Vector3DSSE&		lhs,
								const Vector3DSSE&		rhs )
		{
			return(( lhs.m_x == rhs.m_x ) &&
				   ( lhs.m_y == rhs.m_y ) &&
				   ( lhs.m_z == rhs.m_z ));
		}

		inline bool operator!=( const Vector3DSSE&		lhs,
								const Vector3DSSE&		rhs )
		{
			return(( lhs.m_x != rhs.m_x ) ||
				   ( lhs.m_y != rhs.m_y ) ||
				   ( lhs.m_z != rhs.m_z ));
		}


		// non-destructive arithmetic

		inline Vector3DSSE operator+( const Vector3DSSE&		lhs,
								   const Vector3DSSE&		rhs )
		{
			Vector3DSSE res(lhs);

			return( res += rhs );
		}

		inline Vector3DSSE operator-( const Vector3DSSE&		lhs,
								   const Vector3DSSE&		rhs )
		{
			Vector3DSSE res(lhs);
    
			return( res -= rhs );
		}

		inline Vector3DSSE operator*( const Vector3DSSE&		lhs,
								   const float&			rhs)
		{
			Vector3DSSE res(lhs);
    
			return( res *= rhs );
		}

		inline Vector3DSSE operator*( const float&			lhs,
								   const Vector3DSSE&		rhs )
		{
			Vector3DSSE res(rhs);

			return( res *= lhs );
		}

		inline Vector3DSSE operator/( const Vector3DSSE&		lhs,
								   const float&			rhs )
		{
			Vector3DSSE res(lhs);

			return( res /= rhs );
		}

		inline Vector3DSSE operator-(const Vector3DSSE&		vec )
		{
			return( Vector3DSSE( -vec.m_x, -vec.m_y, -vec.m_z ));
		}

		inline Vector3DSSE abs(const Vector3DSSE&				vec )
		{
			return( Vector3DSSE( fabs(vec.m_x), fabs(vec.m_y), fabs(vec.m_z) ));
		}


		// component-wise max/min

		inline Vector3DSSE max( const Vector3DSSE&			lhs,
							 const Vector3DSSE&			rhs )
		{
			return( Vector3DSSE( _mm_max_ps( lhs, rhs ) ) );
		} 

		inline Vector3DSSE max( const Vector3DSSE&			vec1,
							    const Vector3DSSE&			vec2,
							    const Vector3DSSE&			vec3 )
		{
			__m128		result;

			_mm_store_ps( result.m128_f32, _mm_max_ps( vec1, _mm_max_ps( vec2, vec3 ) ));

			return( Vector3DSSE( result ) );
		} 

		inline Vector3DSSE min( const Vector3DSSE&			lhs,
							    const Vector3DSSE&			rhs )
		{
			return( Vector3DSSE( _mm_min_ps( lhs, rhs ) ) );
		}

		inline Vector3DSSE min( const Vector3DSSE&			vec1,
							    const Vector3DSSE&			vec2,
							    const Vector3DSSE&			vec3 )
		{
			__m128		result;

			_mm_store_ps( result.m128_f32, _mm_min_ps( vec1, _mm_min_ps( vec2, vec3 ) ));

			return( Vector3DSSE( result ) );
		} 



		// dimension logic for projections

		inline uint maxDim( const Vector3DSSE&			vec )
		{
			return(( vec.m_x >= vec.m_y ) ? (( vec.m_x >= vec.m_z ) ? 0 : 2 ) : (( vec.m_y >= vec.m_z ) ? 1 : 2 ));
		}

		inline uint minDim( const Vector3DSSE&			vec )
		{
			return(( vec.m_x <= vec.m_y ) ? (( vec.m_x <= vec.m_z ) ? 0 : 2 ) : (( vec.m_y <= vec.m_z ) ? 1 : 2 ));
		}



		// more sophisticated vector arithmetic

		inline float dot( const Vector3DSSE&			lhs,
						  const Vector3DSSE&			rhs )
		{
			return(( lhs.m_x*rhs.m_x ) +
				   ( lhs.m_y*rhs.m_y ) +
				   ( lhs.m_z*rhs.m_z )); 
		}

		inline Vector3DSSE cross( const Vector3DSSE&		lhs,
							   const Vector3DSSE&		rhs )
		{
			return( Vector3DSSE(( lhs.m_y*rhs.m_z - lhs.m_z*rhs.m_y ),
							 ( lhs.m_z*rhs.m_x - lhs.m_x*rhs.m_z ),
							 ( lhs.m_x*rhs.m_y - lhs.m_y*rhs.m_x )));
		}


		// determinant of 3 Vector3DSSEs

		inline float det( const Vector3DSSE&		v0,
						  const Vector3DSSE&		v1,
						  const Vector3DSSE&		v2 )
		{
			float xy = v0.m_x*v1.m_y - v0.m_y*v1.m_x;
			float xz = v0.m_x*v1.m_z - v0.m_z*v1.m_x;
			float yz = v0.m_y*v1.m_z - v0.m_z*v1.m_y;

			return(( xy * v2.m_z ) - ( xz * v2.m_y ) + ( yz * v2.m_x ));
		}

		// collapsing measurements

		inline float len2(const Vector3DSSE &vec)
		{
			return( vec.m_x*vec.m_x + vec.m_y*vec.m_y + vec.m_z*vec.m_z );
		}

		inline float len( const Vector3DSSE&		vec )
		{
			return( std::sqrt( len2( vec )));
		}

		inline float max( const Vector3DSSE&		vec )
		{
			return( std::max( vec.m_x, std::max( vec.m_y, vec.m_z )) );
		}

		inline float min( const Vector3DSSE&		vec )
		{
			return( std::min( vec.m_x, std::min( vec.m_y, vec.m_z )) );
		}


		// normalization functions

		inline void normalize( Vector3DSSE&		vec )
		{
			vec /= len( vec );
		}

		inline Vector3DSSE normalized( const Vector3DSSE&		vec )
		{
			return( vec / len( vec ));
		}


		inline std::ostream& operator<<(std::ostream &out, const Vector3DSSE &vec)
		{
			return out << '[' << vec.m_x << ',' << vec.m_y << ',' << vec.m_z << ']';
		}

	}	//	namespace Math
}		//	namespace Cork



