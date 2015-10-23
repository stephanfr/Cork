// +-------------------------------------------------------------------------
// | BoundingBox.h
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


#include "Primitives.h"

#include <cfloat>

// NOTE on usage of BBoxes
//  all BBoxes are initialized so that
//      convex(BBox(), bb) == bb
//  for any bb



namespace Cork
{
	namespace Math
	{


		// **************************************************************************
		// *  BBox3 stores 3-dimensional axis aligned bounding boxes
		// **************************************************************************


		const float			two = 2.0;

		#ifdef CORK_SSE
		const __m128			SSEtwo = _mm_load_ps1( &two );
		#endif

		class BBox3D final
		{
		public:

	
			BBox3D::BBox3D()
				: m_minp( FLT_MAX, FLT_MAX, FLT_MAX),
				  m_maxp(-FLT_MAX,-FLT_MAX,-FLT_MAX)
			{}

			 BBox3D( const Vector3D&	minpp,
					 const Vector3D&	maxpp )
				: m_minp(minpp),
				  m_maxp(maxpp)
			{}

			BBox3D(const BBox3D&	bb)
				: m_minp(bb.m_minp),
				  m_maxp(bb.m_maxp)
			{}



			const Vector3D&		minima() const
			{
				return(m_minp);
			}

			const Vector3D&		maxima() const
			{
				return(m_maxp);
			}

			Vector3D			center() const
			{
		#ifdef CORK_SSE
				__m128		result;

				_mm_store_ps( result.m128_f32, _mm_add_ps( (__m128)m_minp, _mm_div_ps( _mm_sub_ps( m_maxp, m_minp ), SSEtwo ) ));

				return( Vector3D( result ));
		#else
				return( m_minp + (( m_maxp - m_minp ) / (NUMERIC_PRECISION)2.0 ));
		#endif
			}


			bool isEmpty() const
			{
				return(( m_maxp[0] < m_minp[0] ) ||
					   ( m_maxp[1] < m_minp[1] ) ||
					   ( m_maxp[2] < m_minp[2] ));
			}


			bool isIn( const Vector3D&	pointToTest ) const
			{
				return(( m_minp[0] <= pointToTest[0] ) &&
					   ( pointToTest[0] <= m_maxp[0] ) &&
					   ( m_minp[1] <= pointToTest[1] ) &&
					   ( pointToTest[1] <= m_maxp[1] ) &&
					   ( m_minp[2] <= pointToTest[2] ) &&
					   ( pointToTest[2] <= m_maxp[2] ));
			}

		#ifdef CORK_SSE
	
			//	SSE2 implementation

			bool	intersects( const BBox3D&			rhs ) const
			{
				return( _mm_movemask_ps( _mm_and_ps( _mm_cmple_ps( m_minp, rhs.m_maxp ), _mm_cmpge_ps( m_maxp, rhs.m_minp ) ) ) == 0x0F );
			}

			inline
			bool	doesNotIntersect( const BBox3D&		rhs ) const
			{
				return( _mm_movemask_ps( _mm_and_ps( _mm_cmple_ps( m_minp, rhs.m_maxp ), _mm_cmpge_ps( m_maxp, rhs.m_minp ) ) ) != 0x0F );
			}

		#else

			inline
			bool intersects(const BBox3D&		rhs) const
			{
				return(( m_minp[0] <= rhs.m_maxp[0] ) &&
					   ( m_maxp[0] >= rhs.m_minp[0] ) &&
					   ( m_minp[1] <= rhs.m_maxp[1] ) &&
					   ( m_maxp[1] >= rhs.m_minp[1] ) &&
					   ( m_minp[2] <= rhs.m_maxp[2] ) &&
					   ( m_maxp[2] >= rhs.m_minp[2] ));
			}

			inline
			bool	doesNotIntersect( const BBox3D&		rhs ) const
			{
				return(( m_minp[0] >= rhs.m_maxp[0] ) ||
					   ( m_maxp[0] <= rhs.m_minp[0] ) ||
					   ( m_minp[1] >= rhs.m_maxp[1] ) ||
					   ( m_maxp[1] <= rhs.m_minp[1] ) ||
					   ( m_minp[2] >= rhs.m_maxp[2] ) ||
					   ( m_maxp[2] <= rhs.m_minp[2] ));
			}

		#endif


			void		scale( const Vector3D&		scaling )
			{
				m_minp = Vector3D( m_minp.x() * scaling.x(), m_minp.y() * scaling.y(), m_minp.z() * scaling.z() );
				m_maxp = Vector3D( m_maxp.x() * scaling.x(), m_maxp.y() * scaling.y(), m_maxp.z() * scaling.z() );
			}



			void		convex(  const BBox3D&		rhs )
			{
		#ifdef CORK_SSE
				_mm_store_ps( (float*)m_minp, _mm_min_ps( m_minp, rhs.m_minp ));
				_mm_store_ps( (float*)m_maxp, _mm_max_ps( m_maxp, rhs.m_maxp ));	
		#else
				m_minp = min( m_minp, rhs.m_minp );
				m_maxp = max( m_maxp, rhs.m_maxp );
		#endif
			}
	

			void		convex( const BBox3D&		rhs,
								BBox3D&				result ) const
			{		
		#ifdef CORK_SSE
				_mm_store_ps( (float*)result.m_minp, _mm_min_ps( m_minp, rhs.m_minp ));
				_mm_store_ps( (float*)result.m_maxp, _mm_max_ps( m_maxp, rhs.m_maxp ));
		#else
				result.m_minp = min( m_minp, rhs.m_minp );
				result.m_maxp = max( m_maxp, rhs.m_maxp );
		#endif
			}


			BBox3D intersection(const BBox3D&		rhs) const
			{
				return( BBox3D( max( m_minp, rhs.m_minp ), min( m_maxp, rhs.m_maxp )));
			}


			Vector3D dim() const 
			{
				return( m_maxp - m_minp );
			}


			NUMERIC_PRECISION	surfaceArea() const
			{
				Vector3D d = dim();
				return( 2 * (d[1] * d[2] + d[0] * d[2] + d[0] * d[1]));
			}


		private :

			Vector3D		m_minp;
			Vector3D		m_maxp;
		};



		inline std::ostream& operator<<(std::ostream &out, const BBox3D &bb)
		{
			return out << "[min" << bb.minima() << ";max" << bb.maxima() << ']';
		}


	}	//	namespace Math
}		//	namespace Cork

