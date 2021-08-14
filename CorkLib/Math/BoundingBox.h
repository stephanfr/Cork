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

//  All BBoxes are initialized so that
//      convex(BBox(), bb) == bb
//  for any bounding box



namespace Cork
{
	namespace Math
	{


		// **************************************************************************
		// *  BBox3 stores 3-dimensional axis aligned bounding boxes
		// **************************************************************************


		const double		two = 2.0;

		#ifdef __AVX_AVAILABLE__
		const __m256d		AVXtwo = _mm256_load_pd( &two );
		#endif

		class BBox3D final
		{
		public:

	
			BBox3D()
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
		#ifdef __AVX_AVAILABLE__
				Vector3D		result;

				_mm256_store_pd( (double*)&result.m_ymm, _mm256_add_pd( m_minp, _mm256_div_pd( _mm256_sub_pd( m_maxp, m_minp ), AVXtwo ) ));

				return( result );
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

		#ifdef __AVX_AVAILABLE__
	
			//	SSE2 implementation

			bool	intersects( const BBox3D&			rhs ) const
			{
				return( _mm256_movemask_pd( _mm256_and_pd( _mm256_cmp_pd( m_minp, rhs.m_maxp, _CMP_LE_OQ ), _mm256_cmp_pd( m_maxp, rhs.m_minp, _CMP_GE_OQ ) ) ) == 0x0F );
			}

			inline
			bool	doesNotIntersect( const BBox3D&		rhs ) const
			{
				return( _mm256_movemask_pd( _mm256_and_pd( _mm256_cmp_pd( m_minp, rhs.m_maxp, _CMP_LE_OQ ), _mm256_cmp_pd( m_maxp, rhs.m_minp, _CMP_GE_OQ ) ) ) != 0x0F );
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
		#ifdef __AVX_AVAILABLE__
				m_minp = _mm256_min_pd( m_minp, rhs.m_minp );
				m_maxp = _mm256_max_pd( m_maxp, rhs.m_maxp );	
		#else
				m_minp = min( m_minp, rhs.m_minp );
				m_maxp = max( m_maxp, rhs.m_maxp );
		#endif
			}
	

			void		convex( const BBox3D&		rhs,
								BBox3D&				result ) const
			{		
		#ifdef __AVX_AVAILABLE__
				_mm256_store_pd( (double*)&result.m_minp, _mm256_min_pd( m_minp, rhs.m_minp ));
				_mm256_store_pd( (double*)&result.m_maxp, _mm256_max_pd( m_maxp, rhs.m_maxp ));
		#else
				result.m_minp = min( m_minp, rhs.m_minp );
				result.m_maxp = max( m_maxp, rhs.m_maxp );
		#endif
			}


			BBox3D intersection(const BBox3D&		rhs) const
			{
				return( BBox3D( m_minp.max( rhs.m_minp ), m_maxp.min( rhs.m_maxp )));
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


			bool	intersects( Cork::Math::Ray3DWithInverseDirection&				ray ) const
			{
				NUMERIC_PRECISION txmin, txmax, tymin, tymax, tzmin, tzmax, txymin, txymax;

				const std::array<Vector3D,2>&		bounds = reinterpret_cast<const std::array<Vector3D,2>&>(m_minp);

				txmin = ( bounds[ray.signs()[0]].x() - ray.origin().x() ) * ray.inverseDirection().x();
				tymax = ( bounds[1 - ray.signs()[1]].y() - ray.origin().y() ) * ray.inverseDirection().y();

				if( txmin > tymax )
				{
					return false;
				}

				txmax = ( bounds[1 - ray.signs()[0]].x() - ray.origin().x() ) * ray.inverseDirection().x();
				tymin = ( bounds[ray.signs()[1]].y() - ray.origin().y() ) * ray.inverseDirection().y();

				if( tymin > txmax )
				{
					return false;
				}

				txymin = std::max( tymin, txmin );
				txymax = std::min( txmax, tymax );

				tzmin = ( bounds[ray.signs()[2]].z() - ray.origin().z() ) * ray.inverseDirection().z();

				if( tzmin > txymax )
				{
					return( false );
				}

				tzmax = ( bounds[1 - ray.signs()[2]].z() - ray.origin().z() ) * ray.inverseDirection().z();

				return( txymin <= tzmax );
			}



		private :

			Vector3D				m_minp;
			Vector3D				m_maxp;
		};



		inline std::ostream& operator<<(std::ostream &out, const BBox3D &bb)
		{
			return out << "[min" << bb.minima() << ";max" << bb.maxima() << ']';
		}


	}	//	namespace Math
}		//	namespace Cork

