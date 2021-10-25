// +-------------------------------------------------------------------------
// | bounding_box_3D_template.hpp
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

#include "math/vector_2D_template.hpp"
#include "math/vector_3D_template.hpp"
#include "math/ray_3D_template.hpp"

namespace Cork::Math
{

    static inline constexpr __m256d cnstexpr_mm256_set1_pd(double value)
    {
        return (__m256d){value, value, value, value};
    };

#ifdef __AVX_AVAILABLE__
    static constexpr __m256d AVXtwo = cnstexpr_mm256_set1_pd(2.0);
#endif

    template<typename N, SIMDInstructionSet SIMD = g_SIMD_Level>
    class BBox3DTemplate final
    {
       public:
        BBox3DTemplate() : m_minp(FLT_MAX, FLT_MAX, FLT_MAX), m_maxp(-FLT_MAX, -FLT_MAX, -FLT_MAX) {}

        BBox3DTemplate(const Vector3DTemplate<N>& minpp, const Vector3DTemplate<N>& maxpp) : m_minp(minpp), m_maxp(maxpp) {}

        BBox3DTemplate(const BBox3DTemplate& bb) : m_minp(bb.m_minp), m_maxp(bb.m_maxp) {}

        const Vector3DTemplate<N>& minima() const { return (m_minp); }

        const Vector3DTemplate<N>& maxima() const { return (m_maxp); }

        Vector3DTemplate<N> center() const
        {
#ifdef __AVX_AVAILABLE__
            Vector3DTemplate<N> result;

            _mm256_store_pd((double*)&result.ymm_,
                            _mm256_add_pd(m_minp, _mm256_div_pd(_mm256_sub_pd(m_maxp, m_minp), AVXtwo)));

            return (result);
#else
            return (m_minp + ((m_maxp - m_minp) / (NUMERIC_PRECISION)2.0));
#endif
        }

        bool isEmpty() const { return ((m_maxp[0] < m_minp[0]) || (m_maxp[1] < m_minp[1]) || (m_maxp[2] < m_minp[2])); }

        bool isIn(const Vector3DTemplate<N>& pointToTest) const
        {
            return ((m_minp[0] <= pointToTest[0]) && (pointToTest[0] <= m_maxp[0]) && (m_minp[1] <= pointToTest[1]) &&
                    (pointToTest[1] <= m_maxp[1]) && (m_minp[2] <= pointToTest[2]) && (pointToTest[2] <= m_maxp[2]));
        }

#ifdef __AVX_AVAILABLE__

        //	SSE2 implementation

        bool intersects(const BBox3DTemplate& rhs) const
        {
            return (_mm256_movemask_pd(_mm256_and_pd(_mm256_cmp_pd(m_minp, rhs.m_maxp, _CMP_LE_OQ),
                                                     _mm256_cmp_pd(m_maxp, rhs.m_minp, _CMP_GE_OQ))) == 0x0F);
        }

        inline bool doesNotIntersect(const BBox3DTemplate& rhs) const
        {
            return (_mm256_movemask_pd(_mm256_and_pd(_mm256_cmp_pd(m_minp, rhs.m_maxp, _CMP_LE_OQ),
                                                     _mm256_cmp_pd(m_maxp, rhs.m_minp, _CMP_GE_OQ))) != 0x0F);
        }

#else

        inline bool intersects(const BBox3DTemplate& rhs) const
        {
            return ((m_minp[0] <= rhs.m_maxp[0]) && (m_maxp[0] >= rhs.m_minp[0]) && (m_minp[1] <= rhs.m_maxp[1]) &&
                    (m_maxp[1] >= rhs.m_minp[1]) && (m_minp[2] <= rhs.m_maxp[2]) && (m_maxp[2] >= rhs.m_minp[2]));
        }

        inline bool doesNotIntersect(const BBox3DTemplate& rhs) const
        {
            return ((m_minp[0] >= rhs.m_maxp[0]) || (m_maxp[0] <= rhs.m_minp[0]) || (m_minp[1] >= rhs.m_maxp[1]) ||
                    (m_maxp[1] <= rhs.m_minp[1]) || (m_minp[2] >= rhs.m_maxp[2]) || (m_maxp[2] <= rhs.m_minp[2]));
        }

#endif

        void scale(const Vector3DTemplate<N>& scaling)
        {
            m_minp = Vector3DTemplate<N>(m_minp.x() * scaling.x(), m_minp.y() * scaling.y(), m_minp.z() * scaling.z());
            m_maxp = Vector3DTemplate<N>(m_maxp.x() * scaling.x(), m_maxp.y() * scaling.y(), m_maxp.z() * scaling.z());
        }

        void convex(const BBox3DTemplate& rhs)
        {
#ifdef __AVX_AVAILABLE__
            m_minp = _mm256_min_pd(m_minp, rhs.m_minp);
            m_maxp = _mm256_max_pd(m_maxp, rhs.m_maxp);
#else
            m_minp = min(m_minp, rhs.m_minp);
            m_maxp = max(m_maxp, rhs.m_maxp);
#endif
        }

        void convex(const BBox3DTemplate& rhs, BBox3DTemplate& result) const
        {
#ifdef __AVX_AVAILABLE__
            _mm256_store_pd((double*)&result.m_minp, _mm256_min_pd(m_minp, rhs.m_minp));
            _mm256_store_pd((double*)&result.m_maxp, _mm256_max_pd(m_maxp, rhs.m_maxp));
#else
            result.m_minp = min(m_minp, rhs.m_minp);
            result.m_maxp = max(m_maxp, rhs.m_maxp);
#endif
        }

        BBox3DTemplate intersection(const BBox3DTemplate& rhs) const
        {
            return (BBox3DTemplate(m_minp.max(rhs.m_minp), m_maxp.min(rhs.m_maxp)));
        }

        Vector3DTemplate<N> dim() const { return (m_maxp - m_minp); }

        N surfaceArea() const
        {
            Vector3DTemplate<N> d = dim();
            return (2 * (d[1] * d[2] + d[0] * d[2] + d[0] * d[1]));
        }

        bool intersects(Ray3DWithInverseDirectionTemplate<N>& ray) const
        {
            N txmin, txmax, tymin, tymax, tzmin, tzmax, txymin, txymax;

            const std::array<Vector3DTemplate<N>, 2>& bounds = reinterpret_cast<const std::array<Vector3DTemplate<N>, 2>&>(m_minp);

            txmin = (bounds[ray.signs()[0]].x() - ray.origin().x()) * ray.inverseDirection().x();
            tymax = (bounds[1 - ray.signs()[1]].y() - ray.origin().y()) * ray.inverseDirection().y();

            if (txmin > tymax)
            {
                return false;
            }

            txmax = (bounds[1 - ray.signs()[0]].x() - ray.origin().x()) * ray.inverseDirection().x();
            tymin = (bounds[ray.signs()[1]].y() - ray.origin().y()) * ray.inverseDirection().y();

            if (tymin > txmax)
            {
                return false;
            }

            txymin = std::max(tymin, txmin);
            txymax = std::min(txmax, tymax);

            tzmin = (bounds[ray.signs()[2]].z() - ray.origin().z()) * ray.inverseDirection().z();

            if (tzmin > txymax)
            {
                return (false);
            }

            tzmax = (bounds[1 - ray.signs()[2]].z() - ray.origin().z()) * ray.inverseDirection().z();

            return (txymin <= tzmax);
        }

       private:
        Vector3DTemplate<N> m_minp;
        Vector3DTemplate<N> m_maxp;
    };

    template<typename N>
    inline std::ostream& operator<<(std::ostream& out, const BBox3DTemplate<N>& bb)
    {
        return out << "[min" << bb.minima() << ";max" << bb.maxima() << ']';
    }

//    template<typename N>
//    static const BBox3DTemplate<N> gEmptyBoundingBox(Math::Vector3DTemplate<N>(0.0, 0.0, 0.0), Vector3DTemplate<N>(0.0, 0.0, 0.0));
}