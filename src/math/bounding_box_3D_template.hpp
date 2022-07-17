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

#include "ray_3D_template.hpp"
#include "vector_2D_template.hpp"
#include "vector_3D_template.hpp"

namespace Cork::Math
{
    static inline constexpr __m256d cnstexpr_mm256_set1_pd(double value)
    {
        return (__m256d){value, value, value, value};
    };

#ifdef __AVX_AVAILABLE__
    static constexpr __m256d AVXtwo = cnstexpr_mm256_set1_pd(2.0);
#endif

    template <typename N, SIMDInstructionSet SIMD = g_SIMD_Level>
    class BBox3DTemplate
    {
       public:
        BBox3DTemplate() : m_minp(FLT_MAX, FLT_MAX, FLT_MAX), m_maxp(-FLT_MAX, -FLT_MAX, -FLT_MAX) {}

        BBox3DTemplate(const Vector3DTemplate<N, SIMD>& minpp, const Vector3DTemplate<N, SIMD>& maxpp)
            : m_minp(minpp), m_maxp(maxpp)
        {
        }

        BBox3DTemplate(const BBox3DTemplate& bb) : m_minp(bb.m_minp), m_maxp(bb.m_maxp) {}
        BBox3DTemplate( BBox3DTemplate&& bb) noexcept : m_minp(std::move(bb.m_minp)), m_maxp(std::move(bb.m_maxp)) {}

        ~BBox3DTemplate() = default;

        BBox3DTemplate& operator=( const BBox3DTemplate& bb )
        {
            m_minp = bb.m_minp;
            m_maxp = bb.m_maxp;

            return *this;
        }

        BBox3DTemplate& operator=( BBox3DTemplate&& bb ) noexcept
        {
            m_minp = std::move( bb.m_minp );
            m_maxp = std::move( bb.m_maxp );

            return *this;
        }

        const Vector3DTemplate<N, SIMD>& minima() const { return (m_minp); }

        const Vector3DTemplate<N, SIMD>& maxima() const { return (m_maxp); }

        Vector3DTemplate<N, SIMD> center() const
        {
            if constexpr (SIMD >= SIMDInstructionSet::AVX)
            {
                Vector3DTemplate<N, SIMD> result;

                _mm256_store_pd((double*)&result.ymm_,
                                _mm256_add_pd(m_minp, _mm256_div_pd(_mm256_sub_pd(m_maxp, m_minp), AVXtwo)));

                return (result);
            }
            else
            {
                return (m_minp + ((m_maxp - m_minp) / (NUMERIC_PRECISION)2.0));     //  NOLINT(cppcoreguidelines-avoid-magic-numbers)
            }
        }

        [[nodiscard]] bool isEmpty() const { return ((m_maxp[0] < m_minp[0]) || (m_maxp[1] < m_minp[1]) || (m_maxp[2] < m_minp[2])); }

        [[nodiscard]] bool isIn(const Vector3DTemplate<N, SIMD>& pointToTest) const
        {
            return ((m_minp[0] <= pointToTest[0]) && (pointToTest[0] <= m_maxp[0]) && (m_minp[1] <= pointToTest[1]) &&
                    (pointToTest[1] <= m_maxp[1]) && (m_minp[2] <= pointToTest[2]) && (pointToTest[2] <= m_maxp[2]));
        }

        [[nodiscard]] bool contains(const BBox3DTemplate& rhs) const
        {
            return isIn( rhs.m_minp ) && isIn( rhs.m_maxp );
        }

        [[nodiscard]] bool intersects(const BBox3DTemplate& rhs) const
        {
            if constexpr (SIMD >= SIMDInstructionSet::AVX)
            {
                return (_mm256_movemask_pd(_mm256_and_pd(_mm256_cmp_pd(m_minp, rhs.m_maxp, _CMP_LE_OQ),
                                                         _mm256_cmp_pd(m_maxp, rhs.m_minp, _CMP_GE_OQ))) == 0x0F);     //  NOLINT(cppcoreguidelines-avoid-magic-numbers)
            }
            else
            {
                return ((m_minp[0] <= rhs.m_maxp[0]) && (m_maxp[0] >= rhs.m_minp[0]) && (m_minp[1] <= rhs.m_maxp[1]) &&
                        (m_maxp[1] >= rhs.m_minp[1]) && (m_minp[2] <= rhs.m_maxp[2]) && (m_maxp[2] >= rhs.m_minp[2]));
            }
        }

        [[nodiscard]] bool doesNotIntersect(const BBox3DTemplate& rhs) const
        {
            if constexpr (SIMD >= SIMDInstructionSet::AVX)
            {
                return (_mm256_movemask_pd(_mm256_and_pd(_mm256_cmp_pd(m_minp, rhs.m_maxp, _CMP_LE_OQ),
                                                         _mm256_cmp_pd(m_maxp, rhs.m_minp, _CMP_GE_OQ))) != 0x0F);     //  NOLINT(cppcoreguidelines-avoid-magic-numbers)
            }
            else
            {
                return ((m_minp[0] >= rhs.m_maxp[0]) || (m_maxp[0] <= rhs.m_minp[0]) || (m_minp[1] >= rhs.m_maxp[1]) ||
                        (m_maxp[1] <= rhs.m_minp[1]) || (m_minp[2] >= rhs.m_maxp[2]) || (m_maxp[2] <= rhs.m_minp[2]));
            }
        }

        void scale(const Vector3DTemplate<N, SIMD>& scaling)
        {
            m_minp = Vector3DTemplate<N>(m_minp.x() * scaling.x(), m_minp.y() * scaling.y(), m_minp.z() * scaling.z());
            m_maxp = Vector3DTemplate<N>(m_maxp.x() * scaling.x(), m_maxp.y() * scaling.y(), m_maxp.z() * scaling.z());
        }

        //  Expand the bounding box if the new point is outside

        void convex(const Vector3DTemplate<N, SIMD>& rhs)
        {
            if constexpr (SIMD >= SIMDInstructionSet::AVX)
            {
                m_minp = _mm256_min_pd(m_minp, rhs);
                m_maxp = _mm256_max_pd(m_maxp, rhs);
            }
            else
            {
                m_minp = m_minp.min(rhs);
                m_maxp = m_maxp.min(rhs);
            }
        }

        void convex(const BBox3DTemplate& rhs)
        {
            if constexpr (SIMD >= SIMDInstructionSet::AVX)
            {
                m_minp = _mm256_min_pd(m_minp, rhs.m_minp);
                m_maxp = _mm256_max_pd(m_maxp, rhs.m_maxp);
            }
            else
            {
                m_minp = m_minp.min(rhs.m_minp);
                m_maxp = m_maxp.min(rhs.m_maxp);
            }
        }

        void convex(const BBox3DTemplate& rhs, BBox3DTemplate& result) const
        {
            if constexpr (SIMD >= SIMDInstructionSet::AVX)
            {
                _mm256_store_pd((double*)&result.m_minp, _mm256_min_pd(m_minp, rhs.m_minp));
                _mm256_store_pd((double*)&result.m_maxp, _mm256_max_pd(m_maxp, rhs.m_maxp));
            }
            else
            {
                result.m_minp = m_minp.min(rhs.m_minp);
                result.m_maxp = m_maxp.max(rhs.m_maxp);
            }
        }

        [[nodiscard]] BBox3DTemplate intersection(const BBox3DTemplate& rhs) const
        {
            return (BBox3DTemplate(m_minp.max(rhs.m_minp), m_maxp.min(rhs.m_maxp)));
        }

        [[nodiscard]] Vector3DTemplate<N, SIMD> dim() const { return (m_maxp - m_minp); }

        [[nodiscard]] N surfaceArea() const
        {
            auto d = dim();
            return (2 * (d[1] * d[2] + d[0] * d[2] + d[0] * d[1]));
        }

        [[nodiscard]] bool intersects(Ray3DWithInverseDirectionTemplate<N>& ray) const
        {
            N txmin;
            N txmax;
            N tymin;
            N tymax;
            N tzmin;
            N tzmax;
            N txymin;
            N txymax;

            const auto& bounds = reinterpret_cast<const std::array<Vector3DTemplate<N, SIMD>, 2>&>(m_minp);

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
        Vector3DTemplate<N, SIMD> m_minp;
        Vector3DTemplate<N, SIMD> m_maxp;
    };

    template <typename N, SIMDInstructionSet SIMD>
    inline std::ostream& operator<<(std::ostream& out, const BBox3DTemplate<N, SIMD>& bb)
    {
        return out << "[min" << bb.minima() << ";max" << bb.maxima() << ']';
    }

    //    template<typename N>
    //    static const BBox3DTemplate<N> gEmptyBoundingBox(Math::Vector3DTemplate<N>(0.0, 0.0, 0.0),
    //    Vector3DTemplate<N>(0.0, 0.0, 0.0));
}  // namespace Cork::Math