// +-------------------------------------------------------------------------
// | ray_3D_template.hpp
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


#include "vector_3D_template.hpp"

namespace Cork::Math
{
    template<typename N, SIMDInstructionSet SIMD = g_SIMD_Level>
    class Ray3DTemplate
    {
       public:
        Ray3DTemplate() = default;

        Ray3DTemplate(const Vector3DTemplate<N, SIMD>& point, const Vector3DTemplate<N>& dir) : m_origin(point), m_direction(dir) {}

        Ray3DTemplate(const Ray3DTemplate& ) = default;
        Ray3DTemplate( Ray3DTemplate&& ) noexcept = default;

        ~Ray3DTemplate() = default;

        Ray3DTemplate& operator=( const Ray3DTemplate& ) = default;
        Ray3DTemplate& operator=( Ray3DTemplate&& ) noexcept = default;

        const Vector3DTemplate<N, SIMD>& origin() const { return (m_origin); }

        const Vector3DTemplate<N, SIMD>& direction() const { return (m_direction); }

       private:
        Vector3DTemplate<N, SIMD> m_origin;
        Vector3DTemplate<N, SIMD> m_direction;
    };

    template<typename N, SIMDInstructionSet SIMD = g_SIMD_Level>
    class Ray3DWithInverseDirectionTemplate : public Ray3DTemplate<N, SIMD>
    {
       public:
        Ray3DWithInverseDirectionTemplate() = delete;

        Ray3DWithInverseDirectionTemplate(const Vector3DTemplate<N, SIMD>& point, const Vector3DTemplate<N, SIMD>& dir)
            : Ray3DTemplate<N, SIMD>(point, dir),
              m_inverseDirection(1.0 / dir.x(), 1.0 / dir.y(), 1.0 / dir.z()),
              m_signs({(m_inverseDirection.x() < 0), (m_inverseDirection.y() < 0), (m_inverseDirection.z() < 0)})
        {
        }

        explicit Ray3DWithInverseDirectionTemplate(const Ray3DTemplate<N, SIMD>& cp)
            : Ray3DTemplate<N, SIMD>(cp),
              m_inverseDirection(1.0 / cp.direction().x(), 1.0 / cp.direction().y(), 1.0 / cp.direction().z()),
              m_signs({(m_inverseDirection.x() < 0), (m_inverseDirection.y() < 0), (m_inverseDirection.z() < 0)})
        {
        }

        [[nodiscard]] const Vector3DTemplate<N, SIMD>& inverseDirection() const { return (m_inverseDirection); }

        [[nodiscard]] const std::array<int, 3>& signs() const { return (m_signs); }

       private:
        Vector3DTemplate<N, SIMD> m_inverseDirection;

        std::array<int, 3> m_signs;
    };

    template<typename N, SIMDInstructionSet SIMD>
    inline std::ostream& operator<<(std::ostream& out, const Ray3DTemplate<N, SIMD>& ray)
    {
        return out << '[' << ray.origin() << ';' << ray.direction() << ']';
    }
}   //  namespace Cork::Math
