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

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include "cork_defs.hpp"

namespace Cork::Math
{
    template <typename T>
    class Vector2DTemplate final
    {
       private:  // names
        union
        {
            alignas(SIMD_MEMORY_ALIGNMENT) T v[2];

            struct
            {
                T m_x;
                T m_y;
            };

//            struct
//            {
//                T m_s;
//                T m_t;
//            };
        };

        // +---------------------------------
       public:  // constructors
        Vector2DTemplate() {}

        Vector2DTemplate(const T& n0, const T& n1) : m_x(n0), m_y(n1) {}

        // allows implicit conversion based on generics...

        Vector2DTemplate(const Vector2DTemplate& cp) : m_x(cp.m_x), m_y(cp.m_y) {}

        Vector2DTemplate(Vector2DTemplate&& cp) : m_x(std::move(cp.m_x)), m_y(std::move(cp.m_y)) {}

        explicit Vector2DTemplate(const std::array<float, 2>& pair) : m_x(pair[0]), m_y(pair[1]) {}

        explicit Vector2DTemplate(const std::array<double, 2>& pair) : m_x(pair[0]), m_y(pair[1]) {}

        Vector2DTemplate& operator=(const Vector2DTemplate& cp)
        {
            m_x = cp.m_x;
            m_y = cp.m_y;
            return (*this);
        }

        Vector2DTemplate& operator=(Vector2DTemplate&& cp)
        {
            m_x = std::move(cp.m_x);
            m_y = std::move(cp.m_y);
            return (*this);
        }

        [[nodiscard]] T x() const { return (m_x); }
        [[nodiscard]] T y() const { return (m_y); }

        [[nodiscard]] T& operator[](unsigned int i) { return v[i]; }
        [[nodiscard]] T operator[](unsigned int i) const { return v[i]; }

        //
        // destructive arithmetic
        //

        Vector2DTemplate& operator+=(const Vector2DTemplate& rhs)
        {
            m_x += rhs.m_x;
            m_y += rhs.m_y;
            return *this;
        }

        Vector2DTemplate& operator-=(const Vector2DTemplate& rhs)
        {
            m_x -= rhs.m_x;
            m_y -= rhs.m_y;
            return *this;
        }

        Vector2DTemplate& operator*=(T rhs)
        {
            m_x *= rhs;
            m_y *= rhs;
            return *this;
        }

        Vector2DTemplate& operator/=(T rhs)
        {
            m_x /= rhs;
            m_y /= rhs;
            return *this;
        }

        //
        //  Non destructive arithmetic
        //

        [[nodiscard]] Vector2DTemplate<T> operator+(const Vector2DTemplate<T>& rhs) const
        {
            Vector2DTemplate<T> result(*this);
            return result += rhs;
        }

        [[nodiscard]] Vector2DTemplate<T> operator-(const Vector2DTemplate<T>& rhs) const
        {
            Vector2DTemplate<T> result(*this);
            return result -= rhs;
        }

        [[nodiscard]] Vector2DTemplate<T> operator*(T rhs) const
        {
            Vector2DTemplate<T> res(*this);
            return res *= rhs;
        }

        [[nodiscard]] Vector2DTemplate<T> operator/(T rhs) const
        {
            Vector2DTemplate<T> res(*this);
            return (res /= rhs);
        }

        //	Comparison operators

        bool operator==(const Vector2DTemplate<T>& rhs) const { return ((m_x == rhs.m_x) && (m_y == rhs.m_y)); }

        bool operator!=(const Vector2DTemplate<T>& rhs) const { return ((m_x != rhs.m_x) || (m_y != rhs.m_y)); }

        //
        //  Max/Min
        //

        [[nodiscard]] T max() const { return std::max(m_x, m_y); }

        [[nodiscard]] T min() const { return std::min(m_x, m_y); }

        [[nodiscard]] Vector2DTemplate<T> max(const Vector2DTemplate<T>& rhs) const
        {
            return Vector2DTemplate<T>(std::max(m_x, rhs.m_x), std::max(m_y, rhs.m_y));
        }

        [[nodiscard]] Vector2DTemplate<T> min(const Vector2DTemplate<T>& rhs) const
        {
            return Vector2DTemplate<T>(std::min(m_x, rhs.m_x), std::min(m_y, rhs.m_y));
        }

        [[nodiscard]] size_t maxDim() { return (m_x >= m_y) ? 0 : 1; }

        [[nodiscard]] size_t minDim() { return (m_x <= m_y) ? 0 : 1; }

        //
        // Collapsing measurements
        //

        [[nodiscard]] T len_squared() const { return (m_x * m_x) + (m_y * m_y); }

        [[nodiscard]] T len() const { return std::sqrt(len_squared()); }

        //
        // normalization functions
        //

        Vector2DTemplate<T>& normalize() { *this /= len();  return *this; }

        Vector2DTemplate<T> normalize() const { *this /= len(); return *this; }

        Vector2DTemplate<T> normalized() const
        {
            Vector2DTemplate<T> copy(*this);
            return copy.normalize();
        }

        //
        //  Misc
        //

        [[nodiscard]] Vector2DTemplate<T> abs() { return Vector2DTemplate<T>(fabs(m_x), fabs(m_y)); }

        [[nodiscard]] Vector2DTemplate<T> operator-() { return Vector2DTemplate<T>(-m_x, -m_y); }

        [[nodiscard]] T dot(const Vector2DTemplate<T>& rhs) { return (m_x * rhs.m_x) + (m_y * rhs.m_y); }
    };

    template <typename T>
    inline std::ostream& operator<<(std::ostream& out, const Vector2DTemplate<T>& vec)
    {
        return out << '[' << vec.x() << ',' << vec.y() << ']';
    }

    template <typename T>
    [[nodiscard]] inline Vector2DTemplate<T> operator*(const T& lhs, const Vector2DTemplate<T>& rhs)
    {
        Vector2DTemplate<T> result(rhs);
        return result *= lhs;
    }

    //
    //  Determinant
    //

    template <typename T>
    [[nodiscard]] inline T determinant(const Vector2DTemplate<T>& lhs, const Vector2DTemplate<T>& rhs)
    {
        return (lhs.x() * rhs.y()) - (lhs.y() * rhs.x());
    }

}  // namespace Cork::Math
