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

#include <immintrin.h>

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include "../CorkDefs.h"
#include "Vector2DTemplate.h"

#ifdef __AVX_AVAILABLE__
#include "SSERng.h"
#endif

namespace Cork::Math
{
    template <class N, SIMDInstructionSet SIMD = g_SIMD_Level>
    class alignas(32) Vector3DTemplate final
    {
       protected:  // names
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

#ifdef __AVX_AVAILABLE__
            __m256d m_ymm;
#endif
        };

        friend class BBox3D;

       public:
        //	Constructors

        Vector3DTemplate() : m_spare(0.0) {}

        Vector3DTemplate(const N &n0, const N &n1, const N &n2) : m_x(n0), m_y(n1), m_z(n2), m_spare(0.0) {}

        // build from short array in memory

        explicit Vector3DTemplate(const N *ns) : m_x(ns[0]), m_y(ns[1]), m_z(ns[2]), m_spare(0.0) {}

        Vector3DTemplate(const std::array<float, 3> &threeTuple)
            : m_x((N)threeTuple[0]), m_y((N)threeTuple[1]), m_z((N)threeTuple[2]), m_spare(0.0)
        {
        }

        Vector3DTemplate(const std::array<double, 3> &threeTuple)
            : m_x((N)threeTuple[0]), m_y((N)threeTuple[1]), m_z((N)threeTuple[2]), m_spare(0.0)
        {
        }

#ifdef __AVX_AVAILABLE__
        // Constructor to convert from type __m256d used in intrinsics:
        Vector3DTemplate(__m256d const &x) { m_ymm = x; }
        // Assignment operator to convert from type __m256d used in intrinsics:
        Vector3DTemplate &operator=(__m256d const &x)
        {
            m_ymm = x;
            return *this;
        }
        // Type cast operator to convert to __m256d used in intrinsics
        operator __m256d() const { return m_ymm; }
        // Member function to load from array (unaligned)
        Vector3DTemplate &load(double const *p)
        {
            m_ymm = _mm256_loadu_pd(p);
            return *this;
        }
        // Member function to load from array, aligned by 32
        // You may use load_a instead of load if you are certain that p points to an address
        // divisible by 32
        Vector3DTemplate &load_a(double const *p)
        {
            m_ymm = _mm256_load_pd(p);
            return *this;
        }
#endif

        static Vector3DTemplate randomVector(float min, float max)
        {
            if constexpr (SIMD == SIMDInstructionSet::AVX)
            {
                std::array<float, 4> result;

                drand_sse(result, min, max);

                return Vector3DTemplate((N)result[0], (N)result[1], (N)result[2]);
            }
            else
            {
                return Vector3DTemplate((N)drand(min, max), (N)drand(min, max), (N)drand(min, max));
            }
        }

        N x() const { return (m_x); }
        N y() const { return (m_y); }
        N z() const { return (m_z); }

        N &operator[](uint i) { return m_v[i]; }
        N operator[](uint i) const { return m_v[i]; }

        //
        //  Non Destructive Arithmetic
        //

        Vector3DTemplate operator-() const { return Vector3DTemplate(-m_x, -m_y, -m_z); }

        Vector3DTemplate operator+(const Vector3DTemplate &rhs) const
        {
            Vector3DTemplate res(*this);
            return res += rhs;
        }

        Vector3DTemplate operator-(const Vector3DTemplate &rhs) const
        {
            Vector3DTemplate res(*this);
            return res -= rhs;
        }

        Vector3DTemplate operator*(const N rhs) const
        {
            Vector3DTemplate res(*this);
            return res *= rhs;
        }

        Vector3DTemplate operator/(const N &rhs) const
        {
            Vector3DTemplate res(*this);
            return res /= rhs;
        }

        //
        //   Destructive Arithmetic
        //

        inline Vector3DTemplate operator+=(const Vector3DTemplate &rhs)
        {
            m_x += rhs.m_x;
            m_y += rhs.m_y;
            m_z += rhs.m_z;
            return *this;
        }

        inline Vector3DTemplate operator-=(const Vector3DTemplate &rhs)
        {
            m_x -= rhs.m_x;
            m_y -= rhs.m_y;
            m_z -= rhs.m_z;
            return *this;
        }

        inline Vector3DTemplate operator*=(const N &rhs)
        {
            m_x *= rhs;
            m_y *= rhs;
            m_z *= rhs;
            return *this;
        }

        inline Vector3DTemplate operator/=(const N &rhs)
        {
            m_x /= rhs;
            m_y /= rhs;
            m_z /= rhs;
            return *this;
        }

        //
        //  Comparison operators
        //

        bool operator==(const Vector3DTemplate &rhs) const
        {
            return m_x == rhs.m_x && m_y == rhs.m_y && m_z == rhs.m_z;
        }

        bool operator!=(const Vector3DTemplate &rhs) const
        {
            return m_x != rhs.m_x || m_y != rhs.m_y || m_z != rhs.m_z;
        }

        bool operator<(const Vector3DTemplate &vertexToCompare) const
        {
            //	Equality is by x, then y and finally z

            if (m_x < vertexToCompare.m_x)
            {
                return (true);
            }

            if (m_x > vertexToCompare.m_x)
            {
                return (false);
            }

            //	X values are equal

            if (m_y < vertexToCompare.m_y)
            {
                return (true);
            }

            if (m_y > vertexToCompare.m_y)
            {
                return (false);
            }

            //	X and Y values are equal

            if (m_z < vertexToCompare.m_z)
            {
                return (true);
            }

            if (m_z > vertexToCompare.m_z)
            {
                return (false);
            }

            //	The only way we should end up down here is if the two vertices are equal

            return (false);
        }

        //
        //  Collapsing operations
        //

        N len_squared() const { return (m_x * m_x) + (m_y * m_y) + (m_z * m_z); }

        N len() const { return std::sqrt(len_squared()); }

        //
        //  Normalization
        //

        void normalize() { *this /= this->len(); }

        Vector3DTemplate normalized() const
        {
            Vector3DTemplate vec(*this);
            return vec / vec.len();
        }

        //
        //  Dot and Cross Products
        //

        N dot(const Vector3DTemplate &rhs) const { return m_x * rhs.m_x + m_y * rhs.m_y + m_z * rhs.m_z; }

        Vector3DTemplate cross(const Vector3DTemplate &rhs) const
        {
            return Vector3DTemplate(m_y * rhs.m_z - m_z * rhs.m_y, m_z * rhs.m_x - m_x * rhs.m_z,
                                    m_x * rhs.m_y - m_y * rhs.m_x);
        }

        //
        //  Projection
        //

        Vector2DTemplate<N> project(uint dim) { return Vector2DTemplate(m_v[(dim + 1) % 3], m_v[(dim + 2) % 3]); }

        //
        //  Other functions
        //

        Vector3DTemplate abs() const { return Vector3DTemplate(fabs(m_x), fabs(m_y), fabs(m_z)); }

        N max() const { return std::max(m_x, std::max(m_y, m_z)); }

        N min() const { return std::min(m_x, std::min(m_y, m_z)); }

        uint maxDim() const { return (m_x >= m_y) ? ((m_x >= m_z) ? 0 : 2) : ((m_y >= m_z) ? 1 : 2); }

        uint minDim() const { return (m_x <= m_y) ? ((m_x <= m_z) ? 0 : 2) : ((m_y <= m_z) ? 1 : 2); }

        Vector3DTemplate max(const Vector3DTemplate &rhs) const
        {
            if constexpr (SIMD == SIMDInstructionSet::AVX)
            {
                return _mm256_max_pd(*this, rhs);
            }
            else
            {
                return Vector3DTemplate(std::max(m_x, rhs.m_x), std::max(m_y, rhs.m_y), std::max(m_z, rhs.m_z));
            }
        }

        Vector3DTemplate max(const Vector3DTemplate &vec2, const Vector3DTemplate &vec3) const
        {
            if constexpr (SIMD == SIMDInstructionSet::AVX)
            {
                return _mm256_max_pd(*this, _mm256_max_pd(vec2, vec3));
            }
            else
            {
                return Vector3DTemplate(std::max(m_x, std::max(vec2.m_x, vec3.m_x)),
                                        std::max(m_y, std::max(vec2.m_y, vec3.m_y)),
                                        std::max(m_z, std::max(vec2.m_z, vec3.m_z)));
            }
        }

        Vector3DTemplate min(const Vector3DTemplate &rhs) const
        {
            if constexpr (SIMD == SIMDInstructionSet::AVX)
            {
                return _mm256_min_pd(*this, rhs);
            }
            else
            {
                return Vector3DTemplate(std::min(m_x, rhs.m_x), std::min(m_y, rhs.m_y), std::min(m_z, rhs.m_z));
            }
        }

        Vector3DTemplate min(const Vector3DTemplate &vec2, const Vector3DTemplate &vec3) const
        {
            if constexpr (SIMD == SIMDInstructionSet::AVX)
            {
                return _mm256_min_pd(*this, _mm256_min_pd(vec2, vec3));
            }
            else
            {
                return Vector3DTemplate(std::min(m_x, std::min(vec2.m_x, vec3.m_x)),
                                        std::min(m_y, std::min(vec2.m_y, vec3.m_y)),
                                        std::min(m_z, std::min(vec2.m_z, vec3.m_z)));
            }
        }

        //
        //  Friends
        //

        template <class T, SIMDInstructionSet S>
        friend T determinant(const Vector3DTemplate<T, S> &, const Vector3DTemplate<T, S> &, const Vector3DTemplate<T, S> &);

        template <class T, SIMDInstructionSet S>
        friend std::ostream &operator<<(std::ostream &, const Vector3DTemplate<T, S> &);
    };

    //
    // Non Destructive Arithmetic
    //

    template <class T, SIMDInstructionSet SIMD>
    Vector3DTemplate<T, SIMD> operator*(const T &lhs, const Vector3DTemplate<T,SIMD> &rhs)
    {
        Vector3DTemplate<T,SIMD> res(rhs);
        return res *= lhs;
    }

    // output/input stream operators
    template <class T, SIMDInstructionSet SIMD>
    inline std::ostream &operator<<(std::ostream &out, const Vector3DTemplate<T,SIMD> &vec)
    {
        return out << '[' << vec.m_x << ',' << vec.m_y << ',' << vec.m_z << ']';
    }

    // determinant of 3 Vector3DTemplates
    template <class T, SIMDInstructionSet SIMD>
    inline T determinant(const Vector3DTemplate<T,SIMD> &v0, const Vector3DTemplate<T,SIMD> &v1, const Vector3DTemplate<T,SIMD> &v2)
    {
        T xy = v0.m_x * v1.m_y - v0.m_y * v1.m_x;
        T xz = v0.m_x * v1.m_z - v0.m_z * v1.m_x;
        T yz = v0.m_y * v1.m_z - v0.m_z * v1.m_y;
        return xy * v2.m_z - xz * v2.m_y + yz * v2.m_x;
    }

}  // namespace Cork::Math
