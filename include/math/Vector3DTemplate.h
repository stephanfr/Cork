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

#include <fmt/compile.h>
#include <fmt/format-inl.h>
#include <immintrin.h>

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include "../src/CorkDefs.h"
#include "Vector2DTemplate.h"
#include "Xoshiro256Plus.h"


namespace Cork::Math
{
    template <class N, SIMDInstructionSet SIMD = g_SIMD_Level>
    class Vector3DTemplate final
    {
       protected:  // names
        union
        {
            N v_[4];

            struct
            {
                N x_;
                N y_;
                N z_;
                N spare_;
            };

#ifdef __AVX_AVAILABLE__
            alignas(SIMD_MEMORY_ALIGNMENT) __m256d ymm_;
#endif
        };

        inline static SEFUtility::RNG::Xoshiro256Plus<SIMD> random_generator_ =
            SEFUtility::RNG::Xoshiro256Plus<SIMD>(1);

        friend class BBox3D;

       public:
        //	Constructors

        Vector3DTemplate() : spare_(0.0) { __CHECK_ALIGNMENT__(ymm_); }

        Vector3DTemplate(const N &n0, const N &n1, const N &n2) : x_(n0), y_(n1), z_(n2), spare_(0.0)
        {
            __CHECK_ALIGNMENT__(ymm_);
        }

        // build from short array in memory

        explicit Vector3DTemplate(const N *ns) : x_(ns[0]), y_(ns[1]), z_(ns[2]), spare_(0.0)
        {
            __CHECK_ALIGNMENT__(ymm_);
        }

        Vector3DTemplate(const std::array<float, 3> &threeTuple)
            : x_((N)threeTuple[0]), y_((N)threeTuple[1]), z_((N)threeTuple[2]), spare_(0.0)
        {
            __CHECK_ALIGNMENT__(ymm_);
        }

        Vector3DTemplate(const std::array<double, 3> &threeTuple)
            : x_((N)threeTuple[0]), y_((N)threeTuple[1]), z_((N)threeTuple[2]), spare_(0.0)
        {
            __CHECK_ALIGNMENT__(ymm_);
        }

#ifdef __AVX_AVAILABLE__
        // Constructor to convert from type __m256d used in intrinsics:
        Vector3DTemplate(__m256d const &x)
        {
            __CHECK_ALIGNMENT__(ymm_);
            ymm_ = x;
        }
        // Assignment operator to convert from type __m256d used in intrinsics:
        Vector3DTemplate &operator=(__m256d const &x)
        {
            __CHECK_ALIGNMENT__(ymm_);

            ymm_ = x;
            return *this;
        }
        // Type cast operator to convert to __m256d used in intrinsics
        operator __m256d() const
        {
            __CHECK_ALIGNMENT__(ymm_);
            return ymm_;
        }
        // Member function to load from array (unaligned)
        Vector3DTemplate &load(double const *p)
        {
            __CHECK_ALIGNMENT__(ymm_);

            ymm_ = _mm256_loadu_pd(p);
            return *this;
        }
        // Member function to load from array, aligned by 32
        // You may use load_a instead of load if you are certain that p points to an address
        // divisible by 32
        Vector3DTemplate &load_a(double const *p)
        {
            __CHECK_ALIGNMENT__(ymm_);

            ymm_ = _mm256_load_pd(p);
            return *this;
        }
#endif

        [[nodiscard]] static Vector3DTemplate randomVector(N min, N max)
        {
            return Vector3DTemplate(random_generator_.dnext4(min, max));
        }

        [[nodiscard]] N x() const { return (x_); }
        [[nodiscard]] N y() const { return (y_); }
        [[nodiscard]] N z() const { return (z_); }

        [[nodiscard]] N &operator[](uint i) { return v_[i]; }
        [[nodiscard]] N operator[](uint i) const { return v_[i]; }

        //
        //  Non Destructive Arithmetic
        //

        [[nodiscard]] Vector3DTemplate operator-() const { return Vector3DTemplate(-x_, -y_, -z_); }

        [[nodiscard]] Vector3DTemplate operator+(const Vector3DTemplate &rhs) const
        {
            Vector3DTemplate res(*this);
            return res += rhs;
        }

        [[nodiscard]] Vector3DTemplate operator-(const Vector3DTemplate &rhs) const
        {
            Vector3DTemplate res(*this);
            return res -= rhs;
        }

        [[nodiscard]] Vector3DTemplate operator*(N rhs) const
        {
            Vector3DTemplate res(*this);
            return res *= rhs;
        }

        [[nodiscard]] Vector3DTemplate operator/(N rhs) const
        {
            Vector3DTemplate res(*this);
            return res /= rhs;
        }

        //
        //   Destructive Arithmetic
        //

        Vector3DTemplate operator+=(const Vector3DTemplate &rhs)
        {
            x_ += rhs.x_;
            y_ += rhs.y_;
            z_ += rhs.z_;
            return *this;
        }

        Vector3DTemplate operator-=(const Vector3DTemplate &rhs)
        {
            x_ -= rhs.x_;
            y_ -= rhs.y_;
            z_ -= rhs.z_;
            return *this;
        }

        Vector3DTemplate operator*=(const N &rhs)
        {
            x_ *= rhs;
            y_ *= rhs;
            z_ *= rhs;
            return *this;
        }

        Vector3DTemplate operator/=(const N &rhs)
        {
            x_ /= rhs;
            y_ /= rhs;
            z_ /= rhs;
            return *this;
        }

        //
        //  Comparison operators
        //

        bool operator==(const Vector3DTemplate &rhs) const { return x_ == rhs.x_ && y_ == rhs.y_ && z_ == rhs.z_; }

        bool operator!=(const Vector3DTemplate &rhs) const { return x_ != rhs.x_ || y_ != rhs.y_ || z_ != rhs.z_; }

        bool operator<(const Vector3DTemplate &vertexToCompare) const
        {
            //	Equality is by x, then y and finally z

            if (x_ < vertexToCompare.x_)
            {
                return (true);
            }

            if (x_ > vertexToCompare.x_)
            {
                return (false);
            }

            //	X values are equal

            if (y_ < vertexToCompare.y_)
            {
                return (true);
            }

            if (y_ > vertexToCompare.y_)
            {
                return (false);
            }

            //	X and Y values are equal

            if (z_ < vertexToCompare.z_)
            {
                return (true);
            }

            if (z_ > vertexToCompare.z_)
            {
                return (false);
            }

            //	The only way we should end up down here is if the two vertices are equal

            return (false);
        }

        //
        //  Collapsing operations
        //

        [[nodiscard]] N len_squared() const { return (x_ * x_) + (y_ * y_) + (z_ * z_); }

        [[nodiscard]] N len() const { return std::sqrt(len_squared()); }

        //
        //  Normalization
        //

        void normalize() { *this /= this->len(); }

        [[nodiscard]] Vector3DTemplate normalized() const
        {
            Vector3DTemplate vec(*this);
            return vec / vec.len();
        }

        //
        //  Dot and Cross Products
        //

        [[nodiscard]] N dot(const Vector3DTemplate &rhs) const { return x_ * rhs.x_ + y_ * rhs.y_ + z_ * rhs.z_; }

        [[nodiscard]] Vector3DTemplate cross(const Vector3DTemplate &rhs) const
        {
            return Vector3DTemplate(y_ * rhs.z_ - z_ * rhs.y_, z_ * rhs.x_ - x_ * rhs.z_, x_ * rhs.y_ - y_ * rhs.x_);
        }

        //
        //  Projection
        //

        [[nodiscard]] Vector2DTemplate<N> project(uint dim)
        {
            return Vector2DTemplate(v_[(dim + 1) % 3], v_[(dim + 2) % 3]);
        }

        //
        //  Other functions
        //

        [[nodiscard]] Vector3DTemplate abs() const { return Vector3DTemplate(fabs(x_), fabs(y_), fabs(z_)); }

        [[nodiscard]] N max() const { return std::max(x_, std::max(y_, z_)); }

        [[nodiscard]] N min() const { return std::min(x_, std::min(y_, z_)); }

        [[nodiscard]] uint maxDim() const { return (x_ >= y_) ? ((x_ >= z_) ? 0 : 2) : ((y_ >= z_) ? 1 : 2); }

        [[nodiscard]] uint minDim() const { return (x_ <= y_) ? ((x_ <= z_) ? 0 : 2) : ((y_ <= z_) ? 1 : 2); }

        [[nodiscard]] Vector3DTemplate max(const Vector3DTemplate &rhs) const
        {
            if constexpr (SIMD >= SIMDInstructionSet::AVX)
            {
                return _mm256_max_pd(*this, rhs);
            }
            else
            {
                return Vector3DTemplate(std::max(x_, rhs.x_), std::max(y_, rhs.y_), std::max(z_, rhs.z_));
            }
        }

        [[nodiscard]] Vector3DTemplate max(const Vector3DTemplate &vec2, const Vector3DTemplate &vec3) const
        {
            if constexpr (SIMD >= SIMDInstructionSet::AVX)
            {
                return _mm256_max_pd(*this, _mm256_max_pd(vec2, vec3));
            }
            else
            {
                return Vector3DTemplate(std::max(x_, std::max(vec2.x_, vec3.x_)),
                                        std::max(y_, std::max(vec2.y_, vec3.y_)),
                                        std::max(z_, std::max(vec2.z_, vec3.z_)));
            }
        }

        [[nodiscard]] Vector3DTemplate min(const Vector3DTemplate &rhs) const
        {
            if constexpr (SIMD >= SIMDInstructionSet::AVX)
            {
                return _mm256_min_pd(*this, rhs);
            }
            else
            {
                return Vector3DTemplate(std::min(x_, rhs.x_), std::min(y_, rhs.y_), std::min(z_, rhs.z_));
            }
        }

        [[nodiscard]] Vector3DTemplate min(const Vector3DTemplate &vec2, const Vector3DTemplate &vec3) const
        {
            if constexpr (SIMD >= SIMDInstructionSet::AVX)
            {
                return _mm256_min_pd(*this, _mm256_min_pd(vec2, vec3));
            }
            else
            {
                return Vector3DTemplate(std::min(x_, std::min(vec2.x_, vec3.x_)),
                                        std::min(y_, std::min(vec2.y_, vec3.y_)),
                                        std::min(z_, std::min(vec2.z_, vec3.z_)));
            }
        }
    };

    //
    //  Define the random number generator
    //

    //    template <class N, SIMDInstructionSet SIMD>
    //    SEFUtility::RNG::Xoshiro256Plus<SIMD> Vector3DTemplate<N, SIMD>::random_generator_(1);

    //
    // Non Destructive Arithmetic
    //

    template <class T, SIMDInstructionSet SIMD>
    [[nodiscard]] Vector3DTemplate<T, SIMD> operator*(const T &lhs, const Vector3DTemplate<T, SIMD> &rhs)
    {
        Vector3DTemplate<T, SIMD> res(rhs);
        return res *= lhs;
    }

    // output/input stream operators

    template <class T, SIMDInstructionSet SIMD>
    inline std::ostream &operator<<(std::ostream &out, const Vector3DTemplate<T, SIMD> &vec)
    {
        return out << fmt::format(FMT_COMPILE("[{:f},{:f},{:f}]"), vec.x(), vec.y(), vec.z());
    }

    // determinant of 3 Vector3DTemplates
    template <class T, SIMDInstructionSet SIMD>
    [[nodiscard]] inline T determinant(const Vector3DTemplate<T, SIMD> &v0, const Vector3DTemplate<T, SIMD> &v1,
                                       const Vector3DTemplate<T, SIMD> &v2)
    {
        T xy = v0.x() * v1.y() - v0.y() * v1.x();
        T xz = v0.x() * v1.z() - v0.z() * v1.x();
        T yz = v0.y() * v1.z() - v0.z() * v1.y();
        return xy * v2.z() - xz * v2.y() + yz * v2.x();
    }

}  // namespace Cork::Math
