// +-------------------------------------------------------------------------
// | ext4.h
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

#include "math/Primitives.h"

/*
 *
 *  Ext4
 *
 *      Support for performing exterior calculus in R4
 *
 */

namespace Ext4
{
    // types for k-vectors in R4:
    //      Ext4_k

    class Ext4_2;
    class Ext4_3;

    class Ext4_1
    {
       public:

        explicit Ext4_1(const Ext4_1 &ext_to_copy) : v_(ext_to_copy.v_) {}
        explicit Ext4_1(Ext4_1 &&ext_to_copy)
        {
            v_ = ext_to_copy.v_;
        }

#if defined(NUMERIC_PRECISION) && NUMERIC_PRECISION == double

        explicit Ext4_1(const Cork::Math::Vector3D &vector) : vector_(vector) { e3_ = 1.0f; }

        const Ext4_1 &operator=(const Cork::Math::Vector3D &vector)
        {
            vector_ = vector;
            e3_ = 1.0f;

            return *this;
        }

        operator Cork::Math::Vector3D() const
        {
            // Warning: beware of division by zero!

            return vector_ / e3_;
        }
#else
        explicit Ext4_1(const Cork::Math::Vector3D &vector) : e0(vector.x()), e1(vector.y()), e1(vector.z()), e3(1.0f)
        {
        }

        const Ext4_1 &operator=(const Cork::Math::Vector3D &vector)
        {
            e0 = vector.x();
            e1 = vector.y();
            e2 = vector.z();
            e3 = 1.0f;

            return *this;
        }

        Cork::Math::Vector3D operator() const
        {
            // Warning: beware of division by zero!

            return Cork::Math::Vector3D((NUMERIC_PRECISION)(e0 / e3), (NUMERIC_PRECISION)(e1 / e3),
                                        (NUMERIC_PRECISION)(e2 / e3));
        }

#endif

        double e0() const { return e0_; }
        double e1() const { return e1_; }
        double e2() const { return e2_; }
        double e3() const { return e3_; }

        double &operator[](size_t index) { return v_[index]; }
        double operator[](size_t index) const { return v_[index]; }

        //  A dual operation takes a k-vector and returns a (4-k)-vector
        //      A reverse dual operation inverts the dual operation
        //      dual(X,Y) is not safe for X=Y (same with revdual)

        Ext4_3 dual() const;
        Ext4_3 reverse_dual() const;

        //  Join takes a j-vector and a k-vector and returns a (j+k)-vector

        Ext4_2 join(const Ext4_1 &rhs) const;

       private:
        Ext4_1(double e0, double e1, double e2, double e3) : e0_(e0), e1_(e1), e2_(e2), e3_(e3){};

        union
        {
            std::array<double, 4> v_;

            struct
            {
                double e0_;
                double e1_;
                double e2_;
                double e3_;
            };

#if defined(NUMERIC_PRECISION) && NUMERIC_PRECISION == double
            Cork::Math::Vector3D vector_;
#endif

#ifdef __AVX_AVAILABLE__
            alignas(SIMD_MEMORY_ALIGNMENT) __m256d ymm_;
#endif
        };

        friend class Ext4_2;
        friend class Ext4_3;
    };

    class Ext4_2
    {
       public:
        double &operator[](size_t index) { return v[index]; }
        double operator[](size_t index) const { return v[index]; }

        Ext4_2 negate() const { return Ext4_2(-e01, -e02, -e03, -e12, -e13, -e23); }

        Ext4_2 dual() const { return Ext4_2(e23, -e13, e12, e03, -e02, e01); }

        Ext4_2 reverse_dual() const { return Ext4_2(e23, -e13, e12, e03, -e02, e01); }

        Ext4_3 join(const Ext4_1 &rhs) const;

        Ext4_1 meet(const Ext4_3 &rhs) const;

       private:
        Ext4_2(){};

        Ext4_2(double e01__, double e02__, double e03__, double e12__, double e13__, double e23__)
            : e01(e01__), e02(e02__), e03(e03__), e12(e12__), e13(e13__), e23(e23__)
        {
        }

        union
        {
            double v[6];

            struct
            {
                double e01;
                double e02;
                double e03;
                double e12;
                double e13;
                double e23;
            };
        };

        friend class Ext4_1;
    };

    struct Ext4_3
    {
        union
        {
            double v[4];

            struct
            {
                double e012;
                double e013;
                double e023;
                double e123;
            };
        };

        Ext4_1 dual() const { return Ext4_1(e123, -e023, e013, -e012); };

        Ext4_1 revdual() const { return Ext4_1(-e123, e023, -e013, e012); }
    };

    // ************************
    // Output routines

    // NONE CURRENTLY

    // ************************
    // A neg takes a k-vector and returns its negation
    // neg(X,Y) is safe for X=Y

    inline void neg(Ext4_1 &out, const Ext4_1 &in)
    {
        for (int i = 0; i < 4; i++)
        {
            out[i] = -in[i];
        }
    }

    inline void neg(Ext4_3 &out, const Ext4_3 &in)
    {
        for (int i = 0; i < 4; i++)
        {
            out.v[i] = -in.v[i];
        }
    }

    // ************************
    // A dual operation takes a k-vector and returns a (4-k)-vector
    // A reverse dual operation inverts the dual operation
    // dual(X,Y) is not safe for X=Y (same with revdual)

    inline Ext4_3 Ext4_1::dual() const
    {
        Ext4_3 out;

        out.e012 = e3_;
        out.e013 = -e2_;
        out.e023 = e1_;
        out.e123 = -e0_;

        return out;
    }

    inline Ext4_3 Ext4_1::reverse_dual() const
    {
        Ext4_3 out;

        out.e012 = -e3_;
        out.e013 = e2_;
        out.e023 = -e1_;
        out.e123 = e0_;

        return out;
    }

    inline Ext4_2 Ext4_1::join(const Ext4_1 &rhs) const
    {
        Ext4_2 out;

        out.e01 = (e0_ * rhs.e1_) - (rhs.e0_ * e1_);
        out.e02 = (e0_ * rhs.e2_) - (rhs.e0_ * e2_);
        out.e03 = (e0_ * rhs.e3_) - (rhs.e0_ * e3_);
        out.e12 = (e1_ * rhs.e2_) - (rhs.e1_ * e2_);
        out.e13 = (e1_ * rhs.e3_) - (rhs.e1_ * e3_);
        out.e23 = (e2_ * rhs.e3_) - (rhs.e2_ * e3_);

        return out;
    }

    inline Ext4_3 Ext4_2::join(const Ext4_1 &rhs) const
    {
        Ext4_3 out;

        out.e012 = (e01 * rhs.e2_) - (e02 * rhs.e1_) + (e12 * rhs.e0_);
        out.e013 = (e01 * rhs.e3_) - (e03 * rhs.e1_) + (e13 * rhs.e0_);
        out.e023 = (e02 * rhs.e3_) - (e03 * rhs.e2_) + (e23 * rhs.e0_);
        out.e123 = (e12 * rhs.e3_) - (e13 * rhs.e2_) + (e23 * rhs.e1_);

        return out;
    }

    inline Ext4_1 Ext4_2::meet(const Ext4_3 &rhs) const { return (dual().join(rhs.dual())).revdual(); }

    // ************************
    // A join takes a j-vector and a k-vector and returns a (j+k)-vector
    /*
        inline void join(Ext4_2 &out, const Ext4_1 &lhs, const Ext4_1 &rhs)
        {
            out.e01 = (lhs.e0 * rhs.e1) - (rhs.e0 * lhs.e1);
            out.e02 = (lhs.e0 * rhs.e2) - (rhs.e0 * lhs.e2);
            out.e03 = (lhs.e0 * rhs.e3) - (rhs.e0 * lhs.e3);
            out.e12 = (lhs.e1 * rhs.e2) - (rhs.e1 * lhs.e2);
            out.e13 = (lhs.e1 * rhs.e3) - (rhs.e1 * lhs.e3);
            out.e23 = (lhs.e2 * rhs.e3) - (rhs.e2 * lhs.e3);
        }

        inline void join(Ext4_3 &out, const Ext4_2 &lhs, const Ext4_1 &rhs)
        {
            out.e012 = (lhs.e01 * rhs.e2) - (lhs.e02 * rhs.e1) + (lhs.e12 * rhs.e0);
            out.e013 = (lhs.e01 * rhs.e3) - (lhs.e03 * rhs.e1) + (lhs.e13 * rhs.e0);
            out.e023 = (lhs.e02 * rhs.e3) - (lhs.e03 * rhs.e2) + (lhs.e23 * rhs.e0);
            out.e123 = (lhs.e12 * rhs.e3) - (lhs.e13 * rhs.e2) + (lhs.e23 * rhs.e1);
        }
    */

    inline Ext4_3 join(const Ext4_1 &lhs, const Ext4_2 &rhs)
    {
        return rhs.join(lhs);
        // no negation since swapping the arguments requires two
        // swaps of 1-vectors
    }

    // ************************
    // A meet takes a j-vector and a k-vector and returns a (j+k-4)-vector

    inline Ext4_2  meet( const Ext4_3 &lhs, const Ext4_3 &rhs)
    {
        return lhs.dual().join(rhs.dual()).reverse_dual();
    }

    inline Ext4_1 meet( const Ext4_3 &lhs, const Ext4_2 &rhs)
    {
        return join(lhs.dual(), rhs.dual()).revdual();
    }

    // ************************
    // An inner product takes two k-vectors and produces a single number

    inline double inner(const Ext4_1 &lhs, const Ext4_1 &rhs)
    {
        double acc = 0.0;

        for (int i = 0; i < 4; i++)
        {
            acc += lhs[i] * rhs[i];
        }

        return (acc);
    }

    inline double inner(const Ext4_2 &lhs, const Ext4_2 &rhs)
    {
        double acc = 0.0;

        for (int i = 0; i < 6; i++)
        {
            acc += lhs[i] * rhs[i];
        }

        return (acc);
    }

    inline double inner(const Ext4_3 &lhs, const Ext4_3 &rhs)
    {
        double acc = 0.0;

        for (int i = 0; i < 4; i++)
        {
            acc += lhs.v[i] * rhs.v[i];
        }

        return (acc);
    }

}  // namespace Ext4
