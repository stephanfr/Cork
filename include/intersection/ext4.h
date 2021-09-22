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

#include <gsl/gsl_util>

#include "math/Primitives.h"

/*

   ExteriorCalculusR4

       Support for performing exterior calculus in R4

   The subscripts in the different classes refer to elements in vectors or matrices.
       The 'k-value' indicates the dimension of the class, i.e. Ext4_1 is one dimensional so it is a vector.
       Ext4_2 is two dimensional so it is a matrix.  Ext4_3 is three dimensional and is a vector of matrices.
       Ext4_2 and Ext4_3 are upper diagonal

     Ext4_1 ->  [ e0   e1   e2   e3 ]

                 -                   -
                 | e01  e02  e03     |
      Ext4_2 ->  |      e12  e13     |
                 |           e23     |
                 |                   |
                 -                   -

                -                                     -
                | -               - -               - |
                | |               | |               | |
      Ext4_3 -> | |      e12  e13 | |               | |
                | |           e23 | |           e23 | |
                | -               - -               - |
                -                                     -

    The join() operation is simply the wedge product of the two items.  The wedge product takes two operands and returns
    an item with a k-value which is the sum of the two k values of the operands.  Thus :

        Ext4_1 ^ Ext4_1 -> Ext4_2
        Ext4_1 ^ Ext4_2 -> Ext4_3
        Ext4_2 ^ Ext4_1 -> Ext4_3

    The wedge product of an Ext4_3 with an Ext4_1 or Ext4_2 would result in a k-value >= the order 4, so these
   operations are not allowed.  I don't think they make physical sense.

 */


#undef __AVX_AVAILABLE__

namespace ExteriorCalculusR4
{
    namespace Constants
    {
        constexpr double DOUBLE_ONE = 1.0;
    }

    // types for k-vectors in R4:
    //      Ext4_k

    class Ext4_2;
    class Ext4_3;
    class AbsExt4_1;
    class AbsExt4_2;
    class AbsExt4_3;

    class Ext4_1
    {
       public:
        explicit Ext4_1(const Ext4_1 &ext_to_copy) : v_(ext_to_copy.v_) {}

#if defined(NUMERIC_PRECISION) && NUMERIC_PRECISION == double

        explicit Ext4_1(const Cork::Math::Vector3D &vector) : vector_(vector) { e3_ = Constants::DOUBLE_ONE; }

#ifdef __AVX_AVAILABLE__
        explicit Ext4_1(__m256d &ymm) : ymm_(ymm) {}

        const Ext4_1 &operator=(const Cork::Math::Vector3D &vector)
        {
            ymm_ = vector;
            e3_ = Constants::DOUBLE_ONE;

            return *this;
        }

        operator __m256d &() { return ymm_; }
        operator __m256d() const { return ymm_; }

#else  //  NOT __AVX_AVAILABLE__
        const Ext4_1 &operator=(const Cork::Math::Vector3D &vector)
        {
            vector_ = vector;
            e3_ = Constants::DOUBLE_ONE;

            return *this;
        }

#endif  //  __AVX_AVAILABLE__

        operator Cork::Math::Vector3D() const
        {
            // Warning: beware of division by zero!

            assert(e3_ != 0);
            return vector_ / e3_;
        }
#else
        explicit Ext4_1(const Cork::Math::Vector3D &vector)
            : e0(vector.x()), e1(vector.y()), e1(vector.z()), e3(Constants::DOUBLE_ONE)
        {
        }

        const Ext4_1 &operator=(const Cork::Math::Vector3D &vector)
        {
            e0 = vector.x();
            e1 = vector.y();
            e2 = vector.z();
            e3 = Constants::DOUBLE_ONE;

            return *this;
        }

        [[nodiscard]] Cork::Math::Vector3D operator() const
        {
            // Warning: beware of division by zero!

            return Cork::Math::Vector3D((NUMERIC_PRECISION)(e0 / e3), (NUMERIC_PRECISION)(e1 / e3),
                                        (NUMERIC_PRECISION)(e2 / e3));
        }

#endif

        [[nodiscard]] double e0() const { return e0_; }
        [[nodiscard]] double e1() const { return e1_; }
        [[nodiscard]] double e2() const { return e2_; }
        [[nodiscard]] double e3() const { return e3_; }

        [[nodiscard]] double &operator[](size_t index) { return gsl::at(v_, index); }
        [[nodiscard]] double operator[](size_t index) const { return gsl::at(v_, index); }

        [[nodiscard]] bool operator==(const Ext4_1 &r4_to_compare) const
        {
            return ((e0_ == r4_to_compare.e0_) && (e1_ == r4_to_compare.e1_) && (e2_ == r4_to_compare.e2_) &&
                    (e3_ == r4_to_compare.e3_));
        }

        [[nodiscard]] bool operator!=(const Ext4_1 &r4_to_compare) const
        {
            return ((e0_ != r4_to_compare.e0_) || (e1_ != r4_to_compare.e1_) || (e2_ != r4_to_compare.e2_) ||
                    (e3_ != r4_to_compare.e3_));
        }

        // Negation takes a k-vector and returns its negation neg(X,Y) and is safe for X=Y

        const Ext4_1 &negate()
        {
            e0_ = -e0_;
            e1_ = -e1_;
            e2_ = -e2_;
            e3_ = -e3_;

            return *this;
        }

        [[nodiscard]] Ext4_1 negative() const { return Ext4_1(-e0_, -e1_, -e2_, -e3_); }

        //  A dual operation takes a k-vector and returns a (4-k)-vector
        //      A reverse dual operation inverts the dual operation
        //      dual(X,Y) is not safe for X=Y (same with revdual)

        [[nodiscard]] Ext4_3 dual() const;
        [[nodiscard]] Ext4_3 reverse_dual() const;

        //  Join takes a j-vector and a k-vector and returns a (j+k) vector
        //      Join is the wedge product.

        [[nodiscard]] Ext4_3 join(const Ext4_2 &rhs) const;

        [[nodiscard]] Ext4_2 join(const Ext4_1 &rhs) const;

        //  The inner product is the familiar dot product.

        [[nodiscard]] double inner(const Ext4_1 &rhs) const
        {
            double acc = 0.0;

            //  The loop below gets unwound and optimized by the compiler

            for (int i = 0; i < 4; i++)
            {
                acc += v_[i] * rhs[i];
            }

            return (acc);
        }

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
        friend class AbsExt4_1;
    };

    class Ext4_2
    {
       public:
        //  Operators

        [[nodiscard]] double &operator[](size_t index) { return gsl::at(v_, index); }
        [[nodiscard]] double operator[](size_t index) const { return gsl::at(v_, index); }

        [[nodiscard]] double e01() const { return e01_; }
        [[nodiscard]] double e02() const { return e02_; }
        [[nodiscard]] double e03() const { return e03_; }
        [[nodiscard]] double e12() const { return e12_; }
        [[nodiscard]] double e13() const { return e13_; }
        [[nodiscard]] double e23() const { return e23_; }

        [[nodiscard]] bool operator==(const Ext4_2 &r4_to_compare) const
        {
            return ((e01_ == r4_to_compare.e01_) && (e02_ == r4_to_compare.e02_) && (e03_ == r4_to_compare.e03_) &&
                    (e12_ == r4_to_compare.e12_) && (e13_ == r4_to_compare.e13_) && (e23_ == r4_to_compare.e23_));
        }

        [[nodiscard]] bool operator!=(const Ext4_2 &r4_to_compare) const
        {
            return ((e01_ != r4_to_compare.e01_) || (e02_ != r4_to_compare.e02_) || (e03_ != r4_to_compare.e03_) ||
                    (e12_ != r4_to_compare.e12_) || (e13_ != r4_to_compare.e13_) || (e23_ != r4_to_compare.e23_));
        }

        //  Negation

        Ext4_2 &negate()
        {
            e01_ = -e01_;
            e02_ = -e02_;
            e03_ = -e03_;
            e12_ = -e12_;
            e13_ = -e13_;
            e23_ = -e23_;
            return *this;
        }

        [[nodiscard]] Ext4_2 negative() const { return Ext4_2(-e01_, -e02_, -e03_, -e12_, -e13_, -e23_); }

        //  Dual and Reverse Dual

        [[nodiscard]] Ext4_2 dual() const { return Ext4_2(e23_, -e13_, e12_, e03_, -e02_, e01_); }

        [[nodiscard]] Ext4_2 reverse_dual() const { return Ext4_2(e23_, -e13_, e12_, e03_, -e02_, e01_); }

        //  Join

        [[nodiscard]] Ext4_3 join(const Ext4_1 &rhs) const;

        //  Meet

        [[nodiscard]] Ext4_1 meet(const Ext4_3 &rhs) const;

        // An inner product takes two k-vectors and produces a single number

        [[nodiscard]] double inner(const Ext4_2 &rhs) const
        {
            double acc = 0.0;

            for (int i = 0; i < 6; i++)
            {
                acc += v_[i] * rhs[i];
            }

            return (acc);
        }

       private:
        Ext4_2(){};

        Ext4_2(double e01, double e02, double e03, double e12, double e13, double e23)
            : e01_(e01), e02_(e02), e03_(e03), e12_(e12), e13_(e13), e23_(e23)
        {
        }

        union
        {
            std::array<double, 6> v_;

            struct
            {
                double e01_;
                double e02_;
                double e03_;
                double e12_;
                double e13_;
                double e23_;
            };
        };

        friend class Ext4_1;
        friend class AbsExt4_2;
    };

    class Ext4_3
    {
       public:
        //  Accessors

        [[nodiscard]] double e012() const { return e012_; }
        [[nodiscard]] double e013() const { return e013_; }
        [[nodiscard]] double e023() const { return e023_; }
        [[nodiscard]] double e123() const { return e123_; }

        [[nodiscard]] double &operator[](size_t index) { return gsl::at(v_, index); }
        [[nodiscard]] double operator[](size_t index) const { return gsl::at(v_, index); }

        [[nodiscard]] bool operator==(const Ext4_3 &r4_to_compare) const
        {
            return ((e012_ == r4_to_compare.e012_) && (e013_ == r4_to_compare.e013_) &&
                    (e023_ == r4_to_compare.e023_) && (e123_ == r4_to_compare.e123_));
        }

        [[nodiscard]] bool operator!=(const Ext4_3 &r4_to_compare) const
        {
            return ((e012_ != r4_to_compare.e012_) || (e013_ != r4_to_compare.e013_) ||
                    (e023_ != r4_to_compare.e023_) || (e123_ != r4_to_compare.e123_));
        }

        //  Negation

        Ext4_3 &negate()
        {
            e012_ = -e012_;
            e013_ = -e013_;
            e023_ = -e023_;
            e123_ = -e123_;
            return *this;
        }

        [[nodiscard]] Ext4_3 negative() const { return Ext4_3(-e012_, -e013_, -e023_, -e123_); }

        //  Dual and reverse dual

        [[nodiscard]] Ext4_1 dual() const { return Ext4_1(e123_, -e023_, e013_, -e012_); };

        [[nodiscard]] Ext4_1 reverse_dual() const { return Ext4_1(-e123_, e023_, -e013_, e012_); }

        // A meet takes a j-vector and a k-vector and returns a (j+k-4)-vector

        [[nodiscard]] Ext4_2 meet(const Ext4_3 &rhs) const { return dual().join(rhs.dual()).reverse_dual(); }

        [[nodiscard]] Ext4_1 meet(const Ext4_2 &rhs) const { return dual().join(rhs.dual()).reverse_dual(); }

        //  Dot product

        [[nodiscard]] double inner(const Ext4_3 &rhs)
        {
            double acc = 0.0;

            for (int i = 0; i < 4; i++)
            {
                acc += v_[i] * rhs[i];
            }

            return (acc);
        }

        Ext4_3(){};

       private:
        Ext4_3(double e012, double e013, double e023, double e123) : e012_(e012), e013_(e013), e023_(e023), e123_(e123)
        {
        }

        union
        {
            std::array<double, 4> v_;

            struct
            {
                double e012_;
                double e013_;
                double e023_;
                double e123_;
            };

#ifdef __AVX_AVAILABLE__
            alignas(SIMD_MEMORY_ALIGNMENT) __m256d ymm_;
#endif
        };

        friend class Ext4_1;
        friend class Ext4_2;
        friend class AbsExt4_3;
    };

    // A dual operation takes a k-vector and returns a (4-k)-vector
    // A reverse dual operation inverts the dual operation
    // dual(X,Y) is not safe for X=Y (same with revdual)

    inline Ext4_3 Ext4_1::dual() const { return Ext4_3(e3_, -e2_, e1_, -e0_); }

    inline Ext4_3 Ext4_1::reverse_dual() const { return Ext4_3(-e3_, e2_, -e1_, e0_); }

    // A join takes a j-vector and a k-vector and returns a (j+k)-vector

    inline Ext4_2 Ext4_1::join(const Ext4_1 &rhs) const
    {
        Ext4_2 out;

        out.e01_ = (e0_ * rhs.e1_) - (rhs.e0_ * e1_);
        out.e02_ = (e0_ * rhs.e2_) - (rhs.e0_ * e2_);
        out.e03_ = (e0_ * rhs.e3_) - (rhs.e0_ * e3_);
        out.e12_ = (e1_ * rhs.e2_) - (rhs.e1_ * e2_);
        out.e13_ = (e1_ * rhs.e3_) - (rhs.e1_ * e3_);
        out.e23_ = (e2_ * rhs.e3_) - (rhs.e2_ * e3_);

        return out;
    }

    inline Ext4_3 Ext4_1::join(const Ext4_2 &rhs) const
    {
        // no negation since swapping the arguments requires two swaps of 1-vectors

        return rhs.join(*this);
    }

    inline Ext4_3 Ext4_2::join(const Ext4_1 &rhs) const
    {
        Ext4_3 out;

        out.e012_ = (e01_ * rhs.e2_) - (e02_ * rhs.e1_) + (e12_ * rhs.e0_);
        out.e013_ = (e01_ * rhs.e3_) - (e03_ * rhs.e1_) + (e13_ * rhs.e0_);
        out.e023_ = (e02_ * rhs.e3_) - (e03_ * rhs.e2_) + (e23_ * rhs.e0_);
        out.e123_ = (e12_ * rhs.e3_) - (e13_ * rhs.e2_) + (e23_ * rhs.e1_);

        return out;
    }

    inline Ext4_1 Ext4_2::meet(const Ext4_3 &rhs) const { return (dual().join(rhs.dual())).reverse_dual(); }

    //
    // An abs takes a k-vector and returns a version with
    //  absolute values of all the coordinates taken
    //

    class AbsExt4_1
    {
       public:
        AbsExt4_1(){};

#ifdef __AVX_AVAILABLE__

        explicit AbsExt4_1(const AbsExt4_1 &element) : ymm_(element.ymm_) {}
        explicit AbsExt4_1(AbsExt4_1 &&element) : ymm_(element.ymm_) {}

        operator __m256d() const { return ymm_; }
        
        AbsExt4_1 &operator=(const AbsExt4_1 &element)
        {
            ymm_ = element.ymm_;

            return *this;
        }

        AbsExt4_1 &operator=(AbsExt4_1 &&element)
        {
            ymm_ = element.ymm_;

            return *this;
        }
#else
        explicit AbsExt4_1(const AbsExt4_1 &element) : v_(element.v_){}
        explicit AbsExt4_1(AbsExt4_1 &&element) : v_(element.v_) {}

        AbsExt4_1 &operator=(const AbsExt4_1 &element)
        {
            v_ = element.v_;

            return *this;
        }

        AbsExt4_1 &operator=(AbsExt4_1 &&element)
        {
            v_ = element.v_;

            return *this;
        }
#endif

        explicit AbsExt4_1(const Ext4_1 &element)
            : e0_(fabs(element.e0())), e1_(fabs(element.e1())), e2_(fabs(element.e2())), e3_(fabs(element.e3()))
        {
        }

        explicit AbsExt4_1( Ext4_1 &&element)
            : e0_(fabs(element.e0())), e1_(fabs(element.e1())), e2_(fabs(element.e2())), e3_(fabs(element.e3()))
        {
        }

        const AbsExt4_1 &operator=(const Ext4_1 &element)
        {
            e0_ = fabs(element.e0());
            e1_ = fabs(element.e1());
            e2_ = fabs(element.e2());
            e3_ = fabs(element.e3());

            return *this;
        }

        const AbsExt4_1 &operator=(Ext4_1 &&element)
        {
            e0_ = fabs(element.e0());
            e1_ = fabs(element.e1());
            e2_ = fabs(element.e2());
            e3_ = fabs(element.e3());

            return *this;
        }

        //  Accessors

        double e0() const { return e0_; }
        double e1() const { return e1_; }
        double e2() const { return e2_; }
        double e3() const { return e3_; }

        double &operator[](size_t index) { return v_[index]; }
        double operator[](size_t index) const { return v_[index]; }

        [[nodiscard]] bool operator==(const AbsExt4_1 &r4_to_compare) const
        {
            return ((e0_ == r4_to_compare.e0_) && (e1_ == r4_to_compare.e1_) && (e2_ == r4_to_compare.e2_) &&
                    (e3_ == r4_to_compare.e3_));
        }

        [[nodiscard]] bool operator!=(const AbsExt4_1 &r4_to_compare) const
        {
            return ((e0_ != r4_to_compare.e0_) || (e1_ != r4_to_compare.e1_) || (e2_ != r4_to_compare.e2_) ||
                    (e3_ != r4_to_compare.e3_));
        }

        //  Negation does not change anything for the AbsExt classes, prett much a no-op.

        AbsExt4_1 &negate() { return *this; }

        [[nodiscard]] AbsExt4_1 negative() const { return AbsExt4_1(*this); }

        // A dual operation takes a k-vector and returns a (4-k)-vector
        // A reverse dual operation inverts the dual operation
        // dual(X,Y) is not safe for X=Y (same with revdual)

        AbsExt4_3 dual() const;
        AbsExt4_3 reverse_dual() const;

        // A join takes a j-vector and a k-vector and returns a (j+k)-vector

        AbsExt4_2 join(const AbsExt4_1 &rhs) const;
        AbsExt4_3 join(const AbsExt4_2 &rhs) const;

        // An inner product takes two k-vectors and produces a single number

        double inner(const AbsExt4_1 &rhs) const
        {
            double acc = 0.0;

            for (int i = 0; i < 4; i++)
            {
                acc += v_[i] * rhs.v_[i];
            }

            return (acc);
        }

       private:
        AbsExt4_1(double e0, double e1, double e2, double e3) : e0_(e0), e1_(e1), e2_(e2), e3_(e3) {}

        union
        {
            std::array<double,4> v_;

            struct
            {
                double e0_;
                double e1_;
                double e2_;
                double e3_;
            };

#ifdef __AVX_AVAILABLE__
            alignas(SIMD_MEMORY_ALIGNMENT) __m256d ymm_;
#endif
        };

        friend class Ext4_1;
        friend class AbsExt4_3;
    };

    class AbsExt4_2
    {
       public:
        AbsExt4_2(){};

        AbsExt4_2(double e01, double e02, double e03, double e12, double e13, double e23)
            : e01_(e01), e02_(e02), e03_(e03), e12_(e12), e13_(e13), e23_(e23)
        {
        }

        explicit AbsExt4_2(Ext4_2 &ext_to_abs)
            : e01_(fabs(ext_to_abs.e01())),
              e02_(fabs(ext_to_abs.e02())),
              e03_(fabs(ext_to_abs.e03())),
              e12_(fabs(ext_to_abs.e12())),
              e13_(fabs(ext_to_abs.e13())),
              e23_(fabs(ext_to_abs.e23()))
        {
        }

        //  Accessors

        double e01() const { return e01_; }
        double e02() const { return e02_; }
        double e03() const { return e03_; }
        double e12() const { return e12_; }
        double e13() const { return e13_; }
        double e23() const { return e23_; }

        double &operator[](size_t index) { return v_[index]; }
        double operator[](size_t index) const { return v_[index]; }

        //  Negation does not change anything for the AbsExt classes

        AbsExt4_2 &negate() { return *this; }

        [[nodiscard]] AbsExt4_2 negative() const { return AbsExt4_2(*this); }

        // A dual operation takes a k-vector and returns a (4-k)-vector
        // A reverse dual operation inverts the dual operation
        // dual(X,Y) is not safe for X=Y (same with revdual)

        AbsExt4_2 dual() const { return AbsExt4_2(e23_, e13_, e12_, e03_, e02_, e01_); }

        AbsExt4_2 reverse_dual() const { return AbsExt4_2(e23_, e13_, e12_, e03_, e02_, e01_); }

        AbsExt4_3 join(const AbsExt4_1 &rhs) const;

        AbsExt4_1 meet(const AbsExt4_3 &rhs) const;

        double inner(const AbsExt4_2 &rhs) const
        {
            double acc = 0.0;

            for (int i = 0; i < 6; i++)
            {
                acc += v_[i] * rhs.v_[i];
            }

            return (acc);
        }

       private:
        union
        {
            double v_[6];

            struct
            {
                double e01_;
                double e02_;
                double e03_;
                double e12_;
                double e13_;
                double e23_;
            };
        };

        friend class AbsExt4_1;
    };

    struct AbsExt4_3
    {
        AbsExt4_3(){};

        AbsExt4_3(double e012, double e013, double e023, double e123)
            : e012_(e012), e013_(e013), e023_(e023), e123_(e123)
        {
        }

        explicit AbsExt4_3(const Ext4_3 &ext_to_abs)
            : e012_(fabs(ext_to_abs.e012())),
              e013_(fabs(ext_to_abs.e013())),
              e023_(fabs(ext_to_abs.e023())),
              e123_(fabs(ext_to_abs.e123()))
        {
        }

        //  Accessors

        double e012() const { return e012_; }
        double e013() const { return e013_; }
        double e023() const { return e023_; }
        double e123() const { return e123_; }

        double &operator[](size_t index) { return v_[index]; }
        double operator[](size_t index) const { return v_[index]; }

        //  Negation does not change anything for the AbsExt classes

        AbsExt4_3 &negate() { return *this; }

        [[nodiscard]] AbsExt4_3 negative() const { return AbsExt4_3(*this); }

        // A dual operation takes a k-vector and returns a (4-k)-vector
        // A reverse dual operation inverts the dual operation
        // dual(X,Y) is not safe for X=Y (same with revdual)
        //
        //  Dual and Reverse Dual are identical for the Abs... classes.

        AbsExt4_1 dual() const { return AbsExt4_1(e123_, e023_, e013_, e012_); }

        AbsExt4_1 reverse_dual() const { return AbsExt4_1(e123_, e023_, e013_, e012_); }

        // A meet takes a j-vector and a k-vector and returns a (j+k-4)-vector

        AbsExt4_2 meet(const AbsExt4_3 &rhs) const { return (dual().join(rhs.dual())).reverse_dual(); }

        AbsExt4_1 meet(const AbsExt4_2 &rhs) const { return (dual().join(rhs.dual())).reverse_dual(); }

        double inner(const AbsExt4_3 &rhs) const
        {
            double acc = 0.0;

            for (int i = 0; i < 4; i++)
            {
                acc += v_[i] * rhs.v_[i];
            }

            return (acc);
        }

        union
        {
            double v_[4];

            struct
            {
                double e012_;
                double e013_;
                double e023_;
                double e123_;
            };
        };

        friend class AbsExt4_2;
    };

    // A dual operation takes a k-vector and returns a (4-k)-vector
    // A reverse dual operation inverts the dual operation
    // dual(X,Y) is not safe for X=Y (same with revdual)

    inline AbsExt4_3 AbsExt4_1::dual() const { return AbsExt4_3(e3_, e2_, e1_, e0_); }

    inline AbsExt4_3 AbsExt4_1::reverse_dual() const { return AbsExt4_3(e3_, e2_, e1_, e0_); }

    inline AbsExt4_2 AbsExt4_1::join(const AbsExt4_1 &rhs) const
    {
        AbsExt4_2 result;

        result.e01_ = (e0_ * rhs.e1_) + (rhs.e0_ * e1_);
        result.e02_ = (e0_ * rhs.e2_) + (rhs.e0_ * e2_);
        result.e03_ = (e0_ * rhs.e3_) + (rhs.e0_ * e3_);
        result.e12_ = (e1_ * rhs.e2_) + (rhs.e1_ * e2_);
        result.e13_ = (e1_ * rhs.e3_) + (rhs.e1_ * e3_);
        result.e23_ = (e2_ * rhs.e3_) + (rhs.e2_ * e3_);

        return result;
    }



    inline AbsExt4_3 AbsExt4_2::join(const AbsExt4_1 &rhs) const
    {
        AbsExt4_3 result;

        result.e012_ = (e01_ * rhs.e2()) + (e02_ * rhs.e1()) + (e12_ * rhs.e0());
        result.e013_ = (e01_ * rhs.e3()) + (e03_ * rhs.e1()) + (e13_ * rhs.e0());
        result.e023_ = (e02_ * rhs.e3()) + (e03_ * rhs.e2()) + (e23_ * rhs.e0());
        result.e123_ = (e12_ * rhs.e3()) + (e13_ * rhs.e2()) + (e23_ * rhs.e1());

        return result;
    }

    inline AbsExt4_3 AbsExt4_1::join(const AbsExt4_2 &rhs) const { return rhs.join(*this); }

    // A meet takes a j-vector and a k-vector and returns a (j+k-4)-vector

    inline AbsExt4_1 AbsExt4_2::meet(const AbsExt4_3 &rhs) const { return (dual().join(rhs.dual())).reverse_dual(); }

}  // namespace ExteriorCalculusR4
