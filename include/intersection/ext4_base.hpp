#pragma once

#include "CorkDefs.h"
#include "math/Primitives.h"

namespace Cork::ExteriorCalculusR4
{
    namespace Constants
    {
        constexpr double DOUBLE_ONE = 1.0;
    }

    namespace Bases
    {
        template <typename T>
        class Ext4_1Base
        {
           public:
            ~Ext4_1Base() = default;

            [[nodiscard]] T e0() const { return e0_; }
            [[nodiscard]] T e1() const { return e1_; }
            [[nodiscard]] T e2() const { return e2_; }
            [[nodiscard]] T e3() const { return e3_; }

            [[nodiscard]] T &operator[](size_t index)
            {
                return gsl::at(reinterpret_cast<std::array<T, 4> &>(e0_), index);
            }
            [[nodiscard]] T operator[](size_t index) const
            {
                return gsl::at(reinterpret_cast<std::array<T, 4> &>(e0_), index);
            }

            [[nodiscard]] bool operator==(const Ext4_1Base &r4_to_compare) const
            {
                return ((e0_ == r4_to_compare.e0_) && (e1_ == r4_to_compare.e1_) && (e2_ == r4_to_compare.e2_) &&
                        (e3_ == r4_to_compare.e3_));
            }

            [[nodiscard]] bool operator!=(const Ext4_1Base &r4_to_compare) const
            {
                return ((e0_ != r4_to_compare.e0_) || (e1_ != r4_to_compare.e1_) || (e2_ != r4_to_compare.e2_) ||
                        (e3_ != r4_to_compare.e3_));
            }

#if defined(NUMERIC_PRECISION) && NUMERIC_PRECISION == double

            operator Math::Vector3D() const
            {
                assert(e3_ != 0);  //  Trap any divisions by zero!

                Math::Vector3D vec(reinterpret_cast<const Math::Vector3D &>(e0_));
                vec /= e3_;

                return vec;
            }
#else

            [[nodiscard]] Math::Vector3D operator() const
            {
                // Warning: beware of division by zero!

                return Math::Vector3D((NUMERIC_PRECISION)(e0_ / e3_), (NUMERIC_PRECISION)(e1_ / e3_),
                                      (NUMERIC_PRECISION)(e2_ / e3_));
            }

#endif

            //  The inner product is the familiar dot product.

            [[nodiscard]] T inner(const Ext4_1Base &rhs) const
            {
                T acc = e0_ * rhs.e0_;
                acc += e1_ * rhs.e1_;
                acc += e2_ * rhs.e2_;
                acc += e3_ * rhs.e3_;

                return (acc);
            }

           protected:
            Ext4_1Base() = default;

            Ext4_1Base(T e0, T e1, T e2, T e3) : e0_(e0), e1_(e1), e2_(e2), e3_(e3){};

            explicit Ext4_1Base(const std::array<T, 4> &array_to_copy)
                : e0_(array_to_copy[0]), e1_(array_to_copy[1]), e2_(array_to_copy[2]), e3_(array_to_copy[3])
            {
            }

            explicit Ext4_1Base(const Ext4_1Base &ext_to_copy)
                : e0_(ext_to_copy.e0_), e1_(ext_to_copy.e1_), e2_(ext_to_copy.e2_), e3_(ext_to_copy.e3_)
            {
            }

            alignas(SIMD_MEMORY_ALIGNMENT) T e0_;
            T e1_;
            T e2_;
            T e3_;

            template <typename T1>
            friend std::ostream &operator<<(std::ostream &out, const Ext4_1Base<T1> &ext);
        };

        template <typename T>
        class Ext4_2Base
        {
           public:
            //  Operators

            [[nodiscard]] T &operator[](size_t index)
            {
                return gsl::at(reinterpret_cast<std::array<T, 6> &>(e01_), index);
            }
            [[nodiscard]] T operator[](size_t index) const
            {
                return gsl::at(reinterpret_cast<std::array<T, 6> &>(e01_), index);
            }

            [[nodiscard]] T e01() const { return e01_; }
            [[nodiscard]] T e02() const { return e02_; }
            [[nodiscard]] T e03() const { return e03_; }
            [[nodiscard]] T e12() const { return e12_; }
            [[nodiscard]] T e13() const { return e13_; }
            [[nodiscard]] T e23() const { return e23_; }

            [[nodiscard]] bool operator==(const Ext4_2Base &r4_to_compare) const
            {
                return ((e01_ == r4_to_compare.e01_) && (e02_ == r4_to_compare.e02_) && (e03_ == r4_to_compare.e03_) &&
                        (e12_ == r4_to_compare.e12_) && (e13_ == r4_to_compare.e13_) && (e23_ == r4_to_compare.e23_));
            }

            [[nodiscard]] bool operator!=(const Ext4_2Base &r4_to_compare) const
            {
                return ((e01_ != r4_to_compare.e01_) || (e02_ != r4_to_compare.e02_) || (e03_ != r4_to_compare.e03_) ||
                        (e12_ != r4_to_compare.e12_) || (e13_ != r4_to_compare.e13_) || (e23_ != r4_to_compare.e23_));
            }

            // An inner product takes two k-vectors and produces a single number

            [[nodiscard]] T inner(const Ext4_2Base &rhs) const
            {
                T acc = e01_ * rhs.e01_;
                acc += e02_ * rhs.e02_;
                acc += e03_ * rhs.e03_;
                acc += e12_ * rhs.e12_;
                acc += e13_ * rhs.e13_;
                acc += e23_ * rhs.e23_;

                return (acc);
            }

           protected:
            Ext4_2Base() = default;

            Ext4_2Base(T e01, T e02, T e03, T e12, T e13, T e23)
                : e01_(e01), e02_(e02), e03_(e03), e12_(e12), e13_(e13), e23_(e23)
            {
            }

            alignas(SIMD_MEMORY_ALIGNMENT) T e01_;
            T e02_;
            T e03_;
            T e12_;
            T e13_;
            T e23_;

            template <typename T1>
            friend std::ostream &operator<<(std::ostream &out, const Ext4_2Base<T1> &ext);
        };

        template <typename T>
        class Ext4_3Base
        {
           public:
            //  Accessors

            [[nodiscard]] T e012() const { return e012_; }
            [[nodiscard]] T e013() const { return e013_; }
            [[nodiscard]] T e023() const { return e023_; }
            [[nodiscard]] T e123() const { return e123_; }

            [[nodiscard]] T &operator[](size_t index)
            {
                return gsl::at(reinterpret_cast<std::array<T, 4> &>(e012_), index);
            }
            [[nodiscard]] T operator[](size_t index) const
            {
                return gsl::at(reinterpret_cast<std::array<T, 4> &>(e012_), index);
            }

            [[nodiscard]] bool operator==(const Ext4_3Base &r4_to_compare) const
            {
                return ((e012_ == r4_to_compare.e012_) && (e013_ == r4_to_compare.e013_) &&
                        (e023_ == r4_to_compare.e023_) && (e123_ == r4_to_compare.e123_));
            }

            [[nodiscard]] bool operator!=(const Ext4_3Base &r4_to_compare) const
            {
                return ((e012_ != r4_to_compare.e012_) || (e013_ != r4_to_compare.e013_) ||
                        (e023_ != r4_to_compare.e023_) || (e123_ != r4_to_compare.e123_));
            }

            //  Dot product

            [[nodiscard]] T inner(const Ext4_3Base &rhs) const
            {
                T acc = e012_ * rhs.e012_;
                acc += e013_ * rhs.e013_;
                acc += e023_ * rhs.e023_;
                acc += e123_ * rhs.e123_;

                return (acc);
            }

           protected:
            Ext4_3Base() = default;
            Ext4_3Base(T e012, T e013, T e023, T e123) : e012_(e012), e013_(e013), e023_(e023), e123_(e123) {}

            alignas(SIMD_MEMORY_ALIGNMENT) T e012_;
            T e013_;
            T e023_;
            T e123_;

            template <typename T1>
            friend std::ostream &operator<<(std::ostream &out, const Ext4_3Base<T1> &ext);
        };

        template <typename T>
        std::ostream &operator<<(std::ostream &out, const Ext4_1Base<T> &ext)
        {
            return out << '[' << ext.e0_ << ',' << ext.e1_ << ',' << ext.e2_ << ',' << ext.e3_ << ']';
        }

        template <typename T>
        std::ostream &operator<<(std::ostream &out, const Ext4_2Base<T> &ext)
        {
            return out << '[' << ext.e01_ << ',' << ext.e02_ << ',' << ext.e03_ << ',' << ext.e12_ << ',' << ext.e13_
                       << ',' << ext.e23_ << ']';
        }

        template <typename T>
        std::ostream &operator<<(std::ostream &out, const Ext4_3Base<T> &ext)
        {
            return out << '[' << ext.e012_ << ',' << ext.e013_ << ',' << ext.e023_ << ',' << ext.e123_ << ']';
        }
    }  // namespace Bases

    using Ext4_1Base = Bases::Ext4_1Base<double>;
    using AbsExt4_1Base = Bases::Ext4_1Base<double>;

    using Ext4_2Base = Bases::Ext4_2Base<double>;
    using AbsExt4_2Base = Bases::Ext4_2Base<double>;

    using Ext4_3Base = Bases::Ext4_3Base<double>;
    using AbsExt4_3Base = Bases::Ext4_3Base<double>;

}  // namespace Cork::ExteriorCalculusR4
