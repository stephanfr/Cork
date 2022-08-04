// +-------------------------------------------------------------------------
// | fixext4.h
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

/*
 *
 *  FixExt4
 *
 *      Support for performing exterior calculus in R4 with exact fixed length integer values.
 *
 */

#include <iostream>

#include "ext4_base.hpp"
#include "fixint.hpp"
#include "quantization.hpp"
#include "vector_3D_template.hpp"

namespace Cork::Math::ExteriorCalculusR4
{
    template <int Nbits>
    class FixExt4_2;

    template <int Nbits>
    class FixExt4_3;

    template <int Nbits>
    class FixExt4_1 : public Bases::Ext4_1Base<FixInt::LimbInt<BITS_TO_LIMBS(Nbits)>>
    {
       public:
        using IntegerType = FixInt::LimbInt<BITS_TO_LIMBS(Nbits)>;
        using FixExt4_1Base = Bases::Ext4_1Base<FixInt::LimbInt<BITS_TO_LIMBS(Nbits)>>;

        FixExt4_1() = default;

        FixExt4_1(const FixExt4_1& ext_to_copy) : FixExt4_1Base(ext_to_copy) {}
        FixExt4_1(FixExt4_1&& ext_to_move) noexcept : FixExt4_1Base(ext_to_move) {}

        explicit FixExt4_1(const Vector3DTemplate<double>& vec, const Math::Quantizer& quantizer)
            : FixExt4_1Base(IntegerType(quantizer.quantize2int(vec.x())), IntegerType(quantizer.quantize2int(vec.y())),
                            IntegerType(quantizer.quantize2int(vec.z())), IntegerType(1))
        {
        }

        virtual ~FixExt4_1() = default;

        FixExt4_1& operator=(const FixExt4_1& ext_to_copy) = default;
        FixExt4_1& operator=(FixExt4_1&& ext_to_move) noexcept = default;

        // Negation takes a k-vector and returns its negation neg(X,Y) and is safe for X=Y

        const FixExt4_1& negate()
        {
            neg(this->e0_, this->e0_);
            neg(this->e1_, this->e1_);
            neg(this->e2_, this->e2_);
            neg(this->e3_, this->e3_);

            return *this;
        }

        [[nodiscard]] FixExt4_1 negative() const
        {
            FixExt4_1 result;

            neg(result.e0_, this->e0_);
            neg(result.e1_, this->e1_);
            neg(result.e2_, this->e2_);
            neg(result.e3_, this->e3_);

            return result;
        }

        [[nodiscard]] FixExt4_3<Nbits> dual() const;
        [[nodiscard]] FixExt4_3<Nbits> reverse_dual() const;

        template <int Nrhsbits>
        [[nodiscard]] FixExt4_2<(Nbits + Nrhsbits + 1)> join(const FixExt4_1<Nrhsbits>& rhs);

        template <int Nrhsbits>
        [[nodiscard]] typename FixInt::BitInt<(Nbits + Nrhsbits + 2)>::Rep inner(const FixExt4_1<Nrhsbits>& rhs)
        {
            typename FixInt::BitInt<(Nbits + Nrhsbits + 2)>::Rep result;

            typename FixInt::BitInt<(Nbits + Nrhsbits)>::Rep p0;
            mul(p0, this->e0_, rhs.e0_);
            typename FixInt::BitInt<(Nbits + Nrhsbits)>::Rep p1;
            mul(p1, this->e1_, rhs.e1_);
            typename FixInt::BitInt<(Nbits + Nrhsbits)>::Rep p2;
            mul(p2, this->e2_, rhs.e2_);
            typename FixInt::BitInt<(Nbits + Nrhsbits)>::Rep p3;
            mul(p3, this->e3_, rhs.e3_);
            typename FixInt::BitInt<(Nbits + Nrhsbits + 1)>::Rep a;
            add(a, p0, p1);
            typename FixInt::BitInt<(Nbits + Nrhsbits + 1)>::Rep b;

            add(b, p2, p3);
            add(result, a, b);

            return result;
        }

       protected:
        template <int N>
        friend class FixExt4_3;
    };

    template <int Nbits>
    class FixExt4_2 : public Bases::Ext4_2Base<FixInt::LimbInt<BITS_TO_LIMBS(Nbits)>>
    {
       public:
        using FixExt4_2Base = Bases::Ext4_2Base<FixInt::LimbInt<BITS_TO_LIMBS(Nbits)>>;

        FixExt4_2() = default;

        FixExt4_2(const FixExt4_2& ext_to_copy) : FixExt4_2Base(ext_to_copy) {}
        FixExt4_2(FixExt4_2&& ext_to_move) noexcept : FixExt4_2Base(ext_to_move) {}

        virtual ~FixExt4_2() = default;

        FixExt4_2& operator=(const FixExt4_2& ext_to_copy) = default;
        FixExt4_2& operator=(FixExt4_2&& ext_to_move) noexcept = default;

        FixExt4_2& negate()
        {
            neg(this->e01_, this->e01_);
            neg(this->e02_, this->e02_);
            neg(this->e03_, this->e03_);
            neg(this->e12_, this->e12_);
            neg(this->e13_, this->e13_);
            neg(this->e23_, this->e23_);

            return *this;
        }

        [[nodiscard]] FixExt4_2 negative() const
        {
            FixExt4_2 result;

            neg(result.e01_, this->e01_);
            neg(result.e02_, this->e02_);
            neg(result.e03_, this->e03_);
            neg(result.e12_, this->e12_);
            neg(result.e13_, this->e13_);
            neg(result.e23_, this->e23_);

            return result;
        }

        [[nodiscard]] FixExt4_2 dual() const
        {
            FixExt4_2 out;

            out.e01_ = this->e23_;
            neg(out.e02_, this->e13_);
            out.e03_ = this->e12_;
            out.e12_ = this->e03_;
            neg(out.e13_, this->e02_);
            out.e23_ = this->e01_;

            return out;
        }

        [[nodiscard]] FixExt4_2 reverse_dual() const
        {
            FixExt4_2 out;

            out.e01_ = this->e23_;
            neg(out.e02_, this->e13_);
            out.e03_ = this->e12_;
            out.e12_ = this->e03_;
            neg(out.e13_, this->e02_);
            out.e23_ = this->e01_;

            return out;
        }

        template <int Nrhsbits>
        [[nodiscard]] typename FixInt::BitInt<(Nbits + Nrhsbits + 3)>::Rep inner(const FixExt4_2<Nrhsbits>& rhs)
        {
            typename FixInt::BitInt<(Nbits + Nrhsbits)>::Rep p0;
            mul(p0, this->e01_, rhs.e01());
            typename FixInt::BitInt<(Nbits + Nrhsbits)>::Rep p1;
            mul(p1, this->e02_, rhs.e02());
            typename FixInt::BitInt<(Nbits + Nrhsbits)>::Rep p2;
            mul(p2, this->e03_, rhs.e03());
            typename FixInt::BitInt<(Nbits + Nrhsbits)>::Rep p3;
            mul(p3, this->e12_, rhs.e12());
            typename FixInt::BitInt<(Nbits + Nrhsbits)>::Rep p4;
            mul(p4, this->e13_, rhs.e13());
            typename FixInt::BitInt<(Nbits + Nrhsbits)>::Rep p5;
            mul(p5, this->e23_, rhs.e23());
            typename FixInt::BitInt<(Nbits + Nrhsbits + 1)>::Rep a;
            add(a, p0, p1);
            typename FixInt::BitInt<(Nbits + Nrhsbits + 1)>::Rep b;
            add(b, p2, p3);
            typename FixInt::BitInt<(Nbits + Nrhsbits + 1)>::Rep c;
            add(c, p4, p5);
            typename FixInt::BitInt<(Nbits + Nrhsbits + 2)>::Rep x;
            add(x, a, b);

            typename FixInt::BitInt<(Nbits + Nrhsbits + 3)>::Rep out;
            add(out, x, c);

            return out;
        }

        template <int Nrhsbits>
        FixExt4_3<(Nbits + Nrhsbits + 2)> join(const ExteriorCalculusR4::FixExt4_1<Nrhsbits>& rhs);

        template <int Nrhsbits>
        FixExt4_1<(Nbits + Nrhsbits + 2)> meet(const FixExt4_3<Nrhsbits>& rhs);

       protected:
        template <int N>
        friend class FixExt4_1;
    };

    template <int Nbits>
    class FixExt4_3 : public Bases::Ext4_3Base<FixInt::LimbInt<BITS_TO_LIMBS(Nbits)>>
    {
       public:
        using FixExt4_3Base = Bases::Ext4_3Base<FixInt::LimbInt<BITS_TO_LIMBS(Nbits)>>;

        FixExt4_3(){};

        FixExt4_3(const FixExt4_3& ext_to_copy) : FixExt4_3Base(ext_to_copy) {}
        FixExt4_3(FixExt4_3&& ext_to_move) noexcept : FixExt4_3Base(ext_to_move) {}

        virtual ~FixExt4_3() = default;

        FixExt4_3& operator=(const FixExt4_3& ext_to_copy) = default;
        FixExt4_3& operator=(FixExt4_3&& ext_to_move) noexcept = default;

        FixExt4_3& negate()
        {
            neg(this->e012_, this->e012_);
            neg(this->e013_, this->e013_);
            neg(this->e023_, this->e023_);
            neg(this->e123_, this->e123_);

            return *this;
        }

        [[nodiscard]] FixExt4_3 negative() const
        {
            FixExt4_3 result;

            neg(result.e012_, this->e012_);
            neg(result.e013_, this->e013_);
            neg(result.e023_, this->e023_);
            neg(result.e123_, this->e123_);

            return result;
        }

        [[nodiscard]] FixExt4_1<Nbits> dual() const
        {
            FixExt4_1<Nbits> out;

            out.e0_ = this->e123_;
            neg(out.e1_, this->e023_);
            out.e2_ = this->e013_;
            neg(out.e3_, this->e012_);

            return out;
        }

        [[nodiscard]] FixExt4_1<Nbits> reverse_dual() const
        {
            FixExt4_1<Nbits> result;

            neg(result.e0_, this->e123_);
            result.e1_ = this->e023_;
            neg(result.e2_, this->e013_);
            result.e3_ = this->e012_;

            return result;
        }

        template <int Nrhsbits>
        [[nodiscard]] FixExt4_2<(Nbits + Nrhsbits + 1)> meet(const FixExt4_3<Nrhsbits>& rhs) const
        {
            return (dual().join(rhs.dual())).reverse_dual();
        }

        template <int Nrhsbits>
        [[nodiscard]] FixExt4_1<(Nbits + Nrhsbits + 2)> meet(const FixExt4_2<Nrhsbits>& rhs)
        {
            return (dual().join(rhs.dual())).reverse_dual();
        }

        template <int Nrhsbits>
        [[nodiscard]] typename FixInt::BitInt<(Nbits + Nrhsbits + 2)>::Rep inner(const FixExt4_3<Nrhsbits>& rhs) const
        {
            typename FixInt::BitInt<(Nbits + Nrhsbits)>::Rep p0;
            mul(p0, this->e012_, rhs.e012());
            typename FixInt::BitInt<(Nbits + Nrhsbits)>::Rep p1;
            mul(p1, this->e013_, rhs.e013());
            typename FixInt::BitInt<(Nbits + Nrhsbits)>::Rep p2;
            mul(p2, this->e023_, rhs.e023());
            typename FixInt::BitInt<(Nbits + Nrhsbits)>::Rep p3;
            mul(p3, this->e123_, rhs.e123());
            typename FixInt::BitInt<(Nbits + Nrhsbits + 1)>::Rep a;
            add(a, p0, p1);
            typename FixInt::BitInt<(Nbits + Nrhsbits + 1)>::Rep b;
            add(b, p2, p3);

            typename FixInt::BitInt<(Nbits + Nrhsbits + 2)>::Rep out;

            add(out, a, b);

            return out;
        }

       protected:
        template <int N>
        friend class FixExt4_1;

        template <int N>
        friend class FixExt4_2;
    };

    template <int Nbits>
    inline FixExt4_3<Nbits> FixExt4_1<Nbits>::dual() const
    {
        FixExt4_3<Nbits> result;

        result.e012_ = this->e3_;
        neg(result.e013_, this->e2_);
        result.e023_ = this->e1_;
        neg(result.e123_, this->e0_);

        return result;
    }

    template <int Nbits>
    inline FixExt4_3<Nbits> FixExt4_1<Nbits>::reverse_dual() const
    {
        FixExt4_3<Nbits> result;

        neg(result.e012_, this->e3_);
        result.e013_ = this->e2_;
        neg(result.e023_, this->e1_);
        result.e123_ = this->e0_;

        return result;
    }

    template <int Nbits>
    template <int Nrhsbits>
    inline FixExt4_2<(Nbits + Nrhsbits + 1)> FixExt4_1<Nbits>::join(const FixExt4_1<Nrhsbits>& rhs)
    {
        typename FixInt::BitInt<(Nbits + Nrhsbits)>::Rep a;
        typename FixInt::BitInt<(Nbits + Nrhsbits)>::Rep b;

        FixExt4_2<(Nbits + Nrhsbits + 1)> out;

        mul(a, this->e0_, rhs.e1());
        mul(b, rhs.e0(), this->e1_);
        // cout << "a/b limbs: " << BITS_TO_LIMBS(Nlhs+Nrhs) << endl;
        // cout << "out bits: " << BITS_TO_LIMBS(Nlhs+Nrhs+1) << endl;
        // cout << "a,b: " << toString(a) << ',' << toString(b) << endl;

        sub(out.e01_, a, b);

        mul(a, this->e0_, rhs.e2());
        mul(b, rhs.e0(), this->e2_);
        sub(out.e02_, a, b);

        mul(a, this->e0_, rhs.e3());
        mul(b, rhs.e0(), this->e3_);
        sub(out.e03_, a, b);

        mul(a, this->e1_, rhs.e2());
        mul(b, rhs.e1(), this->e2_);
        sub(out.e12_, a, b);

        mul(a, this->e1_, rhs.e3());
        mul(b, rhs.e1(), this->e3_);
        sub(out.e13_, a, b);

        mul(a, this->e2_, rhs.e3());
        mul(b, rhs.e2(), this->e3_);
        sub(out.e23_, a, b);

        return out;
    }

    template <int Nbits>
    template <int Nrhsbits>
    inline FixExt4_3<(Nbits + Nrhsbits + 2)> FixExt4_2<Nbits>::join(const ExteriorCalculusR4::FixExt4_1<Nrhsbits>& rhs)
    {
        typename FixInt::BitInt<(Nbits + Nrhsbits)>::Rep a;
        typename FixInt::BitInt<(Nbits + Nrhsbits)>::Rep b;
        typename FixInt::BitInt<(Nbits + Nrhsbits)>::Rep c;
        typename FixInt::BitInt<(Nbits + Nrhsbits + 1)>::Rep x;

        FixExt4_3<(Nbits + Nrhsbits + 2)> out;

        mul(a, this->e01_, rhs.e2());
        mul(b, this->e02_, rhs.e1());
        mul(c, this->e12_, rhs.e0());

        sub(x, a, b);
        add(out.e012_, x, c);
        mul(a, this->e01_, rhs.e3());
        mul(b, this->e03_, rhs.e1());
        mul(c, this->e13_, rhs.e0());

        sub(x, a, b);
        add(out.e013_, x, c);
        mul(a, this->e02_, rhs.e3());
        mul(b, this->e03_, rhs.e2());
        mul(c, this->e23_, rhs.e0());

        sub(x, a, b);
        add(out.e023_, x, c);
        mul(a, this->e12_, rhs.e3());
        mul(b, this->e13_, rhs.e2());
        mul(c, this->e23_, rhs.e1());

        sub(x, a, b);
        add(out.e123_, x, c);

        return out;
    }

    template <int Nbits>
    template <int Nrhsbits>
    inline FixExt4_1<(Nbits + Nrhsbits + 2)> FixExt4_2<Nbits>::meet(const FixExt4_3<Nrhsbits>& rhs)
    {
        return (dual().join(rhs.dual())).reverse_dual();
    }

}  // namespace Cork::Math::ExteriorCalculusR4
