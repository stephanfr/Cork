// +-------------------------------------------------------------------------
// | empty3d.h
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
#include <atomic>

#include "ext4.h"
//#include "absext4.h"
#include "fixext4.h"
#include "gmpext4.h"
#include "math/Primitives.h"
#include "quantization.h"

namespace Cork
{
    namespace Empty3d
    {
        class ExactArithmeticContext
        {
           public:
            std::atomic<int> degeneracy_count = 0;
            std::atomic<int> exact_count = 0;
            std::atomic<int> callcount = 0;
        };

        class TriIn
        {
           public:
            TriIn() = delete;

            TriIn(const Cork::Math::Vector3D& tri0, const Cork::Math::Vector3D& tri1, const Cork::Math::Vector3D& tri2)
                : p0_(tri0), p1_(tri1), p2_(tri2)
            {
            }

            const ExteriorCalculusR4::Ext4_1& p0() const { return p0_; };
            const ExteriorCalculusR4::Ext4_1& p1() const { return p1_; };
            const ExteriorCalculusR4::Ext4_1& p2() const { return p2_; };

           private:
            const ExteriorCalculusR4::Ext4_1 p0_;
            const ExteriorCalculusR4::Ext4_1 p1_;
            const ExteriorCalculusR4::Ext4_1 p2_;
        };

        class EdgeIn
        {
           public:
            EdgeIn() = delete;

            EdgeIn(const Cork::Math::Vector3D& edge0, const Cork::Math::Vector3D& edge1) : p0_(edge0), p1_(edge1) {}

            const ExteriorCalculusR4::Ext4_1& p0() const { return p0_; }
            const ExteriorCalculusR4::Ext4_1& p1() const { return p1_; }

           private:
            const ExteriorCalculusR4::Ext4_1 p0_;
            const ExteriorCalculusR4::Ext4_1 p1_;
        };

        class TriEdgeIn
        {
           public:
            TriEdgeIn(const TriIn&& triangle, const EdgeIn&& _edge) : tri(triangle), edge(_edge) {}

            bool isEmpty(ExactArithmeticContext& context) const;

            bool emptyExact(const Quantization::Quantizer& quantizer, ExactArithmeticContext& context) const;

            Cork::Math::Vector3D coords() const;
            Cork::Math::Vector3D coordsExact(const Quantization::Quantizer& quantizer) const;

           private:
            TriIn tri;
            EdgeIn edge;

            int emptyFilter() const;

            bool exactFallback(const Quantization::Quantizer& quantizer, ExactArithmeticContext& context) const;
        };

        Cork::Math::Vector3D coordsExact(const GMPExt4::GmpExt4_2& edge, const GMPExt4::GmpExt4_3& triangle,
                                         const Quantization::Quantizer& quantizer);

        class TriTriTriIn
        {
           public:
            TriTriTriIn(const TriIn&& triangle0, const TriIn&& triangle1, const TriIn&& triangle2)
                : m_tri({{triangle0, triangle1, triangle2}})
            {
            }

            const std::array<TriIn, 3>& triangle() const { return (m_tri); }

            bool isEmpty(ExactArithmeticContext& context) const;
            bool emptyExact(const Quantization::Quantizer& quantizer, ExactArithmeticContext& context) const;

            Cork::Math::Vector3D coords() const;
            Cork::Math::Vector3D coordsExact(const Quantization::Quantizer& quantizer) const;

           private:
            std::array<TriIn, 3> m_tri;

            int emptyFilter() const;

            bool exactFallback(const Quantization::Quantizer& quantizer, ExactArithmeticContext& context) const;
        };

        inline void toGmpExt(GMPExt4::GmpExt4_1& out, const Cork::Math::Vector3D& in,
                             const Quantization::Quantizer& quantizer)
        {
            out.e0 = quantizer.quantize2int(in.x());
            out.e1 = quantizer.quantize2int(in.y());
            out.e2 = quantizer.quantize2int(in.z());
            out.e3 = 1;
        }

        inline void toGmpExt(GMPExt4::GmpExt4_1& out, const ExteriorCalculusR4::Ext4_1& in,
                             const Quantization::Quantizer& quantizer)
        {
            out.e0 = quantizer.quantize2int(in.e0());
            out.e1 = quantizer.quantize2int(in.e1());
            out.e2 = quantizer.quantize2int(in.e2());
            out.e3 = 1;
        }

        Cork::Math::Vector3D coordsExact(const GMPExt4::GmpExt4_3& triangle0, const GMPExt4::GmpExt4_3& triangle1,
                                         const GMPExt4::GmpExt4_3& triangle2, const Quantization::Quantizer& quantizer);

    }  // namespace Empty3d

}  // namespace Cork
