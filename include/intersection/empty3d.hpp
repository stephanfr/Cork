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

#include "math/ext4.hpp"
#include "math/fixext4.hpp"
#include "math/gmpext4.hpp"
#include "primitives/primitives.hpp"


namespace Cork::Empty3d
{
    using Ext4_1 = Math::ExteriorCalculusR4::Ext4_1;
    using Ext4_2 = Math::ExteriorCalculusR4::Ext4_2;
    using Ext4_3 = Math::ExteriorCalculusR4::Ext4_3;

    using GMPExt4_1 = Math::ExteriorCalculusR4::GMPExt4_1;
    using GMPExt4_2 = Math::ExteriorCalculusR4::GMPExt4_2;
    using GMPExt4_3 = Math::ExteriorCalculusR4::GMPExt4_3;

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

        TriIn(const Primitives::Vector3D& tri0, const Primitives::Vector3D& tri1, const Primitives::Vector3D& tri2)
            : p0_(tri0), p1_(tri1), p2_(tri2)
        {
        }

        const Ext4_1& p0() const { return p0_; };
        const Ext4_1& p1() const { return p1_; };
        const Ext4_1& p2() const { return p2_; };

       private:
        const Ext4_1 p0_;
        const Ext4_1 p1_;
        const Ext4_1 p2_;
    };

    class EdgeIn
    {
       public:
        EdgeIn() = delete;

        EdgeIn(const Primitives::Vector3D& edge0, const Primitives::Vector3D& edge1) : p0_(edge0), p1_(edge1) {}

        const Ext4_1& p0() const { return p0_; }
        const Ext4_1& p1() const { return p1_; }

       private:
        const Ext4_1 p0_;
        const Ext4_1 p1_;
    };

    class TriEdgeIn
    {
       public:
        TriEdgeIn(const TriIn&& triangle, const EdgeIn&& _edge) : tri(triangle), edge(_edge) {}

        bool isEmpty(ExactArithmeticContext& context) const;

        bool emptyExact(const Math::Quantizer& quantizer, ExactArithmeticContext& context) const;

        Primitives::Vector3D coords() const;
        Primitives::Vector3D coordsExact(const Math::Quantizer& quantizer) const;

       private:
        TriIn tri;
        EdgeIn edge;

        int emptyFilter() const;

        bool exactFallback(const Math::Quantizer& quantizer, ExactArithmeticContext& context) const;
    };

    Primitives::Vector3D coordsExact(const GMPExt4_2& edge, const GMPExt4_3& triangle,
                                     const Math::Quantizer& quantizer);

    class TriTriTriIn
    {
       public:
        TriTriTriIn(const TriIn&& triangle0, const TriIn&& triangle1, const TriIn&& triangle2)
            : m_tri({{triangle0, triangle1, triangle2}})
        {
        }

        const std::array<TriIn, 3>& triangle() const { return (m_tri); }

        bool isEmpty(ExactArithmeticContext& context) const;
        bool emptyExact(const Math::Quantizer& quantizer, ExactArithmeticContext& context) const;

        Primitives::Vector3D coords() const;
        Primitives::Vector3D coordsExact(const Math::Quantizer& quantizer) const;

       private:
        std::array<TriIn, 3> m_tri;

        int emptyFilter() const;

        bool exactFallback(const Math::Quantizer& quantizer, ExactArithmeticContext& context) const;
    };

    Primitives::Vector3D coordsExact(const GMPExt4_3& triangle0, const GMPExt4_3& triangle1, const GMPExt4_3& triangle2,
                                     const Math::Quantizer& quantizer);

}  // namespace Cork::Empty3d
