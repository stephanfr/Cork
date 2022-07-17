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

    enum class HasIntersection
    {
        NO = -1,
        MAYBE = 0,
        YES = 1
    };

    class ExactArithmeticContext
    {
       public:
        ExactArithmeticContext() : degeneracy_count_(0), exact_count_(0), call_count_(0) {}

        ExactArithmeticContext(const ExactArithmeticContext&) = delete;
        ExactArithmeticContext(ExactArithmeticContext&&) = delete;

        ExactArithmeticContext& operator=(const ExactArithmeticContext&) = delete;
        ExactArithmeticContext& operator=(ExactArithmeticContext&&) = delete;

        ~ExactArithmeticContext() = default;

        void reset_degeneracy_count() { degeneracy_count_ = 0; }
        [[nodiscard]] bool has_degeneracies() const { return degeneracy_count_ > 0; }
        void found_degeneracy() { degeneracy_count_++; }

        [[nodiscard]] uint32_t exact_count() const { return exact_count_; }
        void add_exact_computation() { exact_count_++; }

        [[nodiscard]] uint32_t call_count() const { return call_count_; }
        void add_call_count() { call_count_++; }

       private:
        std::atomic<uint32_t> degeneracy_count_ = 0;
        std::atomic<uint32_t> exact_count_ = 0;
        std::atomic<uint32_t> call_count_ = 0;
    };

    class IntersectingTriangle
    {
       public:
        IntersectingTriangle() = delete;

        IntersectingTriangle(const Primitives::Vector3D& tri0, const Primitives::Vector3D& tri1,
                             const Primitives::Vector3D& tri2)
            : p0_(tri0), p1_(tri1), p2_(tri2)
        {
        }

        [[nodiscard]] const Ext4_1& p0() const { return p0_; };
        [[nodiscard]] const Ext4_1& p1() const { return p1_; };
        [[nodiscard]] const Ext4_1& p2() const { return p2_; };

       private:
        const Ext4_1 p0_;
        const Ext4_1 p1_;
        const Ext4_1 p2_;
    };

    class IntersectingEdge
    {
       public:
        IntersectingEdge() = delete;

        IntersectingEdge(const Primitives::Vector3D& edge0, const Primitives::Vector3D& edge1) : p0_(edge0), p1_(edge1)
        {
        }

        [[nodiscard]] const Ext4_1& p0() const { return p0_; }
        [[nodiscard]] const Ext4_1& p1() const { return p1_; }

       private:
        const Ext4_1 p0_;
        const Ext4_1 p1_;
    };

    class TriangleEdgeIntersection
    {
       public:
        TriangleEdgeIntersection(const IntersectingTriangle&& triangle, const IntersectingEdge&& edge)
            : tri_(triangle), edge_(edge)
        {
        }

        [[nodiscard]] HasIntersection isEmpty(ExactArithmeticContext& context) const;

        [[nodiscard]] HasIntersection emptyExact(const Math::Quantizer& quantizer, ExactArithmeticContext& context) const;

        [[nodiscard]] Primitives::Vector3D coords() const;
        [[nodiscard]] Primitives::Vector3D coordsExact(const Math::Quantizer& quantizer) const;

       private:
        IntersectingTriangle tri_;
        IntersectingEdge edge_;

        [[nodiscard]] HasIntersection emptyFilter() const;

        [[nodiscard]] HasIntersection exactFallback(const Math::Quantizer& quantizer, ExactArithmeticContext& context) const;
    };

    Primitives::Vector3D coordsExact(const GMPExt4_2& edge, const GMPExt4_3& triangle,
                                     const Math::Quantizer& quantizer);

    class TriangleTriangleTriangleIntersection
    {
       public:
        TriangleTriangleTriangleIntersection(const IntersectingTriangle&& triangle0,
                                             const IntersectingTriangle&& triangle1,
                                             const IntersectingTriangle&& triangle2)
            : m_tri({{triangle0, triangle1, triangle2}})
        {
        }

        [[nodiscard]] const std::array<IntersectingTriangle, 3>& triangle() const { return (m_tri); }

        [[nodiscard]] HasIntersection isEmpty(ExactArithmeticContext& context) const;
        [[nodiscard]] HasIntersection emptyExact(const Math::Quantizer& quantizer, ExactArithmeticContext& context) const;

        [[nodiscard]] Primitives::Vector3D coords() const;
        [[nodiscard]] Primitives::Vector3D coordsExact(const Math::Quantizer& quantizer) const;

       private:
        std::array<IntersectingTriangle, 3> m_tri;

        [[nodiscard]] HasIntersection emptyFilter() const;

        [[nodiscard]] HasIntersection exactFallback(const Math::Quantizer& quantizer, ExactArithmeticContext& context) const;
    };

    Primitives::Vector3D coordsExact(const GMPExt4_3& triangle0, const GMPExt4_3& triangle1, const GMPExt4_3& triangle2,
                                     const Math::Quantizer& quantizer);

}  // namespace Cork::Empty3d
