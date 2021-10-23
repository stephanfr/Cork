#pragma once
// +-------------------------------------------------------------------------
// | triangulator.hpp
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

#include <stdint.h>

#include <memory>
#include <vector>

#include "CPPResult.hpp"

namespace Cork::Triangulator
{
    class Point
    {
       public:
        Point(double x, double y, bool boundary) : x_(x), y_(y), boundary_(boundary) {}

        double x() const { return x_; }
        double y() const { return y_; }
        const std::pair<double, double>& pair() const { return reinterpret_cast<const std::pair<double, double>&>(x_); }
        bool boundary() const { return boundary_; }

       private:
        const double x_;
        const double y_;

        bool boundary_;
    };

    class Segment
    {
       public:
        Segment(int start, int end, bool boundary) : start_(start), end_(end), boundary_(boundary) {}

        int start() const { return start_; }
        int end() const { return end_; }
        const std::pair<int, int>& pair() const { return reinterpret_cast<const std::pair<int, int>&>(start_); }
        bool boundary() const { return boundary_; }

       private:
        const int start_;
        const int end_;

        bool boundary_;
    };

    class Triangle
    {
       public:
        Triangle(int v0, int v1, int v2) : v0_(v0), v1_(v1), v2_(v2) {}

        bool operator==( const Triangle&    tri_to_compare ) const
        {
            return (v0_ == tri_to_compare.v0_ ) && (v1_ == tri_to_compare.v1_ ) && (v2_ == tri_to_compare.v2_ );
        }

        bool operator!=( const Triangle&    tri_to_compare ) const
        {
            return (v0_ != tri_to_compare.v0_ ) || (v1_ != tri_to_compare.v1_ ) || (v2_ != tri_to_compare.v2_ );
        }

        int v0() const { return v0_; }
        int v1() const { return v1_; }
        int v2() const { return v2_; }

        const std::array<int, 3>& tuple() const { return reinterpret_cast<const std::array<int, 3>&>(v1_); }

       private:
        const int v0_;
        const int v1_;
        const int v2_;
    };

    using TriangleList = std::vector<Triangle>;

    enum class TriangulationResultCodes
    {
        SUCCESS = 0,

        UNEQUAL_NUMBER_OF_INPUT_AND_OUTPUT_POINTS
    };

    using TriangulateResult = SEFUtility::ResultWithReturnUniquePtr<TriangulationResultCodes, TriangleList>;


    class TriangulatorIfx
    {
       public:
        virtual ~TriangulatorIfx() = default;

        virtual void add_point(Point point) = 0;
        virtual void add_point(double x, double y, bool boundary) = 0;

        virtual void add_segment(Segment edge) = 0;
        virtual void add_segment(uint32_t start, uint32_t end, bool boundary) = 0;

        virtual TriangulateResult compute_triangulation() = 0;

        static std::unique_ptr<TriangulatorIfx> get_triangulator();
    };
}  // namespace Cork::Triangulator
