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

#include <cstdint>

#include "../constants.hpp"

#include "boost/container/small_vector.hpp"

#include "CPPResult.hpp"
#include "math/normal_projector.hpp"
#include "primitives/primitives.hpp"
#include "result_codes.hpp"

namespace Cork::Triangulator
{
    class Point
    {
       public:
        Point() = delete;
        Point(const Vertex2D& vertex) : x_(vertex.x()), y_(vertex.y()) {}
        Point(double x, double y) : x_(x), y_(y) {}

        [[nodiscard]] double x() const { return x_; }
        [[nodiscard]] double y() const { return y_; }

       private:
        double x_;
        double y_;
    };

    class Segment
    {
       public:
        Segment() = delete;
        Segment(int start, int end) : start_(start), end_(end) {}

        [[nodiscard]] int start() const { return start_; }
        [[nodiscard]] int end() const { return end_; }

       private:
        int start_;
        int end_;
    };

    class Triangle
    {
       public:
        Triangle(int v0, int v1, int v2) : v0_(v0), v1_(v1), v2_(v2) {}

        bool operator==(const Triangle& tri_to_compare) const
        {
            return (v0_ == tri_to_compare.v0_) && (v1_ == tri_to_compare.v1_) && (v2_ == tri_to_compare.v2_);
        }

        bool operator!=(const Triangle& tri_to_compare) const
        {
            return (v0_ != tri_to_compare.v0_) || (v1_ != tri_to_compare.v1_) || (v2_ != tri_to_compare.v2_);
        }

        [[nodiscard]] int v0() const { return v0_; }
        [[nodiscard]] int v1() const { return v1_; }
        [[nodiscard]] int v2() const { return v2_; }

       private:
        const int v0_;
        const int v1_;
        const int v2_;
    };

    using TriangleList = std::vector<Triangle>;

    using TriangulateResult = SEFUtility::ResultWithReturnUniquePtr<TriangulationResultCodes, TriangleList>;

    class Triangulator
    {
       public:

        Triangulator() = default;
        Triangulator(const Triangulator&) = delete;
        Triangulator(Triangulator&&) = delete;

        ~Triangulator() = default;

        Triangulator& operator=(const Triangulator&) = delete;
        Triangulator& operator=(Triangulator&&) = delete;


        void add_point(const Vertex2D& vertex, bool boundary)
        {
            points_.emplace_back( vertex );

            point_markers_.emplace_back( static_cast<int>(boundary) );
        }

        void add_point(double x, double y, bool boundary)
        {
            points_.emplace_back( x, y );

            point_markers_.emplace_back( static_cast<int>(boundary) );
        }

        void add_point(const Primitives::Vector3D& point, bool boundary, const Math::NormalProjector& projector)
        {
            points_.emplace_back( point[projector.proj_dim0()], projector.flip_sign()
                                                    ? point[projector.proj_dim1()] * projector.sign_flip()
                                                    : point[projector.proj_dim1()] );

            point_markers_.emplace_back( static_cast<int>(boundary) );
        }

        void add_segment(uint32_t start, uint32_t end, bool boundary)
        {
            segments_.emplace_back(start, end);
            segment_markers_.emplace_back( boundary ? 1 : 0 );
        }

        [[nodiscard]] TriangulateResult compute_triangulation();

       private:

        boost::container::small_vector<Point, MAX_TRIANGULATION_POINTS> points_;
        boost::container::small_vector<int, MAX_TRIANGULATION_POINTS> point_markers_;
        boost::container::small_vector<Segment, MAX_TRIANGULATION_POINTS> segments_;
        boost::container::small_vector<int, MAX_TRIANGULATION_POINTS> segment_markers_;
    };
}  // namespace Cork::Triangulator
