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

#include "CPPResult.hpp"
#include "math/normal_projector.hpp"
#include "primitives/primitives.hpp"
#include "result_codes.hpp"

namespace Cork::Triangulator
{
    class Point
    {
       public:
        Point(Vertex2D vertex, bool boundary) : x_(vertex.x()), y_(vertex.y()), boundary_(boundary) {}
        Point(double x, double y, bool boundary) : x_(x), y_(y), boundary_(boundary) {}

        [[nodiscard]] double x() const { return x_; }
        [[nodiscard]] double y() const { return y_; }
        [[nodiscard]] const std::pair<double, double>& pair() const { return reinterpret_cast<const std::pair<double, double>&>(x_); }
        [[nodiscard]] bool boundary() const { return boundary_; }

       private:
        const double x_;
        const double y_;

        bool boundary_;
    };

    class Segment
    {
       public:
        Segment(int start, int end, bool boundary) : start_(start), end_(end), boundary_(boundary) {}

        [[nodiscard]] int start() const { return start_; }
        [[nodiscard]] int end() const { return end_; }
        [[nodiscard]] const std::pair<int, int>& pair() const { return reinterpret_cast<const std::pair<int, int>&>(start_); }
        [[nodiscard]] bool boundary() const { return boundary_; }

       private:
        const int start_;
        const int end_;

        bool boundary_;
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
        static constexpr size_t MAX_POINTS = 2048;

        Triangulator()
            : number_of_points_(0), number_of_segments_(0), too_many_points_(false), too_many_segments_(false){};
        Triangulator(const Triangulator&) = delete;
        Triangulator(Triangulator&&) = delete;

        ~Triangulator() = default;

        Triangulator& operator=(const Triangulator&) = delete;
        Triangulator& operator=(Triangulator&&) = delete;

        [[nodiscard]] TriangulationResultCodes will_problem_fit(uint32_t num_points, uint32_t num_segments)
        {
            if (num_points >= MAX_POINTS)
            {
                return TriangulationResultCodes::TOO_MANY_POINTS;
            }

            if (num_segments >= MAX_POINTS)
            {
                return TriangulationResultCodes::TOO_MANY_SEGMENTS;
            }

            return TriangulationResultCodes::SUCCESS;
        }

        void reset()
        {
            number_of_points_ = 0;
            number_of_segments_ = 0;
            too_many_points_ = false;
            too_many_segments_ = false;
        }

        void add_point(Point point)
        {
            points_[number_of_points_] = point.pair();
            point_markers_[number_of_points_++] = point.boundary() ? 1 : 0;
        }

        void add_point(const Vertex2D& vertex, bool boundary)
        {
            points_[number_of_points_].first = vertex.x();
            points_[number_of_points_].second = vertex.y();

            point_markers_[number_of_points_++] = boundary;
        }

        void add_point(double x, double y, bool boundary)
        {
            points_[number_of_points_].first = x;
            points_[number_of_points_].second = y;

            point_markers_[number_of_points_++] = boundary;
        }

        void add_point(const Primitives::Vector3D& point, bool boundary, const Math::NormalProjector& projector)
        {
            points_[number_of_points_].first = point[projector.proj_dim0()];
            points_[number_of_points_].second = projector.flip_sign()
                                                    ? point[projector.proj_dim1()] * projector.sign_flip()
                                                    : point[projector.proj_dim1()];
            point_markers_[number_of_points_++] = boundary;
        }

        void add_segment(Segment segment)
        {
            segments_[number_of_segments_] = segment.pair();
            segment_markers_[number_of_segments_++] = segment.boundary() ? 1 : 0;
        }

        void add_segment(uint32_t start, uint32_t end, bool boundary)
        {
            segments_[number_of_segments_] = std::pair<int, int>(start, end);
            segment_markers_[number_of_segments_++] = boundary ? 1 : 0;
        }

        [[nodiscard]] const std::array<std::pair<double, double>, MAX_POINTS + 1>& points() const { return points_; }

        [[nodiscard]] TriangulateResult compute_triangulation();

       private:
        bool too_many_points_;
        bool too_many_segments_;
        uint32_t number_of_points_;
        uint32_t number_of_segments_;

        std::array<std::pair<double, double>, MAX_POINTS + 1> points_;
        std::array<int, MAX_POINTS + 1> point_markers_;
        std::array<std::pair<int, int>, MAX_POINTS + 1> segments_;
        std::array<int, MAX_POINTS + 1> segment_markers_;
    };
}  // namespace Cork::Triangulator
