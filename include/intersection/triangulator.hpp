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
#include "math/Primitives.h"

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

        bool operator==(const Triangle& tri_to_compare) const
        {
            return (v0_ == tri_to_compare.v0_) && (v1_ == tri_to_compare.v1_) && (v2_ == tri_to_compare.v2_);
        }

        bool operator!=(const Triangle& tri_to_compare) const
        {
            return (v0_ != tri_to_compare.v0_) || (v1_ != tri_to_compare.v1_) || (v2_ != tri_to_compare.v2_);
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

    class NormalProjector
    {
       public:
        NormalProjector(const Math::Vector3D& v0, const Math::Vector3D& v1, const Math::Vector3D& v2)
        {
            Math::Vector3D normal = (v1 - v0).cross(v2 - v0);
            uint normdim = normal.abs().maxDim();
            proj_dim0_ = (normdim + 1) % 3;
            proj_dim1_ = (normdim + 2) % 3;
            sign_flip_ = (normal[normdim] < 0.0) ? -1.0 : 1.0;
            flip_sign_ = (sign_flip_ != 1.0 );
        }

        [[nodiscard]] uint32_t    proj_dim0() const { return proj_dim0_; }
        [[nodiscard]] uint32_t    proj_dim1() const { return proj_dim1_; }

        [[nodiscard]] double      sign_flip() const { return sign_flip_; }
        [[nodiscard]] bool        flip_sign() const { return flip_sign_; }

        private :

        uint32_t    proj_dim0_;
        uint32_t    proj_dim1_;

        double      sign_flip_;
        bool        flip_sign_;
    };

    enum class TriangulationResultCodes {
        SUCCESS = 0,

        TOO_MANY_POINTS,
        TOO_MANY_SEGMENTS,
        UNEQUAL_NUMBER_OF_INPUT_AND_OUTPUT_POINTS
    };

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

        inline void add_point(Point point)
        {
            points_[number_of_points_] = point.pair();
            point_markers_[number_of_points_++] = point.boundary() ? 1 : 0;
        }

        inline void add_point(double x, double y, bool boundary)
        {
            points_[number_of_points_].first = x;
            points_[number_of_points_].second = y;

            point_markers_[number_of_points_++] = boundary;
        }

        inline void add_point( const Math::Vector3D&    point, bool boundary, const NormalProjector& projector )
        {
            points_[number_of_points_].first = point[projector.proj_dim0()];
            points_[number_of_points_].second = projector.flip_sign() ? point[projector.proj_dim1()]  * projector.sign_flip() : point[projector.proj_dim1()];
            point_markers_[number_of_points_++] = boundary;
        }

        inline void add_segment(Segment segment)
        {
            segments_[number_of_segments_] = segment.pair();
            segment_markers_[number_of_segments_++] = segment.boundary() ? 1 : 0;
        }

        inline void add_segment(uint32_t start, uint32_t end, bool boundary)
        {
            segments_[number_of_segments_] = std::pair<int, int>(start, end);
            segment_markers_[number_of_segments_++] = boundary ? 1 : 0;
        }

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
