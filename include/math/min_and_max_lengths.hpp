// +-------------------------------------------------------------------------
// | min_and_max_lengths.hpp
// |
// | Author: Stephan Friedl
// +-------------------------------------------------------------------------
// | COPYRIGHT:
// |    Copyright Stephan Friedl 2015
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

#include <cmath>
#include <limits>

namespace Cork::Math
{
    class MinAndMaxLengths
    {
       public:
        static MinAndMaxLengths from_squares(double min_squared, double max_squared)
        {
            return MinAndMaxLengths(min_squared, max_squared);
        }

        MinAndMaxLengths()
            : min_squared_(std::numeric_limits<double>::max()), max_squared_(std::numeric_limits<double>::min())
        {
        }

        MinAndMaxLengths(const MinAndMaxLengths& obj_to_copy)
            : min_squared_(obj_to_copy.min_squared_), max_squared_(obj_to_copy.max_squared_)
        {
        }

        ~MinAndMaxLengths() = default;

        double min() const { return sqrt(min_squared_); }

        double max() const { return sqrt(max_squared_); }

        void update(const MinAndMaxLengths& second)
        {
            min_squared_ = std::min(min_squared_, second.min_squared_);
            max_squared_ = std::max(max_squared_, second.max_squared_);
        }

        void update_with_squares(double second_min_squared, double second_max_squared)
        {
            min_squared_ = std::min(min_squared_, second_min_squared);
            max_squared_ = std::max(max_squared_, second_max_squared);
        }

       private:
        double min_squared_;
        double max_squared_;

        MinAndMaxLengths(double min_squared, double max_squared) : min_squared_(min_squared), max_squared_(max_squared)
        {
        }
    };
}  // namespace Cork::Math
