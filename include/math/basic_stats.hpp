// Copyright (c) 2021 Stephan Friedl
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files (the "Software"), to deal in
// the Software without restriction, including without limitation the rights to
// use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
// the Software, and to permit persons to whom the Software is furnished to do so,
// subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
// IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
// CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#pragma once

#include <cstdint>
#include <utility>

namespace Cork::Math
{
    class Statistics
    {
       public:
        template <typename T>
        Statistics(T itr_start, T itr_end)
        {
            sum_ = 0;
            count_ = 0;

            for (T itr = itr_start; itr != itr_end; itr++)
            {
                sum_ += *itr;

                range_.first = std::min(range_.first, *itr);
                range_.second = std::max(range_.second, *itr);

                count_++;
            }

            mean_ = sum_ / count_;

            standard_deviation_ = 0;

            for (T itr = itr_start; itr != itr_end; itr++)
            {
                standard_deviation_ += (mean_ - *itr) * (mean_ - *itr);
            }

            standard_deviation_ = sqrt(standard_deviation_ / count_);
        }

        uint32_t count() const { return count_; }

        double sum() const { return sum_; }

        std::pair<double, double> range() const { return range_; }

        double standard_deviation() const { return standard_deviation_; }

       private:
        uint32_t count_;
        double sum_;
        double mean_;
        std::pair<double, double> range_;
        double standard_deviation_;
    };
}  // namespace Math::BasicStats