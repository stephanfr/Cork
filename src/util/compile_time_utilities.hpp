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

namespace SEFUtility::CompileTime
{
    class conststr
    {
       public:
        template <std::size_t N>
        constexpr conststr(const char (&a)[N]) : p_(a), sz_(N - 1)
        {}

        //  In C++11, constexpr expressions signal errors by throwing exceptions from the conditional operator ?:

        constexpr char operator[](std::size_t n) const { return n < sz_ ? p_[n] : throw std::out_of_range(""); }
        constexpr operator const char*() { return p_; }
        constexpr std::size_t size() const { return sz_; }

       private:
        const char* p_;
        std::size_t sz_;
    };

    constexpr unsigned count_char_occurances(conststr str, char c, unsigned i = 0, unsigned ans = 0)
    {
        return i == str.size() ? ans
                               : str[i] == c ? count_char_occurances(str, c, i + 1, ans + 1)
                                             : count_char_occurances(str, c, i + 1, ans);
    }
}  // namespace SEFUtility::CompileTime
