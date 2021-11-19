/*
 Copyright (c) 2021 Stephan Friedl

 Permission is hereby granted, free of charge, to any person obtaining a copy of
 this software and associated documentation files (the "Software"), to deal in
 the Software without restriction, including without limitation the rights to
 use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 the Software, and to permit persons to whom the Software is furnished to do so,
 subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#pragma once

#include <streambuf>

#include <fmt/compile.h>
#include <fmt/format-inl.h>

namespace SEFUtility
{
    class GrowableStreambuf : public std::streambuf
    {
       public:
        GrowableStreambuf(size_t initial_size) { buffer_.reserve(initial_size); }

        ~GrowableStreambuf() {}

        std::streamsize xsputn(const char_type* s, std::streamsize count) { return buffer_.append(s, count), count; }

        int overflow(int_type character) { return 1; }

       protected:
        std::string buffer_;
    };

    class PerformanceOStringStream : protected GrowableStreambuf, public std::ostream
    {
       public:
        PerformanceOStringStream(size_t initial_size) : GrowableStreambuf(initial_size), std::ostream(this) {}

        std::back_insert_iterator<std::string>      back_insert_iterator(){ return std::back_insert_iterator( buffer_ ); }

        const std::string& str() const { return buffer_; }

        template <typename T>
        PerformanceOStringStream& operator<<(T&& value)
        {
            using base_reference = std::add_lvalue_reference_t<std::ostream>;
            return static_cast<base_reference>(*this) << std::forward<T>(value), *this;
        }

        template <int N>
        PerformanceOStringStream& operator<<(char (&value)[N])
        {
            xsputn(value, N - 1);
            return *this;
        }

        template <int N>
        PerformanceOStringStream& operator<<(char const (&value)[N])
        {
            xsputn(value, N - 1);
            return *this;
        }
    };

    template <>
    inline PerformanceOStringStream& PerformanceOStringStream::operator<<(char*&& value)
    {
        xsputn(value, strlen(value));
        return (*this);
    }

    template <>
    inline PerformanceOStringStream& PerformanceOStringStream::operator<<(char const*&& value)
    {
        xsputn(value, strlen(value));
        return (*this);
    }

    template <>
    inline PerformanceOStringStream& PerformanceOStringStream::operator<<(std::string& value)
    {
        xsputn(value.c_str(), value.length());
        return (*this);
    }

    template <>
    inline PerformanceOStringStream& PerformanceOStringStream::operator<<(std::string const& value)
    {
        xsputn(value.c_str(), value.length());
        return (*this);
    }

    template <>
    inline PerformanceOStringStream& PerformanceOStringStream::operator<<(int32_t& value)
    {
        fmt::format_to( std::back_insert_iterator(buffer_), FMT_COMPILE( "{}" ), value );

        return (*this);
    }

    template <>
    inline PerformanceOStringStream& PerformanceOStringStream::operator<<(int32_t const& value)
    {
        fmt::format_to( std::back_insert_iterator(buffer_), FMT_COMPILE( "{}" ), value );

        return (*this);
    }

    template <>
    inline PerformanceOStringStream& PerformanceOStringStream::operator<<(uint32_t& value)
    {
        fmt::format_to( std::back_insert_iterator(buffer_), FMT_COMPILE( "{}" ), value );

        return (*this);
    }

    template <>
    inline PerformanceOStringStream& PerformanceOStringStream::operator<<(uint32_t const& value)
    {
        fmt::format_to( std::back_insert_iterator(buffer_), FMT_COMPILE( "{}" ), value );

        return (*this);
    }

    template <>
    inline PerformanceOStringStream& PerformanceOStringStream::operator<<(double& value)
    {
        fmt::format_to( std::back_insert_iterator(buffer_), FMT_COMPILE( "{}" ), value );

        return (*this);
    }

    template <>
    inline PerformanceOStringStream& PerformanceOStringStream::operator<<(double const& value)
    {
        fmt::format_to( std::back_insert_iterator(buffer_), FMT_COMPILE( "{}" ), value );

        return (*this);
    }

}  // namespace SEFUtility
