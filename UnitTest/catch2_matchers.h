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

#include <algorithm>
#include <boost/iostreams/device/mapped_file.hpp>
#include <catch2/matchers/catch_matchers_templated.hpp>
#include <filesystem>

class MatchesFile : public Catch::Matchers::MatcherGenericBase
{
   public:
    explicit MatchesFile(std::filesystem::__cxx11::path reference_file) : reference_file_(std::move(reference_file)) {}

    bool match(const std::filesystem::__cxx11::path& file_to_check) const
    {
        boost::iostreams::mapped_file_source test_file(file_to_check);
        boost::iostreams::mapped_file_source reference_file(reference_file_);

        return ((test_file.size() == reference_file.size()) &&
                std::equal(test_file.data(), test_file.data() + test_file.size(), reference_file.data()));      //  NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }

    std::string describe() const override
    {
        std::ostringstream ss;
        ss << "check the a file matches reference file: " << reference_file_ << std::endl;
        return ss.str();
    }

   private:
    const std::filesystem::__cxx11::path reference_file_;
};
