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

#include <fmt/compile.h>
#include <fmt/format-inl.h>
#include <tbb/task_group.h>

#include <cstdio>
#include <fstream>
#include <iterator>
#include <sstream>

#ifdef __HAS_PERFORMANCE_STRINGSTREAM__
#include "util/performance_stringstream.hpp"
#endif

#include "file_formats/files.hpp"
#include "result_codes.hpp"
#include "util/file_helpers.hpp"

namespace Cork::Files
{
    WriteFileResult write_3d_polyline(const std::filesystem::path& file_path, const Writeable3DPolyline& polyline_to_write)
    {
        //	Open the output file

        std::ofstream out(file_path);

        if (!out.good())
        {
            return WriteFileResult::failure(WriteFileResultCodes::UNABLE_TO_OPEN_FILE,
                                            fmt::format("Unable to open file: {}", file_path.c_str()));
        }

        //  Write in a simple matrix format, simply vertex by vertex - one for each line.

        for( auto current_vertex : polyline_to_write.vertices() )
        {
            out << current_vertex.x() << " " << current_vertex.y() << " " << current_vertex.z() << std::endl;
        }

        //  Flush and return

        out.flush();

        return WriteFileResult::success();
    }
}  // namespace Cork::Files
