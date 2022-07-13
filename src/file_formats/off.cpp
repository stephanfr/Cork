// +-------------------------------------------------------------------------
// | off.cpp
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

#include "result_codes.hpp"
#include "files.hpp"
#include "util/file_helpers.hpp"

namespace Cork::Files
{
    //  Define some local symbols to cur through some of the namespacing

    using IncrementalVertexIndexTriangleMeshBuilder = Cork::Meshes::IncrementalVertexIndexTriangleMeshBuilder;
    using TriangleMeshBuilderResultCodes = Cork::Meshes::TriangleMeshBuilderResultCodes;

    constexpr uint32_t INITIAL_WRITE_BUFFER_SIZE = 131072;

    inline std::istream& operator>>(std::istream& inStream, Primitives::TriangleByIndices& triToRead)
    {
        return (inStream >> triToRead[0] >> triToRead[1] >> triToRead[2]);
    }

    inline SEFUtility::PerformanceOStringStream& operator<<(SEFUtility::PerformanceOStringStream& out_stream,
                                                            const Primitives::TriangleByIndices& triToWrite)
    {
        fmt::format_to(out_stream.back_insert_iterator(), FMT_COMPILE("3 {:d} {:d} {:d}"),
                       VertexIndex::integer_type(triToWrite[0]), VertexIndex::integer_type(triToWrite[1]),
                       VertexIndex::integer_type(triToWrite[2]));

        return (out_stream);
    }

    inline SEFUtility::PerformanceOStringStream& WriteVertex(SEFUtility::PerformanceOStringStream& out_stream,
                                                             const Primitives::Vertex3D& vertex)
    {
        fmt::format_to(out_stream.back_insert_iterator(), FMT_COMPILE("{:g} {:g} {:g}"), vertex.x(), vertex.y(),
                       vertex.z());

        return out_stream;
    }

    inline SEFUtility::PerformanceOStringStream& operator<<(SEFUtility::PerformanceOStringStream& out_stream,
                                                            const Primitives::Vector3D& vec)
    {
        fmt::format_to(out_stream.back_insert_iterator(), FMT_COMPILE("{:g} {:g} {:g}"), vec.x(), vec.y(), vec.z());

        return out_stream;
    }

    //
    //  Read an OFF or COFF encoded file
    //

    ReadFileResult readOFF(const std::filesystem::path& file_path)
    {
        //	Open the mesh file

        LineByLineFileReader input_file(file_path, LineByLineFileReader::DEFAULT_BUFFER_SIZE, "#");

        if (!input_file.good())
        {
            return ReadFileResult::failure(ReadFileResultCodes::UNABLE_TO_OPEN_FILE,
                                           fmt::format("Unable to open file: {}", file_path.string()));
        }

        //	Look for the label at the top of the file to indicate formatting of the rest of the file
        //		If the encoding is in the COFF format which includes color info, we will have to strip the color values

        constexpr int STRING_BUFFER_LENGTH = 64;

        std::array<char, STRING_BUFFER_LENGTH> string_buffer;  //  NOLINT

        int items_processed(0);
        int chars_processed(0);

        std::string next_line;

        if (!input_file.read_line_exactly("%60s", string_buffer.data()))
        {
            return ReadFileResult::failure(ReadFileResultCodes::ERROR_READING_FILE_TYPE,
                                           "Error reading file type in OFF file header");
        }

        const std::string file_type(string_buffer.data());

        if ((file_type != "OFF") && (file_type != "COFF"))
        {
            return ReadFileResult::failure(ReadFileResultCodes::OFF_UNRECOGNIZED_HEADER,
                                           fmt::format("Unrecognized header for OFF file: {}", file_type));
        }

        bool strip_color = (file_type == "COFF");

        //	Load the number of vertices, faces and edges

        uint32_t num_vertices(0);
        uint32_t num_faces(0);
        uint32_t num_edges(0);

        if (!input_file.read_line_exactly("%d %d %d", &num_vertices, &num_faces, &num_edges))
        {
            return ReadFileResult::failure(ReadFileResultCodes::OFF_ERROR_READING_COUNTS,
                                           "Error reading counts of vertices, faces and edges.");
        }

        //	Get an incremental indexed vertices mesh builder

        std::unique_ptr<IncrementalVertexIndexTriangleMeshBuilder> meshBuilder(
            IncrementalVertexIndexTriangleMeshBuilder::GetBuilder(num_vertices, num_faces));

        //	Read the Vertex data

        {
            double x(0.0);
            double y(0.0);
            double z(0.0);

            for (unsigned int i = 0; i < num_vertices; ++i)
            {
                if (!strip_color)
                {
                    if (!input_file.read_line_exactly("%lg %lg %lg", &x, &y, &z))
                    {
                        return ReadFileResult::failure(ReadFileResultCodes::OFF_ERROR_READING_VERTICES,
                                                       "Error reading vertices.");
                    }
                }
                else
                {
                    int32_t r(0);
                    int32_t b(0);
                    int32_t g(0);
                    int32_t lum(0);

                    if (!input_file.read_line_exactly("%lg %lg %lg %d %d %d %d", &x, &y, &z, &r, &g, &g, &lum))
                    {
                        return ReadFileResult::failure(ReadFileResultCodes::OFF_ERROR_READING_VERTICES,
                                                       "Error reading vertices with color.");
                    }
                }

                meshBuilder->add_vertex(Primitives::Vertex3D(x, y, z));

                if (meshBuilder->num_vertices() != i + 1)
                {
                    return ReadFileResult::failure(
                        ReadFileResultCodes::OFF_READ_DUPLICATE_VERTICES,
                        fmt::format("Error reading vertices - duplicate vertices found in the file: ( {}, {}, {} )", x,
                                    y, z));
                }
            }
        }

        //  Load faces

        {
            TriangleMeshBuilderResultCodes resultCode;

            for (uint32_t i = 0; i < num_faces; ++i)
            {
                uint32_t poly_sides(0);

                VertexIndex::integer_type x_index(0U);
                VertexIndex::integer_type y_index(0U);
                VertexIndex::integer_type z_index(0U);

                //  We cannot use read_line_exactly as we want to detect non-triangular polygons

                next_line = input_file.next_line();

                if (!input_file.good())
                {
                    return ReadFileResult::failure(ReadFileResultCodes::OFF_ERROR_READING_FACES,
                                                   "Error reading faces.");
                }

                //  NOLINTNEXTLINE(cert-err34-c, cppcoreguidelines-pro-type-vararg, hicpp-vararg)
                items_processed = std::sscanf(next_line.c_str(), "%u %u %u %u %n", &poly_sides, &x_index, &y_index,
                                              &z_index, &chars_processed);

                if ((items_processed >= 1) && (poly_sides != 3))
                {
                    return ReadFileResult::failure(
                        ReadFileResultCodes::OFF_NON_TRIANGULAR_FACE,
                        fmt::format("Non Triangular face encountered on triangle index: {}", i));
                }

                if ((items_processed != 4) || (chars_processed != next_line.length()))
                {
                    return ReadFileResult::failure(ReadFileResultCodes::OFF_ERROR_READING_FACES,
                                                   "Error reading faces.");
                }

                if (meshBuilder->add_triangle(i, x_index, y_index, z_index) != TriangleMeshBuilderResultCodes::SUCCESS)
                {
                    return ReadFileResult::failure(
                        ReadFileResultCodes::OFF_ERROR_ADDING_FACE_TO_MESH,
                        fmt::format("Error adding triangle to mesh encountered on triangle index: {}", i));
                }
            }
        }

        return ReadFileResult(std::move(meshBuilder->mesh()));
    }

    //
    //  Write on OFF file
    //

    WriteFileResult writeOFF(const std::filesystem::path& file_path, const WriteableMesh& mesh_to_write)
    {
        //	Open the output file

        std::ofstream out(file_path);

        if (!out.good())
        {
            return WriteFileResult::failure(WriteFileResultCodes::UNABLE_TO_OPEN_FILE,
                                            fmt::format("Unable to open file: {}", file_path.c_str()));
        }

        //	Create two memory buffers so we can write the output data in two threads

        SEFUtility::PerformanceOStringStream vertices_stream(INITIAL_WRITE_BUFFER_SIZE);
        SEFUtility::PerformanceOStringStream triangles_stream(INITIAL_WRITE_BUFFER_SIZE);

        //	We will be writing in 'OFF' format, no color information.  Write the format label first.

        vertices_stream << "OFF\n";

        //	Write the number of vertices and triangles (i.e. faces)
        //		We will not be writing any edges, so write zero for that value.

        vertices_stream << mesh_to_write.vertices().size() << ' ' << mesh_to_write.triangles().size() << ' ' << 0 << "\n";

        //	Create a task group to allow us to spin off a thread

        tbb::task_group taskGroup;

        //	Write the vertices

        taskGroup.run([&] {
            for (const auto& currentVertex : mesh_to_write.vertices())
            {
                WriteVertex(vertices_stream, currentVertex) << "\n";
            }

            vertices_stream.flush();
        });

        //	Write the triangles - they are the faces

        for (const auto& currentTriangle : mesh_to_write.triangles())
        {
            triangles_stream << currentTriangle << "\n";
        }

        triangles_stream.flush();

        taskGroup.wait();

        out << vertices_stream.str();
        out << triangles_stream.str();

        if (!out)
        {
            return (WriteFileResult::failure(WriteFileResultCodes::ERROR_WRITING_TO_OFS_FILE,
                                             "Unknown Error writing to OFS file"));
        }

        out.flush();

        return WriteFileResult::success();
    }

}  // namespace Cork::Files