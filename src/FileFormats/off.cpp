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

#include <fstream>
#include <iterator>
#include <sstream>

#ifdef __HAS_PERFORMANCE_STRINGSTREAM__
#include "util/performance_stringstream.h"
#endif

#include "file_formats/files.h"
#include "util/file_helpers.h"

namespace Cork::Files
{
    inline std::istream& operator>>(std::istream& inStream, Cork::Math::TriangleByIndicesBase& triToRead)
    {
        return (inStream >> triToRead[0] >> triToRead[1] >> triToRead[2]);
    }

    inline SEFUtility::PerformanceOStringStream& operator<<(SEFUtility::PerformanceOStringStream& out_stream,
                                                            const Cork::Math::TriangleByIndicesBase& triToWrite)
    {
        fmt::format_to(std::move(out_stream.back_insert_iterator()), FMT_COMPILE("3 {:d} {:d} {:d}"), triToWrite[0],
                       triToWrite[1], triToWrite[2]);

        return (out_stream);
    }

    inline SEFUtility::PerformanceOStringStream& WriteVertex(SEFUtility::PerformanceOStringStream& out_stream,
                                                             const Cork::Math::Vertex3D& vertex)
    {
        fmt::format_to(std::move(out_stream.back_insert_iterator()), FMT_COMPILE("{:g} {:g} {:g}"), vertex.x(),
                       vertex.y(), vertex.z());

        return out_stream;
    }

    inline SEFUtility::PerformanceOStringStream& operator<<(SEFUtility::PerformanceOStringStream& out_stream,
                                                            const Cork::Math::Vector3D& vec)
    {
        fmt::format_to(std::move(out_stream.back_insert_iterator()), FMT_COMPILE("{:g} {:g} {:g}"), vec.x(), vec.y(),
                       vec.z());

        return out_stream;
    }

    //
    //  Read an OFF or COFF encoded file
    //

    ReadFileResult readOFF(const std::filesystem::path& file_path)
    {
        //	Open the mesh file

        LineByLineFileReader input_file(file_path);

        if (!input_file.good())
        {
            return ReadFileResult::Failure(ReadFileResultCodes::UNABLE_TO_OPEN_FILE,
                                           fmt::format("Unable to open file: {}", file_path.string()));
        }

        //	Look for the label at the top of the file to indicate formatting of the rest of the file
        //		If the encoding is in the COFF format which includes color info, we will have to strip the color values

        char string_buffer[64];

        int items_processed;
        int chars_processed;

        std::string next_line;

        if (!input_file.read_line_exactly("%60s", 1, string_buffer))
        {
            return ReadFileResult::Failure(ReadFileResultCodes::ERROR_READING_FILE_TYPE,
                                           "Error reading file type in OFF file header");
        }

        const std::string file_type(string_buffer);

        if ((file_type != "OFF") && (file_type != "COFF"))
        {
            return ReadFileResult::Failure(ReadFileResultCodes::OFF_UNRECOGNIZED_HEADER,
                                           fmt::format("Unrecognized header for OFF file: {}", file_type));
        }

        bool strip_color = (file_type == "COFF");

        //	Load the number of vertices, faces and edges

        unsigned int num_vertices, num_faces, num_edges;

        if (!input_file.read_line_exactly("%d %d %d", 3, &num_vertices, &num_faces, &num_edges))
        {
            return ReadFileResult::Failure(ReadFileResultCodes::OFF_ERROR_READING_COUNTS,
                                           "Error reading counts of vertices, faces and edges.");
        }

        //	Get an incremental indexed vertices mesh builder

        std::unique_ptr<IncrementalVertexIndexTriangleMeshBuilder> meshBuilder(
            IncrementalVertexIndexTriangleMeshBuilder::GetBuilder(num_vertices, num_faces));

        //	Read the Vertex data

        {
            double x, y, z;

            for (unsigned int i = 0; i < num_vertices; ++i)
            {
                if (!input_file.read_line_exactly("%lg %lg %lg", 3, &x, &y, &z))
                {
                    return ReadFileResult::Failure(ReadFileResultCodes::OFF_ERROR_READING_VERTICES,
                                                   "Error reading vertices.");
                }

                meshBuilder->AddVertex(Cork::Math::Vertex3D(x, y, z));

                if (meshBuilder->num_vertices() != i + 1)
                {
                    return ReadFileResult::Failure(
                        ReadFileResultCodes::OFF_READ_DUPLICATE_VERTICES,
                        fmt::format("Error reading vertices - duplicate vertices found in the file: ( {}, {}, {} )", x,
                                    y, z));
                }

                if (strip_color)
                {
                    next_line = input_file.next_line();

                    if (!input_file.good())
                    {
                        return (ReadFileResult::Failure(ReadFileResultCodes::OFF_ERROR_STRIPPING_VERTEX_COLOR,
                                                        "Error stripping out vertex colors."));
                    }
                }
            }
        }

        //  Load faces

        {
            TriangleMeshBuilderResultCodes resultCode;

            for (unsigned int i = 0; i < num_faces; ++i)
            {
                uint32_t poly_sides, x_index, y_index, z_index;

                //  We cannot use read_line_exactly as we want to detect non-triangular polygons

                next_line = input_file.next_line();

                if (!input_file.good())
                {
                    return ReadFileResult::Failure(ReadFileResultCodes::OFF_ERROR_READING_FACES,
                                                   "Error reading faces.");
                }

                items_processed = sscanf(next_line.c_str(), "%u %u %u %u %n", &poly_sides, &x_index, &y_index, &z_index,
                                         &chars_processed);

                if ((items_processed >= 1) && (poly_sides != 3))
                {
                    return ReadFileResult::Failure(
                        ReadFileResultCodes::OFF_NON_TRIANGULAR_FACE,
                        fmt::format("Non Triangular face encountered on triangle index: {}", i));
                }

                if ((items_processed != 4) || (chars_processed != next_line.length()))
                {
                    return ReadFileResult::Failure(ReadFileResultCodes::OFF_ERROR_READING_FACES,
                                                   "Error reading faces.");
                }

                if ((resultCode = meshBuilder->AddTriangle(TriangleMesh::TriangleByIndices(
                         x_index, y_index, z_index))) != TriangleMeshBuilderResultCodes::SUCCESS)
                {
                    return ReadFileResult::Failure(
                        ReadFileResultCodes::OFF_ERROR_ADDING_FACE_TO_MESH,
                        fmt::format("Error adding triangle to mesh encountered on triangle index: {}", i));
                }
            }
        }

        std::unique_ptr<Cork::TriangleMesh> triMesh(std::move(meshBuilder->Mesh()));

        return ReadFileResult(triMesh);
    }

    //
    //  Write on OFF file
    //
    
    WriteFileResult writeOFF(const std::filesystem::path& file_path, const TriangleMesh& mesh_to_write)
    {
        //	Open the output file

        std::ofstream out(file_path);

        if (!out.good())
        {
            return WriteFileResult::Failure(WriteFileResultCodes::UNABLE_TO_OPEN_FILE,
                                            fmt::format("Unable to open file: {}", file_path.c_str()));
        }

        //	Create two memory buffers so we can write the output data in two threads

        SEFUtility::PerformanceOStringStream vertices_stream(1000000);
        SEFUtility::PerformanceOStringStream triangles_stream(1000000);

        //	We will be writing in 'OFF' format, no color information.  Write the format label first.

        vertices_stream << "OFF\n";

        //	Write the number of vertices and triangles (i.e. faces)
        //		We will not be writing any edges, so write zero for that value.

        vertices_stream << mesh_to_write.numVertices() << ' ' << mesh_to_write.numTriangles() << ' ' << 0 << "\n";

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
            return (WriteFileResult::Failure(WriteFileResultCodes::ERROR_WRITING_TO_OFS_FILE,
                                             "Unknown Error writing to OFS file"));
        }

        out.flush();

        return (WriteFileResult::Success());
    }

}  // namespace Cork::Files