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
        fmt::format_to(std::move(out_stream.back_insert_iterator()), FMT_COMPILE("{:f} {:f} {:f}"), vertex.x(),
                       vertex.y(), vertex.z());

        return out_stream;
    }

    inline SEFUtility::PerformanceOStringStream& operator<<(SEFUtility::PerformanceOStringStream& out_stream, const Cork::Math::Vector3D& vec)
    {
        fmt::format_to(std::move(out_stream.back_insert_iterator()), FMT_COMPILE("{:f} {:f} {:f}"), vec.x(), vec.y(),
                       vec.z());

        return out_stream;
    }

    ReadFileResult readOFF(const std::filesystem::path& file_path)
    {
        //	Open the mesh file

        std::ifstream in(file_path);

        if (!in.good())
        {
            return (ReadFileResult::Failure(ReadFileResultCodes::UNABLE_TO_OPEN_FILE,
                                            boost::format("Unable to open file: %s") % file_path.string()));
        }

        //	Look for the label at the top of the file to indicate formatting of the rest of the file
        //		If the encoding is in the COFF format which includes color info, we will have to strip the color values

        std::string fileType;

        in >> fileType;

        if ((fileType != "OFF") && (fileType != "COFF"))
        {
            return (ReadFileResult::Failure(ReadFileResultCodes::OFF_UNRECOGNIZED_HEADER,
                                            boost::format("Unrecognized header for OFF file: %s") % fileType));
        }

        bool stripColor = (fileType == "COFF");

        //	Load the number of vertices, faces and edges

        unsigned int numVertices, numFaces, numEdges;

        in >> numVertices >> numFaces >> numEdges;

        if (!in.good())
        {
            return (ReadFileResult::Failure(ReadFileResultCodes::OFF_ERROR_READING_COUNTS,
                                            "Error reading counts of vertices, faces and edges."));
        }

        //	Get an incremental indexed vertices mesh builder

        std::unique_ptr<IncrementalVertexIndexTriangleMeshBuilder> meshBuilder(
            IncrementalVertexIndexTriangleMeshBuilder::GetBuilder(numVertices, numFaces));

        //	Read the Vertex data

        {
            int red, green, blue, lum;

            NUMERIC_PRECISION x, y, z;

            for (unsigned int i = 0; i < numVertices; ++i)
            {
                in >> x >> y >> z;

                if (meshBuilder->AddVertex(Cork::Math::Vertex3D(x, y, z)) != i)
                {
                    return (ReadFileResult::Failure(
                        ReadFileResultCodes::OFF_READ_DUPLICATE_VERTICES,
                        boost::format("Error reading vertices - duplicate vertices found in the file: (%f,%f,%f)") % x %
                            y % z));
                }

                if (stripColor)
                {
                    in >> red >> green >> blue >> lum;
                }
            }

            if (!in.good())
            {
                return (ReadFileResult::Failure(ReadFileResultCodes::OFF_ERROR_READING_VERTICES,
                                                "Error reading vertices."));
            }
        }

        // face data

        {
            TriangleMesh::TriangleByIndices newTriangle(0, 0, 0);
            unsigned int polySize;
            TriangleMeshBuilderResultCodes resultCode;

            for (unsigned int i = 0; i < numFaces; ++i)
            {
                in >> polySize;

                if (polySize != 3)
                {
                    return (ReadFileResult::Failure(
                        ReadFileResultCodes::OFF_NON_TRIANGULAR_FACE,
                        boost::format("Non Triangular face encountered on triangle index: %i") % i));
                }

                in >> newTriangle;

                if ((resultCode = meshBuilder->AddTriangle(newTriangle)) != TriangleMeshBuilderResultCodes::SUCCESS)
                {
                    return (ReadFileResult::Failure(
                        ReadFileResultCodes::OFF_ERROR_ADDING_FACE_TO_MESH,
                        boost::format("Error adding triangle to mesh encountered on triangle index: %i") % i));
                }
            }

            if (!in.good())
            {
                return (ReadFileResult::Failure(ReadFileResultCodes::OFF_ERROR_READING_FACES, "Error reading faces."));
            }
        }

        std::unique_ptr<Cork::TriangleMesh> triMesh(std::move(meshBuilder->Mesh()));

        return (ReadFileResult(triMesh));
    }

    WriteFileResult writeOFF(const std::filesystem::path& file_path, const TriangleMesh& mesh_to_write)
    {
        //	Open the output file

        std::ofstream out(file_path);

        if (!out.good())
        {
            return (WriteFileResult::Failure(WriteFileResultCodes::UNABLE_TO_OPEN_FILE,
                                             (boost::format("Unable to open file: %s") % file_path.c_str()).str()));
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