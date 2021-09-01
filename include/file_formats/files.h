// +-------------------------------------------------------------------------
// | files.h
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
#pragma once

// The following ifdef block is the standard way of creating macros which make exporting
// from a DLL simpler. All files within this DLL are compiled with the CORKLIB_EXPORTS
// symbol defined on the command line. This symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see
// CORKLIB_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.
#ifdef WINDOWS
#ifdef CORKLIB_EXPORTS
#define CORKLIB_API __declspec(dllexport)
#else
#define CORKLIB_API __declspec(dllimport)
#endif
#else
#define CORKLIB_API
#endif

#include <filesystem>
#include <string>

#include "mesh/triangle_mesh.h"

/*
 *  Files provides a wrapper for different file types and a common
 *  data view for the rest of the program.  This wrapper was introduced
 *  to make it easier to support multiple file types using other people's
 *  file importer/exporter code
 */

namespace Cork::Files
{
    enum class ReadFileResultCodes
    {
        SUCCESS = 0,
        UNABLE_TO_OPEN_FILE,
        UNABLE_TO_FIND_FILE_EXTENSION,
        ERROR_READING_FILE_TYPE,
        UNSUPPORTED_FILE_TYPE,

        IFS_UNRECOGNIZED_HEADER,
        IFS_FILE_FORMAT_ERROR,
        IFS_NO_VERSION_FOUND,
        IFS_UNSUPPORTED_VERSION,
        IFS_ERROR_READING_MODEL_NAME,
        IFS_ERROR_READING_VERTEX,
        IFS_ERROR_READING_TRIANGLE,

        OFF_UNRECOGNIZED_HEADER,
        OFF_ERROR_READING_COUNTS,
        OFF_ERROR_READING_VERTICES,
        OFF_READ_DUPLICATE_VERTICES,
        OFF_ERROR_STRIPPING_VERTEX_COLOR,
        OFF_NON_TRIANGULAR_FACE,
        OFF_ERROR_ADDING_FACE_TO_MESH,
        OFF_ERROR_READING_FACES
    };

    enum class WriteFileResultCodes
    {
        SUCCESS = 0,
        UNABLE_TO_FIND_FILE_EXTENSION,
        UNSUPPORTED_FILE_TYPE,
        UNABLE_TO_OPEN_FILE,

        ERROR_WRITING_TO_OFS_FILE,
        ERROR_WRITING_TO_IFS_FILE
    };

    typedef SEFUtility::ResultWithUniqueReturnPtr<ReadFileResultCodes, TriangleMesh> ReadFileResult;
    typedef SEFUtility::Result<WriteFileResultCodes> WriteFileResult;

    CORKLIB_API ReadFileResult readOFF(const std::filesystem::path& file_path);
    CORKLIB_API WriteFileResult writeOFF(const std::filesystem::path& file_path, const TriangleMesh& mesh_to_write);

}  // namespace Cork::Files