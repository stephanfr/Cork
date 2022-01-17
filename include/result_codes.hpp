// +-------------------------------------------------------------------------
// | cork.h
// |
// | Author: Stephan Friedl
// +-------------------------------------------------------------------------
// | COPYRIGHT:
// |    Copyright Stephan Friedl 2021
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

namespace Cork
{
    enum class BooleanOperationResultCodes
    {
        SUCCESS = 0,

        ERROR_DURING_BOOLEAN_PROBLEM_SETUP
    };

    enum class HoleClosingResultCodes
    {
        SUCCESS = 0,

        HOLE_BOUNDARY_SELF_INTERSECTS,
        TRIANGULATION_FAILED
    };

    enum class TopologicalStatisticsResultCodes
    {
        SUCCESS = 0,

        ANALYSIS_FAILED
    };

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

    enum class AdjustPerturbationResultCodes
    {
        SUCCESS = 0,

        MAXIMUM_PERTURBATION_REACHED
    };

    enum class ConsolidateResultCodes
    {
        SUCCESS = 0,

        COULD_NOT_FIND_COMMON_VERTEX
    };

    enum SubdivideResultCodes
    {
        SUCCESS = 0,

        SELF_INTERSECTING_MESH,
        FAILED_TRIANGULATION
    };

    enum class TriangulationResultCodes
    {
        SUCCESS = 0,

        TOO_MANY_POINTS,
        TOO_MANY_SEGMENTS,
        UNEQUAL_NUMBER_OF_INPUT_AND_OUTPUT_POINTS
    };

    enum class QuantizerResultCodes
    {
        SUCCESS = 0,

        EXCESSIVE_DYNAMIC_RANGE_FOR_QUANTIZATION,
        INSUFFICIENT_PERTURBATION_RANGE
    };

    enum class ExtractBoundariesResultCodes
    {
        SUCCESS = 0,

        EMPTY_REGION,
        BOUNDARY_IS_NOT_CLOSED,
        COULD_NOT_FIND_CLOSED_BOUNDARY
    };

    enum class SetupBooleanProblemResultCodes
    {
        SUCCESS = 0,

        TOO_MANY_TRIANGLES_IN_DISJOINT_UNION,
        QUANTIZER_CREATION_FAILED,
        FIND_INTERSECTIONS_FAILED,
        RESOLVE_INTERSECTIONS_FAILED,
        POPULATE_EDGE_GRAPH_CACHE_FAILED,
        SELF_INTERSECTING_MESH
    };

    enum class BuildEGraphCacheResultCodes
    {
        SUCCESS = 0,

        OUT_OF_MEMORY
    };

    enum class TryToFindIntersectionsResultCodes
    {
        SUCCESS = 0,

        OUT_OF_MEMORY,
        TRI_EGDE_DEGENERACIES,
        TRI_TRI_TRI_INTERSECTIONS_FAILED
    };

    enum class FindEnclosingTrianglesResultCodes
    {
        SUCCESS = 0,

        UNABLE_TO_RECOMPUTE_BOUNDARY
    };

    
    enum class FindTopoEdgeBoundariesResultCodes
    {
        SUCCESS = 0,

        EDGE_WITH_MORE_THAN_TWO_INCIDENCES_FOUND
    };

}  // namespace Cork
