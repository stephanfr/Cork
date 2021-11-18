// +-------------------------------------------------------------------------
// | cork.h
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

#include "CPPResult.hpp"
#include "statistics.hpp"

namespace Cork
{
//    using TriangleMesh = Meshes::TriangleMesh;

    enum class TopologicalStatisticsResultCodes
    {
        SUCCESS = 0,

        ANALYSIS_FAILED
    };

    using TopologicalStatisticsResult =
        SEFUtility::ResultWithReturnValue<TopologicalStatisticsResultCodes, Statistics::TopologicalStatistics>;

    enum class HoleClosingResultCodes
    {
        SUCCESS = 0,

        TRIANGULATION_FAILED
    };

    using HoleClosingResult = SEFUtility::Result<HoleClosingResultCodes>;

    class TriangleMesh
    {
       public:
        //	Methods follow

        virtual ~TriangleMesh(){};

        virtual size_t numTriangles() const = 0;
        virtual size_t numVertices() const = 0;

        virtual const Primitives::Vertex3DVector& vertices() const = 0;
        virtual const Primitives::TriangleByIndicesVector& triangles() const = 0;

        virtual Primitives::TriangleByVertices triangleByVertices(
            const Primitives::TriangleByIndices& triangleByIndices) const = 0;

        virtual void AddTriangle(const Primitives::TriangleByIndices& triangle_to_add) = 0;
        virtual void remove_triangle(Primitives::TriangleByIndicesIndex triangle_index) = 0;

        virtual const Primitives::BBox3D& boundingBox() const = 0;
        virtual Primitives::MinAndMaxEdgeLengths min_and_max_edge_lengths() const = 0;
        virtual double max_vertex_magnitude() const = 0;

        virtual Statistics::GeometricStatistics ComputeGeometricStatistics(
            Statistics::GeometricProperties props_to_compute) const = 0;
        virtual TopologicalStatisticsResult ComputeTopologicalStatistics(
            Statistics::TopologicalProperties props_to_compute) const = 0;

        virtual HoleClosingResult close_holes(const Statistics::TopologicalStatistics& topo_stats) = 0;
        virtual void remove_self_intersections(const Statistics::TopologicalStatistics& topo_stats) = 0;
    };


    class SolverControlBlock
    {
       public:
        SolverControlBlock() = delete;

        explicit SolverControlBlock(bool use_multiple_threads, uint64_t min_triangles_for_threading,
                                    bool use_pooled_workspaces)
            : use_multiple_threads_(use_multiple_threads),
              min_triangles_for_threading_(min_triangles_for_threading),
              use_pooled_workspaces_(use_pooled_workspaces),
              num_triangles_(0)
        {
        }

        SolverControlBlock(const SolverControlBlock& block_to_copy) = default;

        SolverControlBlock& operator=(const SolverControlBlock& block_to_copy) = default;

        void set_num_triangles(uint64_t numTriangles) { num_triangles_ = numTriangles; }

        [[nodiscard]] bool use_multiple_threads() const
        {
            return (use_multiple_threads_ && (num_triangles_ > min_triangles_for_threading_));
        }

        void set_use_multiple_threads(bool newValue) { use_multiple_threads_ = newValue; }

        [[nodiscard]] uint64_t min_triangles_for_threading() const { return (min_triangles_for_threading_); }

        void set_min_triangles_for_threading(uint64_t minTriangles) { min_triangles_for_threading_ = minTriangles; }

        [[nodiscard]] bool use_pooled_workspaces() const { return (use_pooled_workspaces_); }

       private:
        uint64_t num_triangles_;

        bool use_multiple_threads_;
        uint64_t min_triangles_for_threading_;

        bool use_pooled_workspaces_;
    };

    class SolverPerformanceStatisticsIfx
    {
       public:
        SolverPerformanceStatisticsIfx() = default;

        SolverPerformanceStatisticsIfx(const SolverPerformanceStatisticsIfx&) = delete;
        SolverPerformanceStatisticsIfx(const SolverPerformanceStatisticsIfx&&) = delete;

        virtual ~SolverPerformanceStatisticsIfx() = default;

        SolverPerformanceStatisticsIfx& operator=(const SolverPerformanceStatisticsIfx&) = delete;
        SolverPerformanceStatisticsIfx& operator=(const SolverPerformanceStatisticsIfx&&) = delete;

        [[nodiscard]] virtual uint64_t number_of_triangles_in_disjoint_union() const = 0;
        [[nodiscard]] virtual uint64_t number_of_triangles_in_final_mesh() const = 0;
        [[nodiscard]] virtual uint64_t elapsed_cpu_time_in_nanoseconds() const = 0;
        [[nodiscard]] virtual uint64_t elapsed_wall_time_in_nanoseconds() const = 0;
        [[nodiscard]] virtual uint64_t starting_virtual_memory_size_in_MB() const = 0;
        [[nodiscard]] virtual uint64_t ending_virtual_memory_size_in_MB() const = 0;
    };

    class CorkMesh;

    class CorkService
    {
       public:
        CorkService() = default;
        CorkService(const CorkService&) = delete;
        CorkService(CorkService&&) = delete;

        ~CorkService();

        CorkService& operator=(const CorkService&) = delete;

        static std::unique_ptr<CorkMesh> from_triangle_mesh(const TriangleMesh& triangleMesh);

        static const SolverControlBlock& get_default_control_block();
    };

    enum class BooleanOperationResultCodes
    {
        SUCCESS = 0,

        ERROR_DURING_BOOLEAN_PROBLEM_SETUP
    };

    class CorkMesh
    {
       public:
        typedef SEFUtility::ResultWithReturnUniquePtr<BooleanOperationResultCodes, CorkMesh> BooleanOperationResult;

        CorkMesh() = default;

        CorkMesh(const CorkMesh&) = delete;
        CorkMesh(const CorkMesh&&) = delete;

        virtual ~CorkMesh() = default;

        CorkMesh& operator=(const CorkMesh&) = delete;
        CorkMesh& operator=(const CorkMesh&&) = delete;

        // NOLINTBEGIN(google-default-arguments)

        [[nodiscard]] virtual BooleanOperationResult Union(
            const CorkMesh& rhs, const SolverControlBlock& solverControlBlock = CorkService::get_default_control_block()) const = 0;

        [[nodiscard]] virtual BooleanOperationResult Difference(
            const CorkMesh& rhs, const SolverControlBlock& solverControlBlock = CorkService::get_default_control_block()) const = 0;

        [[nodiscard]] virtual BooleanOperationResult Intersection(
            const CorkMesh& rhs, const SolverControlBlock& solverControlBlock = CorkService::get_default_control_block()) const = 0;

        [[nodiscard]] virtual BooleanOperationResult SymmetricDifference(
            const CorkMesh& rhs, const SolverControlBlock& solverControlBlock = CorkService::get_default_control_block()) const = 0;

        // NOLINTEND(google-default-arguments)

        [[nodiscard]] virtual std::unique_ptr<TriangleMesh> ToTriangleMesh() const = 0;

        [[nodiscard]] virtual const SolverPerformanceStatisticsIfx& GetPerformanceStats() const = 0;

        [[nodiscard]] virtual size_t CountComponents() const = 0;
    };

}  // namespace Cork