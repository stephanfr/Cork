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
#include "mesh/triangle_mesh.h"

namespace Cork
{
    class SolverControlBlock
    {
       public:
        explicit SolverControlBlock(bool useMultipleThreads, uint64_t minTrianglesForThreading,
                                    bool usePooledWorkspaces)
            : m_useMultipleThreads(useMultipleThreads),
              m_minTrianglesForThreading(minTrianglesForThreading),
              m_usePooledWorkspaces(usePooledWorkspaces),
              m_numTriangles(0)
        {
        }

        void setNumTriangles(uint64_t numTriangles) { m_numTriangles = numTriangles; }

        [[nodiscard]] bool useMultipleThreads() const
        {
            return (m_useMultipleThreads && (m_numTriangles > m_minTrianglesForThreading));
        }

        void setUseMultipleThreads(bool newValue) { m_useMultipleThreads = newValue; }

        [[nodiscard]] uint64_t minTrianglesForThreading() const { return (m_minTrianglesForThreading); }

        void setMinTrianglesForThreading(uint64_t minTriangles) { m_minTrianglesForThreading = minTriangles; }

        [[nodiscard]] bool usePooledWorkspaces() const { return (m_usePooledWorkspaces); }

       private:
        uint64_t m_numTriangles;

        bool m_useMultipleThreads;
        uint64_t m_minTrianglesForThreading;

        bool m_usePooledWorkspaces;
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

        [[nodiscard]] virtual uint64_t numberOfTrianglesInDisjointUnion() const = 0;
        [[nodiscard]] virtual uint64_t numberOfTrianglesInFinalMesh() const = 0;
        [[nodiscard]] virtual uint64_t elapsedCPUTimeInNanoSeconds() const = 0;
        [[nodiscard]] virtual uint64_t elapsedWallTimeInNanoSeconds() const = 0;
        [[nodiscard]] virtual uint64_t startingVirtualMemorySizeInMB() const = 0;
        [[nodiscard]] virtual uint64_t endingVirtualMemorySizeInMB() const = 0;
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

        CORKLIB_API
        static std::unique_ptr<CorkMesh> FromTriangleMesh(const Cork::TriangleMesh& triangleMesh);

        CORKLIB_API
        static const SolverControlBlock& GetDefaultControlBlock();

        CorkMesh() = default;

        CorkMesh(const CorkMesh&) = delete;
        CorkMesh(const CorkMesh&&) = delete;

        virtual ~CorkMesh() = default;

        CorkMesh& operator=(const CorkMesh&) = delete;
        CorkMesh& operator=(const CorkMesh&&) = delete;

        // NOLINTBEGIN(google-default-arguments)

        [[nodiscard]] virtual BooleanOperationResult Union(
            const CorkMesh& rhs, const SolverControlBlock& solverControlBlock = GetDefaultControlBlock()) const = 0;

        [[nodiscard]] virtual BooleanOperationResult Difference(
            const CorkMesh& rhs, const SolverControlBlock& solverControlBlock = GetDefaultControlBlock()) const = 0;

        [[nodiscard]] virtual BooleanOperationResult Intersection(
            const CorkMesh& rhs, const SolverControlBlock& solverControlBlock = GetDefaultControlBlock()) const = 0;

        [[nodiscard]] virtual BooleanOperationResult SymmetricDifference(
            const CorkMesh& rhs, const SolverControlBlock& solverControlBlock = GetDefaultControlBlock()) const = 0;

        // NOLINTEND(google-default-arguments)

        [[nodiscard]] virtual std::unique_ptr<TriangleMesh> ToTriangleMesh() const = 0;

        [[nodiscard]] virtual const SolverPerformanceStatisticsIfx& GetPerformanceStats() const = 0;

        [[nodiscard]] virtual size_t CountComponents() const = 0;
    };

}  // namespace Cork
