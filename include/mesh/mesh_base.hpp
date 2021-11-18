// +-------------------------------------------------------------------------
// | mesh_base.hpp
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

#include <functional>
#include <optional>
#include <vector>

#include "math/quantization.hpp"
#include "primitives/primitives.hpp"
#include "triangle_mesh.h"
#include "cork.hpp"

namespace Cork::Meshes
{
    class SolverPerfStats : public SolverPerformanceStatisticsIfx
    {
       public:
        SolverPerfStats()
            : number_of_triangles_in_disjoint_union_(0),
              number_of_triangles_in_final_mesh_(0),
              elapsed_cpu_time_in_nanoseconds_(0),
              elapsed_wall_time_in_nanoseconds_(0),
              starting_virtual_memory_size_in_MB_(0),
              ending_virtual_memory_size_in_MB_(0)
        {
        }

        uint64_t number_of_triangles_in_disjoint_union() const { return (number_of_triangles_in_disjoint_union_); }

        uint64_t number_of_triangles_in_final_mesh() const { return (number_of_triangles_in_final_mesh_); }

        uint64_t elapsed_cpu_time_in_nanoseconds() const { return (elapsed_cpu_time_in_nanoseconds_); }

        uint64_t elapsed_wall_time_in_nanoseconds() const { return (elapsed_wall_time_in_nanoseconds_); }

        uint64_t starting_virtual_memory_size_in_MB() const { return (starting_virtual_memory_size_in_MB_); }

        uint64_t ending_virtual_memory_size_in_MB() const { return (ending_virtual_memory_size_in_MB_); }

        void setNumberOfTrianglesInDisjointUnion(uint64_t numberOfTrianglesInDisjointUnion)
        {
            number_of_triangles_in_disjoint_union_ = numberOfTrianglesInDisjointUnion;
        }

        void setNumberOfTrianglesInFinalMesh(uint64_t numberOfTrianglesInFinalMesh)
        {
            number_of_triangles_in_final_mesh_ = numberOfTrianglesInFinalMesh;
        }

        void setElapsedCPUTimeInNanoSeconds(uint64_t elapsedCPUTimeInNanoSeconds)
        {
            elapsed_cpu_time_in_nanoseconds_ = elapsedCPUTimeInNanoSeconds;
        }

        void setElapsedWallTimeInNanoSeconds(uint64_t elapsedWallTimeInNanoSeconds)
        {
            elapsed_wall_time_in_nanoseconds_ = elapsedWallTimeInNanoSeconds;
        }

        void setStartingVirtualMemorySizeInMB(uint64_t startingVirtualMemorySizeInMB)
        {
            starting_virtual_memory_size_in_MB_ = startingVirtualMemorySizeInMB;
        }

        void setEndingVirtualMemorySizeInMB(uint64_t endingVirtualMemorySizeInMB)
        {
            ending_virtual_memory_size_in_MB_ = endingVirtualMemorySizeInMB;
        }

       private:
        uint64_t number_of_triangles_in_disjoint_union_;
        uint64_t number_of_triangles_in_final_mesh_;

        uint64_t elapsed_cpu_time_in_nanoseconds_;
        uint64_t elapsed_wall_time_in_nanoseconds_;

        uint64_t starting_virtual_memory_size_in_MB_;
        uint64_t ending_virtual_memory_size_in_MB_;
    };

    class CorkTriangle : public TriangleByIndices
    {
       public:
        CorkTriangle() {}

        CorkTriangle(const TriangleByIndices& triangleToCopy, uint32_t boolAlgData)
            : TriangleByIndices(triangleToCopy), m_boolAlgData(boolAlgData)
        {
        }

        const uint32_t boolAlgData() const { return (m_boolAlgData); }

        uint32_t& boolAlgData() { return (m_boolAlgData); }

        void flip() { std::swap(a_, b_); }

        void offsetIndices(uint32_t offsetValue)
        {
            a_ += offsetValue;
            b_ += offsetValue;
            c_ += offsetValue;
        }

       private:
        uint32_t m_boolAlgData;  // internal use by algorithm - value must be copied when the triangle is subdivided
    };

    class MeshBase
    {
       public:

        using TriangleVector = std::vector<CorkTriangle>;
        using CorkVertex = Primitives::Vector3D;

        MeshBase() = delete;

        MeshBase(const SolverControlBlock& controlBlock) : m_controlBlock(controlBlock) {}

        MeshBase(const MeshBase& meshBaseToCopy, const SolverControlBlock& controlBlock)
            : m_boundingBox(meshBaseToCopy.m_boundingBox),
              min_and_max_edge_lengths_(meshBaseToCopy.min_and_max_edge_lengths_),
              max_vertex_magnitude_(meshBaseToCopy.max_vertex_magnitude_),
              m_tris(meshBaseToCopy.m_tris),
              m_verts(meshBaseToCopy.m_verts),
              m_controlBlock(controlBlock)
        {
        }

        virtual ~MeshBase() {}

        MeshBase& operator=(const MeshBase&) = delete;

        const SolverControlBlock& solverControlBlock() const { return (*m_controlBlock); }

        TriangleVector& triangles() { return (m_tris); }

        const TriangleVector& triangles() const { return (m_tris); }

        Vertex3DVector& vertices() { return (m_verts); }

        const Vertex3DVector& vertices() const { return (m_verts); }

        const BBox3D& boundingBox() const { return m_boundingBox; }
        MinAndMaxEdgeLengths min_and_max_edge_lengths() const { return min_and_max_edge_lengths_; }
        double max_vertex_magnitude() const { return max_vertex_magnitude_; }

        const Math::Quantizer::GetQuantizerResult getQuantizer() const
        {
            return Math::Quantizer::get_quantizer(max_vertex_magnitude_, min_and_max_edge_lengths_.min());
        }

        void for_raw_tris(
            std::function<void(VertexIndex, VertexIndex, VertexIndex)> func)
        {
            for (auto& tri : m_tris)
            {
                func(tri.a(), tri.b(), tri.c());
            }
        }

        void for_raw_tris(
            std::function<void(VertexIndex, VertexIndex, VertexIndex)> func) const
        {
            for (auto& tri : m_tris)
            {
                func(tri.a(), tri.b(), tri.c());
            }
        }

       protected:
        TriangleVector m_tris;
        Vertex3DVector m_verts;

        BBox3D m_boundingBox;

        MinAndMaxEdgeLengths min_and_max_edge_lengths_;
        double max_vertex_magnitude_;

        std::optional<SolverControlBlock> m_controlBlock;

        SolverPerfStats m_performanceStats;
    };

}  // namespace Cork::Meshes