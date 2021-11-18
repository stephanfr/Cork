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

#include "cork.hpp"
#include "math/quantization.hpp"

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

        void set_number_of_triangles_in_disjoint_union(uint64_t number_of_triangles_in_disjoint_union)
        {
            number_of_triangles_in_disjoint_union_ = number_of_triangles_in_disjoint_union;
        }

        void set_number_of_triangles_in_final_mesh(uint64_t number_of_triangles_in_final_mesh)
        {
            number_of_triangles_in_final_mesh_ = number_of_triangles_in_final_mesh;
        }

        void set_elapsed_cpu_time_in_nano_seconds(uint64_t elapsed_cpu_time_in_nano_seconds)
        {
            elapsed_cpu_time_in_nanoseconds_ = elapsed_cpu_time_in_nano_seconds;
        }

        void set_elapsed_wall_time_in_nano_seconds(uint64_t elapsed_wall_time_in_nano_seconds)
        {
            elapsed_wall_time_in_nanoseconds_ = elapsed_wall_time_in_nano_seconds;
        }

        void set_starting_virtual_memory_size_in_MB(uint64_t starting_virtual_memory_size_in_MB)
        {
            starting_virtual_memory_size_in_MB_ = starting_virtual_memory_size_in_MB;
        }

        void set_ending_virtual_memory_size_in_MB(uint64_t ending_virtual_memory_size_in_MB)
        {
            ending_virtual_memory_size_in_MB_ = ending_virtual_memory_size_in_MB;
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

        CorkTriangle(const TriangleByIndices& triangle_to_copy, uint32_t bool_alg_data)
            : TriangleByIndices(triangle_to_copy), bool_alg_data_(bool_alg_data)
        {
        }

        const uint32_t bool_alg_data() const { return (bool_alg_data_); }

        uint32_t& boolAlgData() { return (bool_alg_data_); }

        void flip() { std::swap(a_, b_); }

        void offset_indices(uint32_t offset_value)
        {
            a_ += offset_value;
            b_ += offset_value;
            c_ += offset_value;
        }

       private:
        uint32_t bool_alg_data_;  // internal use by algorithm - value must be copied when the triangle is subdivided
    };

    class MeshBase
    {
       public:

        using CorkTriangleVector = std::vector<CorkTriangle>;
        using CorkVertex = Primitives::Vector3D;

        MeshBase() = delete;

        MeshBase(const SolverControlBlock& control_block) : control_block_(control_block) {}

        MeshBase(const MeshBase& mesh_base_to_copy, const SolverControlBlock& control_block)
            : bounding_box_(mesh_base_to_copy.bounding_box_),
              min_and_max_edge_lengths_(mesh_base_to_copy.min_and_max_edge_lengths_),
              max_vertex_magnitude_(mesh_base_to_copy.max_vertex_magnitude_),
              tris_(mesh_base_to_copy.tris_),
              verts_(mesh_base_to_copy.verts_),
              control_block_(control_block)
        {
        }

        virtual ~MeshBase() {}

        MeshBase& operator=(const MeshBase&) = delete;

        const SolverControlBlock& solver_control_block() const { return (*control_block_); }

        CorkTriangleVector& triangles() { return (tris_); }

        const CorkTriangleVector& triangles() const { return (tris_); }

        Vertex3DVector& vertices() { return (verts_); }

        const Vertex3DVector& vertices() const { return (verts_); }

        const BBox3D& bounding_box() const { return bounding_box_; }

        MinAndMaxEdgeLengths min_and_max_edge_lengths() const { return min_and_max_edge_lengths_; }

        double max_vertex_magnitude() const { return max_vertex_magnitude_; }

        const Math::Quantizer::GetQuantizerResult quantizer() const
        {
            return Math::Quantizer::get_quantizer(max_vertex_magnitude_, min_and_max_edge_lengths_.min());
        }

        void for_raw_tris(
            std::function<void(VertexIndex, VertexIndex, VertexIndex)> func)
        {
            for (auto& tri : tris_)
            {
                func(tri.a(), tri.b(), tri.c());
            }
        }

        void for_raw_tris(
            std::function<void(VertexIndex, VertexIndex, VertexIndex)> func) const
        {
            for (auto& tri : tris_)
            {
                func(tri.a(), tri.b(), tri.c());
            }
        }

       protected:
        CorkTriangleVector tris_;
        Vertex3DVector verts_;

        BBox3D bounding_box_;

        MinAndMaxEdgeLengths min_and_max_edge_lengths_;
        double max_vertex_magnitude_;

        std::optional<SolverControlBlock> control_block_;

        SolverPerfStats performance_stats_;
    };

}  // namespace Cork::Meshes