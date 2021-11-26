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

    class MeshBaseImpl : public virtual TriangleMeshBase  //  NOTE Virtual Inheritance !
    {
       public:
        using CorkVertex = Primitives::Vector3D;

        MeshBaseImpl() = delete;

        MeshBaseImpl(MeshBaseImpl&& mesh_base_to_move)
            : MeshBaseImpl(std::move( mesh_base_to_move ),
                           mesh_base_to_move.control_block_.value_or(CorkService::get_default_control_block()))
        {}

        MeshBaseImpl(const SolverControlBlock& control_block)
            : tris_(new TriangleByIndicesVector()),
              verts_(new Vertex3DVector()),
              max_vertex_magnitude_(NUMERIC_PRECISION_MIN),
              control_block_(control_block)
        {
        }

        MeshBaseImpl(MeshBaseImpl&& mesh_base_to_move, const SolverControlBlock& control_block)
            : bounding_box_(mesh_base_to_move.bounding_box_),
              min_and_max_edge_lengths_(mesh_base_to_move.min_and_max_edge_lengths_),
              max_vertex_magnitude_(mesh_base_to_move.max_vertex_magnitude_),
              tris_(std::move(mesh_base_to_move.tris_)),
              verts_(std::move(mesh_base_to_move.verts_)),
              control_block_(control_block)
        {
            mesh_base_to_move.clear();
        }

        MeshBaseImpl(size_t num_vertices, size_t num_triangles,
                     const SolverControlBlock& control_block = CorkService::get_default_control_block())
            : MeshBaseImpl(control_block)
        {
            verts_->reserve(num_vertices);
            tris_->reserve(num_triangles);
        }

        virtual ~MeshBaseImpl() {}

        void clear()
        {
            tris_.reset();
            verts_.reset();

            bounding_box_ = BBox3D();
            max_vertex_magnitude_ = NUMERIC_PRECISION_MIN;
            min_and_max_edge_lengths_ = MinAndMaxEdgeLengths();

            control_block_ = CorkService::get_default_control_block();
        }

        MeshBaseImpl clone() const
        {
            auto copy_of_tris{std::make_shared<TriangleByIndicesVector>(*tris_)};
            auto copy_of_verts{std::make_shared<Vertex3DVector>(*verts_)};

            return MeshBaseImpl(copy_of_tris, copy_of_verts, bounding_box_, min_and_max_edge_lengths_,
                                max_vertex_magnitude_);
        }

        MeshBaseImpl& operator=(const MeshBaseImpl&) = delete;

        const SolverControlBlock& solver_control_block() const { return (*control_block_); }

        size_t num_triangles() const { return tris_->size(); }
        size_t num_vertices() const { return verts_->size(); }

        TriangleByIndicesVector& triangles() { return (*tris_); }

        const TriangleByIndicesVector& triangles() const { return (*tris_); }

        Vertex3DVector& vertices() { return (*verts_); }

        const Vertex3DVector& vertices() const { return (*verts_); }

        const BBox3D& bounding_box() const { return bounding_box_; }

        MinAndMaxEdgeLengths min_and_max_edge_lengths() const { return min_and_max_edge_lengths_; }

        double max_vertex_magnitude() const { return max_vertex_magnitude_; }

        [[nodiscard]] TriangleByVertices triangle_by_vertices(const TriangleByIndices& triangle_by_indices) const final
        {
            return (TriangleByVertices((*verts_)[triangle_by_indices.a()], (*verts_)[triangle_by_indices.b()],
                                       (*verts_)[triangle_by_indices.c()]));
        }

        void add_triangle_and_update_metrics(const TriangleByIndices& new_triangle)
        {
            tris_->push_back(new_triangle);

            //  Compute a few metrics

            TriangleByVertices tri_by_verts{triangle_by_vertices(new_triangle)};

            bounding_box_.convex(tri_by_verts.bounding_box());

            max_vertex_magnitude_ = std::max(max_vertex_magnitude_, tri_by_verts.max_magnitude_vertex());

            min_and_max_edge_lengths_.update(tri_by_verts.min_and_max_edge_lengths());
        }

        const Math::Quantizer::GetQuantizerResult quantizer() const
        {
            return Math::Quantizer::get_quantizer(max_vertex_magnitude_, min_and_max_edge_lengths_.min());
        }

        void for_raw_tris(std::function<void(VertexIndex, VertexIndex, VertexIndex)> func)
        {
            for (auto& tri : *tris_)
            {
                func(tri.a(), tri.b(), tri.c());
            }
        }

        void for_raw_tris(std::function<void(VertexIndex, VertexIndex, VertexIndex)> func) const
        {
            for (auto& tri : *tris_)
            {
                func(tri.a(), tri.b(), tri.c());
            }
        }

       protected:
        std::shared_ptr<TriangleByIndicesVector> tris_;
        std::shared_ptr<Vertex3DVector> verts_;

        BBox3D bounding_box_;

        MinAndMaxEdgeLengths min_and_max_edge_lengths_;
        double max_vertex_magnitude_;

        std::optional<SolverControlBlock> control_block_;

        SolverPerfStats performance_stats_;

       private:
        MeshBaseImpl(std::shared_ptr<TriangleByIndicesVector>& triangles, std::shared_ptr<Vertex3DVector>& vertices,
                     const Primitives::BBox3D& boundingBox,
                     const Primitives::MinAndMaxEdgeLengths min_and_max_edge_lengths, double max_vertex_magnitude)
            : tris_(triangles),
              verts_(vertices),
              bounding_box_(boundingBox),
              min_and_max_edge_lengths_(min_and_max_edge_lengths),
              max_vertex_magnitude_(max_vertex_magnitude)
        {
        }
    };

}  // namespace Cork::Meshes