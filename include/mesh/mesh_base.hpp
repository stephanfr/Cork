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

#include "writeable_mesh.hpp"

#include "math/quantization.hpp"
#include "mesh/topo_cache.hpp"

namespace Cork::Meshes
{
    class MeshBase : public WriteableMesh
    {
       public:
        MeshBase(MeshBase&& mesh_base_to_move);

        MeshBase(size_t num_vertices, size_t num_triangles );

        virtual ~MeshBase() {}

        void clear();

        MeshBase clone() const;

        MeshBase& operator=(const MeshBase&) = delete;

        size_t num_triangles() const { return tris_->size(); }
        size_t num_vertices() const { return verts_->size(); }

        TriangleByIndicesVector& triangles() { return (*tris_); }

        const TriangleByIndicesVector& triangles() const { return (*tris_); }

        Vertex3DVector& vertices() { return (*verts_); }

        const Vertex3DVector& vertices() const { return (*verts_); }

        const BBox3D& bounding_box() const { return bounding_box_; }

        MinAndMaxEdgeLengths min_and_max_edge_lengths() const { return min_and_max_edge_lengths_; }

        double max_vertex_magnitude() const { return max_vertex_magnitude_; }

        [[nodiscard]] TriangleByVertices triangle_by_vertices(const TriangleByIndices& triangle_by_indices) const
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

        [[nodiscard]] MeshTopoCache& topo_cache()
        {
            if (!topo_cache_)
            {
                auto get_quantizer_result =
                    Math::Quantizer::get_quantizer(max_vertex_magnitude(), min_and_max_edge_lengths().min());

                const_cast<std::unique_ptr<MeshTopoCache>&>(topo_cache_)
                    .reset(new MeshTopoCache(*this, get_quantizer_result.return_value()));
            }

            return *topo_cache_;
        }

        [[nodiscard]] const MeshTopoCache& topo_cache() const
        {
            if (!topo_cache_)
            {
                auto get_quantizer_result =
                    Math::Quantizer::get_quantizer(max_vertex_magnitude(), min_and_max_edge_lengths().min());

                const_cast<std::unique_ptr<MeshTopoCache>&>(topo_cache_)
                    .reset(new MeshTopoCache(const_cast<MeshBase&>(*this), get_quantizer_result.return_value()));
            }

            return *topo_cache_;
        }

        std::unique_ptr<MeshBase>       extract_surface( const TriangleByIndicesVector&       tris_to_extract ) const;
        std::unique_ptr<MeshBase>       extract_surface( const TriangleByIndicesIndexSet&     tris_to_extract ) const;

        void    compact();

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

        std::unique_ptr<MeshTopoCache> topo_cache_;

       private:
        MeshBase() = default;

        MeshBase(std::shared_ptr<TriangleByIndicesVector>& triangles, std::shared_ptr<Vertex3DVector>& vertices,
                     const Primitives::BBox3D& boundingBox,
                     const Primitives::MinAndMaxEdgeLengths min_and_max_edge_lengths, double max_vertex_magnitude);
    };

}  // namespace Cork::Meshes