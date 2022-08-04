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

#include "boundary_edge_builder.hpp"
#include "cork.hpp"
#include "math/quantization.hpp"
#include "topo_cache.hpp"
#include "triangle_remapper.hpp"
#include "writeable_interfaces.hpp"

namespace Cork::Meshes
{
    class HoleClosingSolution
    {
       public:
        HoleClosingSolution(TriangleByIndicesVector&& triangles_to_add, VertexIndexVector&& vertices_added)
            : triangles_to_add_(triangles_to_add), vertices_added_(vertices_added)
        {
        }

        TriangleByIndicesVector triangles_to_add_;
        VertexIndexVector vertices_added_;
    };

    using FindEnclosingTrianglesResult =
        SEFUtility::ResultWithReturnUniquePtr<FindEnclosingTrianglesResultCodes, TriangleByIndicesIndexSet>;

    using GetHoleClosingTrianglesResult =
        SEFUtility::ResultWithReturnValue<HoleClosingResultCodes, HoleClosingSolution>;

    class MeshBase : public WriteableMesh
    {
       public:
        MeshBase(const MeshBase&) = delete;
        MeshBase(MeshBase&& mesh_base_to_move) noexcept;

        MeshBase(size_t num_vertices, size_t num_triangles);

        virtual ~MeshBase() {}

        void clear();

        [[nodiscard]] MeshBase clone() const;

        MeshBase& operator=(const MeshBase&) = delete;

        [[nodiscard]] size_t num_triangles() const { return tris_->size(); }
        [[nodiscard]] size_t num_vertices() const { return verts_->size(); }

        [[nodiscard]] TriangleByIndicesVector& triangles() { return (*tris_); }

        [[nodiscard]] const TriangleByIndicesVector& triangles() const override { return (*tris_); }

        [[nodiscard]] Vertex3DVector& vertices() { return (*verts_); }

        [[nodiscard]] const Vertex3DVector& vertices() const override { return (*verts_); }

        [[nodiscard]] const BBox3D& bounding_box() const { return bounding_box_; }

        [[nodiscard]] MinAndMaxEdgeLengths min_and_max_edge_lengths() const { return min_and_max_edge_lengths_; }

        [[nodiscard]] double max_vertex_magnitude() const { return max_vertex_magnitude_; }

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

        [[nodiscard]] const Math::Quantizer::GetQuantizerResult quantizer() const
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

        void clear_topo_cache() { topo_cache_.reset(); }

        [[nodiscard]] ExtractBoundariesResult get_boundary_edge(const TriangleByIndicesIndexSet& tris_to_outline) const;
        [[nodiscard]] ExtractBoundariesResult get_boundary_edge(
            const TriangleByIndicesIndexVector& tris_to_outline) const;

        [[nodiscard]] FindEnclosingTrianglesResult find_enclosing_triangles(
            const TriangleByIndicesIndexVector& triangles, uint32_t num_layers) const;

        [[nodiscard]] FindEnclosingTrianglesResult find_enclosing_triangles(
            const TriangleByIndicesIndexSet& interior_triangles, uint32_t num_layers) const;

        [[nodiscard]] FindEnclosingTrianglesResult find_enclosing_triangles(
            const BoundaryEdge& boundary, const TriangleByIndicesIndexSet& interior_triangles,
            uint32_t num_layers) const;

        [[nodiscard]] TriangleByIndicesIndexSet find_triangles_including_vertex(VertexIndex vertex_index)
        {
            TriangleByIndicesIndexSet triangles_including_vertex;

            for (const auto& triangle_to_add : topo_cache().vertices().getPool()[vertex_index].triangles())
            {
                triangles_including_vertex.insert(triangle_to_add->ref());
            }

            return (triangles_including_vertex);
        }

        [[nodiscard]] std::optional<TriangleByIndicesIndex> tri_containing_all_three_vertices(VertexIndex vert1,
                                                                                              VertexIndex vert2,
                                                                                              VertexIndex vert3) const;

        [[nodiscard]] std::unique_ptr<MeshBase> extract_surface(TriangleRemapper& remapper,
                                                                TriangleByIndicesIndex center_triangle,
                                                                uint32_t num_rings) const;

        [[nodiscard]] std::unique_ptr<MeshBase> extract_surface(TriangleRemapper& remapper,
                                                                const TriangleByIndicesVector& tris_to_extract) const;
        [[nodiscard]] std::unique_ptr<MeshBase> extract_surface(TriangleRemapper& remapper,
                                                                const TriangleByIndicesIndexSet& tris_to_extract) const;

        [[nodiscard]] GetHoleClosingTrianglesResult close_hole(const BoundaryEdge& hole);

        void compact();

        void for_raw_tris(std::function<void(TriangleUID uid, VertexIndex, VertexIndex, VertexIndex)> func)
        {
            for (auto& tri : *tris_)
            {
                func(tri.uid(), tri.a(), tri.b(), tri.c());
            }
        }

        void for_raw_tris(std::function<void(TriangleUID uid, VertexIndex, VertexIndex, VertexIndex)> func) const
        {
            for (auto& tri : *tris_)
            {
                func(tri.uid(), tri.a(), tri.b(), tri.c());
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

        MeshBase& operator=(MeshBase&&) = default;

        MeshBase(std::shared_ptr<TriangleByIndicesVector>& triangles, std::shared_ptr<Vertex3DVector>& vertices,
                 const Primitives::BBox3D& boundingBox, const Primitives::MinAndMaxEdgeLengths min_and_max_edge_lengths,
                 double max_vertex_magnitude);

        friend class TriangleRemapper;
        friend class SurfaceMesh;
        friend class ExtractedSurfaceMesh;
    };

}  // namespace Cork::Meshes