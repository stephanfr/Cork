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
#include "mesh/boundary_edge_builder.hpp"
#include "mesh/topo_cache.hpp"
#include "mesh/triangle_remapper.hpp"
#include "writeable_mesh.hpp"

namespace Cork::Meshes
{
    using FindEnclosingTrianglesResult =
        SEFUtility::ResultWithReturnUniquePtr<FindEnclosingTrianglesResultCodes, TriangleByIndicesIndexSet>;

    class MeshBase : public WriteableMesh
    {
       public:
        MeshBase(MeshBase&& mesh_base_to_move);

        MeshBase(size_t num_vertices, size_t num_triangles);

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

        void clear_topo_cache() { topo_cache_.reset(); }

        ExtractBoundariesResult get_boundary_edge(const TriangleByIndicesIndexSet& tris_to_outline) const;
        ExtractBoundariesResult get_boundary_edge(const TriangleByIndicesIndexVector& tris_to_outline) const;


        FindEnclosingTrianglesResult find_enclosing_triangles(const TriangleByIndicesIndexVector& triangles,
                                                              uint32_t num_layers) const;

        FindEnclosingTrianglesResult find_enclosing_triangles(const TriangleByIndicesIndexSet& interior_triangles,
                                                              uint32_t num_layers) const;

        FindEnclosingTrianglesResult find_enclosing_triangles(const BoundaryEdge& boundary,
                                                              const TriangleByIndicesIndexSet& interior_triangles,
                                                              uint32_t num_layers) const;

        TriangleByIndicesIndexSet find_triangles_including_vertex(VertexIndex vertex_index)
        {
            TriangleByIndicesIndexSet triangles_including_vertex;

            for (const auto& triangle_to_add : topo_cache().vertices().getPool()[vertex_index].triangles())
            {
                triangles_including_vertex.insert(triangle_to_add->ref());
            }

            return (triangles_including_vertex);
        }

        std::optional<TriangleByIndicesIndex> tri_containing_all_three_vertices(VertexIndex vert1, VertexIndex vert2,
                                                                                VertexIndex vert3) const;

        std::unique_ptr<MeshBase> extract_surface(TriangleRemapper& remapper,
                                                  TriangleByIndicesIndex center_triangle, uint32_t num_rings) const;

        std::unique_ptr<MeshBase> extract_surface(TriangleRemapper& remapper,
                                                  const TriangleByIndicesVector& tris_to_extract) const;
        std::unique_ptr<MeshBase> extract_surface(TriangleRemapper& remapper,
                                                  const TriangleByIndicesIndexSet& tris_to_extract) const;

        using GetHoleClosingTrianglesResult =
            SEFUtility::ResultWithReturnUniquePtr<HoleClosingResultCodes, TriangleByIndicesVector>;

        GetHoleClosingTrianglesResult get_hole_closing_triangles(const BoundaryEdge& hole);

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

        MeshBase&   operator=( MeshBase&& ) = default;

        MeshBase(std::shared_ptr<TriangleByIndicesVector>& triangles, std::shared_ptr<Vertex3DVector>& vertices,
                 const Primitives::BBox3D& boundingBox, const Primitives::MinAndMaxEdgeLengths min_and_max_edge_lengths,
                 double max_vertex_magnitude);

        friend class TriangleRemapper;
        friend class SurfaceMesh;
        friend class ExtractedSurfaceMesh;
    };

}  // namespace Cork::Meshes