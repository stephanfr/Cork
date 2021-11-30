// +-------------------------------------------------------------------------
// | mesh_base.cpp
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

#include "mesh/mesh_base.hpp"

#include "primitives/remappers.hpp"

namespace Cork::Meshes
{
    class TriangleRemapper
    {
       public:
        TriangleRemapper(const MeshBase& primary_mesh) : primary_mesh_(primary_mesh) {}

        std::unique_ptr<MeshBase> extract_surface(const TriangleByIndicesVector& tris_to_extract)
        {
            result_mesh_ = std::make_unique<MeshBase>(tris_to_extract.size() * 3, tris_to_extract.size());

            for (const auto& tri : tris_to_extract)
            {
                remap(tri);
            }

            return std::unique_ptr<MeshBase>(result_mesh_.release());
        }

        std::unique_ptr<MeshBase> extract_surface(const TriangleByIndicesIndexSet& tris_to_extract)
        {
            result_mesh_ = std::make_unique<MeshBase>(tris_to_extract.size() * 3, tris_to_extract.size());

            for (const auto& tri : tris_to_extract)
            {
                remap(primary_mesh_.triangles()[tri]);
            }

            return std::unique_ptr<MeshBase>(result_mesh_.release());
        }

       private:
        const MeshBase& primary_mesh_;
        std::unique_ptr<MeshBase> result_mesh_;
        std::map<VertexIndex, VertexIndex> remapper_;

        void remap(const TriangleByIndices& triangle)
        {
            if (!remapper_.contains(triangle.a()))
            {
                remapper_.emplace(triangle.a(), VertexIndex(uint32_t(result_mesh_->vertices().size())));
                result_mesh_->vertices().emplace_back(primary_mesh_.vertices()[triangle.a()]);
            }

            if (!remapper_.contains(triangle.b()))
            {
                remapper_.emplace(triangle.b(), VertexIndex(uint32_t(result_mesh_->vertices().size())));
                result_mesh_->vertices().emplace_back(primary_mesh_.vertices()[triangle.b()]);
            }

            if (!remapper_.contains(triangle.c()))
            {
                remapper_.emplace(triangle.c(), VertexIndex(uint32_t(result_mesh_->vertices().size())));
                result_mesh_->vertices().emplace_back(primary_mesh_.vertices()[triangle.c()]);
            }

            VertexIndex new_a = remapper_.at(triangle.a());
            VertexIndex new_b = remapper_.at(triangle.b());
            VertexIndex new_c = remapper_.at(triangle.c());

            result_mesh_->triangles().emplace_back(result_mesh_->triangles().size(), new_a, new_b, new_c);
        }
    };

    MeshBase::MeshBase(MeshBase&& mesh_base_to_move)
        : bounding_box_(mesh_base_to_move.bounding_box_),
          min_and_max_edge_lengths_(mesh_base_to_move.min_and_max_edge_lengths_),
          max_vertex_magnitude_(mesh_base_to_move.max_vertex_magnitude_),
          tris_(std::move(mesh_base_to_move.tris_)),
          verts_(std::move(mesh_base_to_move.verts_))
    {
        mesh_base_to_move.clear();
    }

    MeshBase::MeshBase(size_t num_vertices, size_t num_triangles)
        : tris_(new TriangleByIndicesVector()),
          verts_(new Vertex3DVector()),
          max_vertex_magnitude_(NUMERIC_PRECISION_MIN)
    {
        tris_->reserve(num_triangles);
        verts_->reserve(num_vertices);
    }

    MeshBase::MeshBase(std::shared_ptr<TriangleByIndicesVector>& triangles, std::shared_ptr<Vertex3DVector>& vertices,
                       const Primitives::BBox3D& boundingBox,
                       const Primitives::MinAndMaxEdgeLengths min_and_max_edge_lengths, double max_vertex_magnitude)
        : tris_(triangles),
          verts_(vertices),
          bounding_box_(boundingBox),
          min_and_max_edge_lengths_(min_and_max_edge_lengths),
          max_vertex_magnitude_(max_vertex_magnitude)
    {
    }

    void MeshBase::clear()
    {
        tris_.reset();
        verts_.reset();

        bounding_box_ = BBox3D();
        max_vertex_magnitude_ = NUMERIC_PRECISION_MIN;
        min_and_max_edge_lengths_ = MinAndMaxEdgeLengths();
    }

    MeshBase MeshBase::clone() const
    {
        auto copy_of_tris{std::make_shared<TriangleByIndicesVector>(*tris_)};
        auto copy_of_verts{std::make_shared<Vertex3DVector>(*verts_)};

        return MeshBase(copy_of_tris, copy_of_verts, bounding_box_, min_and_max_edge_lengths_, max_vertex_magnitude_);
    }

    std::unique_ptr<MeshBase> MeshBase::extract_surface(const TriangleByIndicesVector& tris_to_extract)
    {
        return TriangleRemapper(*this).extract_surface(tris_to_extract);
    }

    std::unique_ptr<MeshBase> MeshBase::extract_surface(const TriangleByIndicesIndexSet& tris_to_extract)
    {
        return TriangleRemapper(*this).extract_surface(tris_to_extract);
    }

    void MeshBase::compact()
    {
        //  Removes unreferenced vertices and remaps triangles

        //  Start by identifying the vertices that are still in use

        BooleanVector<VertexIndex> live_verts(verts_->size(), false);

        for (const auto& tri : *tris_)
        {
            live_verts[tri.a()] = true;
            live_verts[tri.b()] = true;
            live_verts[tri.c()] = true;
        }

        //  Next, remap those verts - this will end up with vertices getting smaller indices as unused
        //      vertices are skipped.

        Primitives::IndexRemapper<VertexIndex> vertex_remapper;

        vertex_remapper.reserve(verts_->size());

        VertexIndex new_index = 0U;

        for (VertexIndex i = 0U; i < live_verts.size(); i++)
        {
            if (live_verts[i])
            {
                vertex_remapper[i] = new_index;
                (*verts_)[new_index] = (*verts_)[i];
                new_index++;
            }
        }

        verts_->resize(VertexIndex::integer_type(new_index));

        //  Remap the triangles

        TriangleByIndices::UIDType tri_uid = 0U;

        for (auto& tri : *tris_)
        {
            tri = TriangleByIndices(tri_uid++, vertex_remapper[tri.a()], vertex_remapper[tri.b()],
                                    vertex_remapper[tri.c()]);
        }

        //  Invalidate the topo cache

        topo_cache_.release();

        //  Done
    }

}  // namespace Cork::Meshes
