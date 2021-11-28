#pragma once

#include <map>
#include <set>

#include "mesh/mesh_base.hpp"

namespace Cork::Meshes
{
    class SurfaceMesh : public MeshBase
    {
       public:
        SurfaceMesh(const Vertex3DVector&               reference_verts)
            : MeshBase(reference_verts.size(), reference_verts.size() / 2.75 ),
              reference_verts_(reference_verts)
        {
        }

        SurfaceMesh& add(const TriangleByIndices& triangle)
        {
            remap(triangle);
            return *this;
        }

        SurfaceMesh& add(const TriangleByIndicesVector& triangle_vec)
        {
            for (const auto& triangle : triangle_vec)
            {
                remap(triangle);
            }
            return *this;
        }

       private:
        const Vertex3DVector&               reference_verts_;
        std::map<VertexIndex, VertexIndex> remapper_;

        void remap(const TriangleByIndices& triangle)
        {
            if (!remapper_.contains(triangle.a()))
            {
                remapper_.emplace(triangle.a(), VertexIndex(uint32_t(verts_->size())));
                verts_->emplace_back(reference_verts_[triangle.a()]);
            }

            if (!remapper_.contains(triangle.b()))
            {
                remapper_.emplace(triangle.b(), VertexIndex(uint32_t(verts_->size())));
                verts_->emplace_back(reference_verts_[triangle.b()]);
            }

            if (!remapper_.contains(triangle.c()))
            {
                remapper_.emplace(triangle.c(), VertexIndex(uint32_t(verts_->size())));
                verts_->emplace_back(reference_verts_[triangle.c()]);
            }

            VertexIndex new_a = remapper_.at(triangle.a());
            VertexIndex new_b = remapper_.at(triangle.b());
            VertexIndex new_c = remapper_.at(triangle.c());

            tris_->emplace_back(tris_->size(), new_a, new_b, new_c);
        }
    };
}  // namespace Cork::Meshes
