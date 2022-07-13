// +-------------------------------------------------------------------------
// | triangle_remapper.cpp
// |
// | Author: Stephan Friedl
// +-------------------------------------------------------------------------
// | COPYRIGHT:
// |    Copyright Stephan Friedl 2021
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

#include "triangle_remapper.hpp"

#include "mesh_base.hpp"

namespace Cork::Meshes
{
    TriangleRemapper::TriangleRemapper(const MeshBase& primary_mesh) : primary_mesh_(primary_mesh) {}

    inline void TriangleRemapper::remap_into_mesh(MeshBase& result_mesh, const TriangleByIndices& triangle)
    {
        if (!remapper_.contains(triangle.a()))
        {
            remapper_.emplace(triangle.a(), VertexIndex(uint32_t(result_mesh.vertices().size())));
            reverse_remapper_.emplace(VertexIndex(uint32_t(result_mesh.vertices().size())), triangle.a());

            result_mesh.vertices().emplace_back(primary_mesh_.vertices()[triangle.a()]);
        }

        if (!remapper_.contains(triangle.b()))
        {
            remapper_.emplace(triangle.b(), VertexIndex(uint32_t(result_mesh.vertices().size())));
            reverse_remapper_.emplace(VertexIndex(uint32_t(result_mesh.vertices().size())), triangle.b());

            result_mesh.vertices().emplace_back(primary_mesh_.vertices()[triangle.b()]);
        }

        if (!remapper_.contains(triangle.c()))
        {
            remapper_.emplace(triangle.c(), VertexIndex(uint32_t(result_mesh.vertices().size())));
            reverse_remapper_.emplace(VertexIndex(uint32_t(result_mesh.vertices().size())), triangle.c());

            result_mesh.vertices().emplace_back(primary_mesh_.vertices()[triangle.c()]);
        }

        VertexIndex new_a = remapper_.at(triangle.a());
        VertexIndex new_b = remapper_.at(triangle.b());
        VertexIndex new_c = remapper_.at(triangle.c());

        result_mesh.triangles().emplace_back(triangle.uid(), new_a, new_b, new_c);
    }

    std::unique_ptr<MeshBase> TriangleRemapper::extract_surface(const TriangleByIndicesVector& tris_to_extract)
    {
        auto result_mesh = std::make_unique<MeshBase>(tris_to_extract.size() * 3, tris_to_extract.size());

        for (const auto& tri : tris_to_extract)
        {
            remap_into_mesh(*result_mesh, tri);
        }

        return result_mesh;
    }

    std::unique_ptr<MeshBase> TriangleRemapper::extract_surface(const TriangleByIndicesIndexSet& tris_to_extract)
    {
        auto result_mesh = std::make_unique<MeshBase>(tris_to_extract.size() * 3, tris_to_extract.size());

        MinAndMaxEdgeLengths min_max_edges;
        double max_vertex_magnitude = DBL_MIN;

        for (const auto& tri : tris_to_extract)
        {
            remap_into_mesh(*result_mesh, primary_mesh_.triangles()[tri]);

            TriangleByVertices tri_by_verts = primary_mesh_.triangle_by_vertices(primary_mesh_.triangles()[tri]);

            min_max_edges.update(tri_by_verts.min_and_max_edge_lengths());
            max_vertex_magnitude = std::max(max_vertex_magnitude, tri_by_verts.max_magnitude_vertex());
        }

        result_mesh->min_and_max_edge_lengths_ = min_max_edges;
        result_mesh->max_vertex_magnitude_ = max_vertex_magnitude;

        return result_mesh;
    }

}  // namespace Cork::Meshes