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

namespace Cork::Meshes
{

    MeshBase::MeshBase(MeshBase&& mesh_base_to_move )
        : bounding_box_(mesh_base_to_move.bounding_box_),
          min_and_max_edge_lengths_(mesh_base_to_move.min_and_max_edge_lengths_),
          max_vertex_magnitude_(mesh_base_to_move.max_vertex_magnitude_),
          tris_(std::move(mesh_base_to_move.tris_)),
          verts_(std::move(mesh_base_to_move.verts_))
    {
        mesh_base_to_move.clear();
    }

    MeshBase::MeshBase(size_t num_vertices, size_t num_triangles )
        : tris_(new TriangleByIndicesVector()),
          verts_(new Vertex3DVector()),
          max_vertex_magnitude_(NUMERIC_PRECISION_MIN)
    {
        tris_->reserve( num_triangles );
        verts_->reserve( num_vertices );
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

        return MeshBase(copy_of_tris, copy_of_verts, bounding_box_, min_and_max_edge_lengths_,
                            max_vertex_magnitude_);
    }

    MeshBase::MeshBase(std::shared_ptr<TriangleByIndicesVector>& triangles,
                               std::shared_ptr<Vertex3DVector>& vertices, const Primitives::BBox3D& boundingBox,
                               const Primitives::MinAndMaxEdgeLengths min_and_max_edge_lengths,
                               double max_vertex_magnitude)
        : tris_(triangles),
          verts_(vertices),
          bounding_box_(boundingBox),
          min_and_max_edge_lengths_(min_and_max_edge_lengths),
          max_vertex_magnitude_(max_vertex_magnitude)
    {
    }

}  // namespace Cork::Meshes
