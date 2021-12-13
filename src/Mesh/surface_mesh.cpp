// +-------------------------------------------------------------------------
// | surface_mesh.cpp
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

#include "mesh/surface_mesh.hpp"

namespace Cork::Meshes
{
    std::unique_ptr<SurfaceMesh> SurfaceMesh::extract_surface(const MeshBase& mesh,
                                                              TriangleByIndicesIndex center_triangle,
                                                              uint32_t num_rings)
    {
        Cork::Primitives::TriangleByIndicesIndexSet single_triangle;

        single_triangle.emplace(center_triangle);

        return Cork::Meshes::SurfaceMesh::extract_surface(
            mesh, mesh.find_enclosing_triangles(single_triangle, num_rings).merge(single_triangle));
    }

}  // namespace Cork::Meshes
