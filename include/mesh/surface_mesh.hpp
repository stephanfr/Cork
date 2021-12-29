// +-------------------------------------------------------------------------
// | surface_mesh.hpp
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

#include "tbb/concurrent_vector.h"

#include "mesh/edge_graph_cache.hpp"
#include "mesh_base.hpp"

#include "primitives/self_intersecting_edge.hpp"

namespace Cork::Meshes
{
    //
    //	The Mesh class brings together the functionality needed for the boolean operations
    //

    class SurfaceMesh : public MeshBase
    {
       public:
        SurfaceMesh() = delete;

        SurfaceMesh(SurfaceMesh&& src) : MeshBase(std::move(src)){};

        SurfaceMesh(MeshBase&& src) : MeshBase(std::move(src)){};

        explicit SurfaceMesh(const TriangleMesh& inputMesh);

        virtual ~SurfaceMesh();

        void operator=(SurfaceMesh&& src);

        void        scrub_surface();

        BoundaryEdge        find_outside_boundary();

        void        remove_self_intersection( const SelfIntersectingEdge&   self_intersection );
    };
}  // namespace Cork::Meshes