// +-------------------------------------------------------------------------
// | triangle_mesh_with_topo_cache.h
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

#include "mesh/TopoCache.h"
#include "triangle_mesh.h"

namespace Cork::Meshes
{
    class TriangleMeshWithTopoCache : public TriangleMesh
    {
       public:
        //	Methods follow

        virtual ~TriangleMeshWithTopoCache(){};

        virtual Meshes::TopoCacheBase<Primitives::TriangleByIndicesVector>& topo_cache() const = 0;
    };
}  // namespace Cork::Meshes