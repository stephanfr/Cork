// +-------------------------------------------------------------------------
// | triangle_remapper.hpp
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
#pragma once

#include "map"
#include "primitives/primitives.hpp"

namespace Cork::Meshes
{
    //  Forward declare the MeshBase class

    class MeshBase;

    class TriangleRemapper
    {
       public:
        TriangleRemapper(const MeshBase& primary_mesh);

        std::unique_ptr<MeshBase> extract_surface(const TriangleByIndicesVector& tris_to_extract);

        std::unique_ptr<MeshBase> extract_surface(const TriangleByIndicesIndexSet& tris_to_extract);

        void remap_into_mesh(MeshBase& result_mesh, const TriangleByIndices& triangle);

        VertexIndex     reverse_mapping( VertexIndex    remapped_index )
        {
            return reverse_remapper_.find( remapped_index )->second;
        }

       private:
        const MeshBase& primary_mesh_;
        std::map<VertexIndex, VertexIndex> remapper_;
        std::map<VertexIndex, VertexIndex> reverse_remapper_;
    };
}  // namespace Cork::Meshes