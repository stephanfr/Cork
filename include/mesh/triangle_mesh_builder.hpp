// +-------------------------------------------------------------------------
// | TriangleMesh.h
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

#include "cork.hpp"

namespace Cork::Meshes
{
    enum class TriangleMeshBuilderResultCodes
    {
        SUCCESS = 0,

        VERTEX_INDEX_OUT_OF_BOUNDS,

        MESH_NOT_2_MANIFOLD
    };

    using TriangleMeshBuilderResult =
        SEFUtility::ResultWithReturnUniquePtr<TriangleMeshBuilderResultCodes, TriangleMesh>;

    class IncrementalVertexIndexTriangleMeshBuilder
    {
       public:
        static std::unique_ptr<IncrementalVertexIndexTriangleMeshBuilder> GetBuilder(size_t numVertices = 0,
                                                                                     size_t numTriangles = 0);

        virtual ~IncrementalVertexIndexTriangleMeshBuilder(){};

        virtual size_t num_vertices() const = 0;

        virtual VertexIndex add_vertex(const Vertex3D& vertexToAdd) = 0;
        virtual TriangleMeshBuilderResultCodes add_triangle(VertexIndex a, VertexIndex b, VertexIndex c) = 0;

        virtual std::unique_ptr<TriangleMesh> mesh() = 0;
    };

}  // namespace Cork::Meshes
