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

#include <cassert>
#include <deque>
#include <vector>

#include "CPPResult.hpp"
#include "math/Primitives.h"
#include "statistics/statistics.h"

namespace Cork::Meshes
{
    using Vertex = Math::Vertex3D;
    using VertexVector = Math::Vertex3DVector;

    using EdgeByIndices = Math::EdgeByIndices;
    using EdgeByIndicesVector = Math::EdgeByIndicesVector;

    using TriangleByIndices = Math::TriangleByIndices;
    using TriangleByVertices = Math::TriangleByVertices;

    using TriangleByIndicesVector = Math::TriangleByIndicesVector;
    using TriangleByIndicesIndex = Math::TriangleByIndicesIndex;


    class TriangleMesh
    {
       public:
        //	Methods follow

        virtual ~TriangleMesh(){};

        virtual size_t numTriangles() const = 0;
        virtual size_t numVertices() const = 0;

        virtual const VertexVector& vertices() const = 0;
        virtual const TriangleByIndicesVector& triangles() const = 0;

        virtual TriangleByVertices triangleByVertices(const TriangleByIndices& triangleByIndices) const = 0;

        virtual void remove_triangle(TriangleByIndicesIndex triangle_index) = 0;

        virtual const Math::BBox3D& boundingBox() const = 0;
        virtual Math::MinAndMaxEdgeLengths min_and_max_edge_lengths() const = 0;
        virtual double max_vertex_magnitude() const = 0;

        virtual Statistics::GeometricStatistics ComputeGeometricStatistics() const = 0;
        virtual Statistics::TopologicalStatistics ComputeTopologicalStatistics() const = 0;
    };

    enum class TriangleMeshBuilderResultCodes
    {
        SUCCESS = 0,

        VERTEX_INDEX_OUT_OF_BOUNDS,

        MESH_NOT_2_MANIFOLD
    };

    typedef SEFUtility::ResultWithReturnUniquePtr<TriangleMeshBuilderResultCodes, TriangleMesh>
        TriangleMeshBuilderResult;

    class IncrementalVertexIndexTriangleMeshBuilder
    {
       public:
        static std::unique_ptr<IncrementalVertexIndexTriangleMeshBuilder> GetBuilder(size_t numVertices = 0,
                                                                                     size_t numTriangles = 0);

        virtual ~IncrementalVertexIndexTriangleMeshBuilder(){};

        virtual size_t num_vertices() const = 0;

        virtual VertexIndex AddVertex(const Vertex& vertexToAdd) = 0;
        virtual TriangleMeshBuilderResultCodes AddTriangle(const TriangleByIndices& triangleToAdd) = 0;

        virtual std::unique_ptr<TriangleMesh> Mesh() = 0;
    };

}  // namespace Cork
