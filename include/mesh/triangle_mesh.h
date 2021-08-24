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

// The following ifdef block is the standard way of creating macros which make exporting
// from a DLL simpler. All files within this DLL are compiled with the CORKLIB_EXPORTS
// symbol defined on the command line. This symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see
// CORKLIB_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.
#ifdef WINDOWS
#ifdef CORKLIB_EXPORTS
#define CORKLIB_API __declspec(dllexport)
#else
#define CORKLIB_API __declspec(dllimport)
#endif
#else
#define CORKLIB_API
#endif

#include <array>
#include <string>
#include <vector>

#include "math/BoundingBox.h"
#include "statistics/statistics.h"
#include "util/Result.h"

namespace Cork
{
    class TriangleMesh
    {
       public:
        //	Include the relevent types for the triangle mesh into the class itself.
        //		May look verbose in code but forces crystal clarity on class associations.

        typedef Cork::IndexType VertexIndexType;

        typedef Cork::Math::Vertex3D Vertex;
        typedef Cork::Math::Vertex3DVector VertexVector;

        typedef Cork::Math::TriangleByIndicesBase TriangleByIndices;

        typedef Cork::Math::TriangleByVerticesBase TriangleByVertices;

        //	Methods follow

        virtual ~TriangleMesh(){};

        virtual size_t numTriangles() const = 0;
        virtual size_t numVertices() const = 0;

        virtual const VertexVector& vertices() const = 0;
        virtual const std::vector<TriangleByIndices>& triangles() const = 0;

        virtual TriangleByVertices triangleByVertices(const TriangleByIndices& triangleByIndices) const = 0;

        //		virtual SelfIntersectionStatistics					ComputeSelfIntersectionStatistics() const = 0;

        virtual const Cork::Math::BBox3D& boundingBox() const = 0;

        virtual Cork::Statistics::GeometricStatistics ComputeGeometricStatistics() const = 0;
        virtual Cork::Statistics::TopologicalStatistics ComputeTopologicalStatistics() const = 0;
    };

    enum class TriangleMeshBuilderResultCodes
    {
        SUCCESS = 0,

        VERTEX_INDEX_OUT_OF_BOUNDS,

        MESH_NOT_2_MANIFOLD
    };

    typedef SEFUtility::ResultWithUniqueReturnPtr<TriangleMeshBuilderResultCodes, TriangleMesh>
        TriangleMeshBuilderResult;

    class IncrementalVertexIndexTriangleMeshBuilder
    {
       public:
        static std::unique_ptr<IncrementalVertexIndexTriangleMeshBuilder> GetBuilder(size_t numVertices = 0,
                                                                                     size_t numTriangles = 0);

        virtual ~IncrementalVertexIndexTriangleMeshBuilder(){};

        virtual TriangleMesh::VertexIndexType AddVertex(const TriangleMesh::Vertex& vertexToAdd) = 0;
        virtual TriangleMeshBuilderResultCodes AddTriangle(const TriangleMesh::TriangleByIndices& triangleToAdd) = 0;

        virtual std::unique_ptr<TriangleMesh> Mesh() = 0;
    };

}  // namespace Cork
