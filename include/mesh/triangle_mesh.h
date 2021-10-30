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
#include "primitives/primitives.hpp"
#include "statistics/statistics.h"

namespace Cork::Meshes
{

        
    enum class TopologicalStatisticsResultCodes
    {
        SUCCESS = 0,

        ANALYSIS_FAILED
    };

    using TopologicalStatisticsResult = SEFUtility::ResultWithReturnValue<TopologicalStatisticsResultCodes,Statistics::TopologicalStatistics>;
    
    enum class HoleClosingResultCodes
    {
        SUCCESS = 0,

        TRIANGULATION_FAILED
    };

    using HoleClosingResult = SEFUtility::Result<HoleClosingResultCodes>;


    class TriangleMesh
    {
       public:
        //	Methods follow

        virtual ~TriangleMesh(){};

        virtual size_t numTriangles() const = 0;
        virtual size_t numVertices() const = 0;

        virtual const Primitives::Vertex3DVector& vertices() const = 0;
        virtual const Primitives::TriangleByIndicesVector& triangles() const = 0;

        virtual Primitives::TriangleByVertices triangleByVertices(
            const Primitives::TriangleByIndices& triangleByIndices) const = 0;

        virtual void AddTriangle(const Primitives::TriangleByIndices& triangle_to_add) = 0;
        virtual void remove_triangle(Primitives::TriangleByIndicesIndex triangle_index) = 0;

        virtual const Primitives::BBox3D& boundingBox() const = 0;
        virtual Primitives::MinAndMaxEdgeLengths min_and_max_edge_lengths() const = 0;
        virtual double max_vertex_magnitude() const = 0;

        virtual Statistics::GeometricStatistics ComputeGeometricStatistics(Statistics::GeometricProperties props_to_compute ) const = 0;
        virtual TopologicalStatisticsResult ComputeTopologicalStatistics(Statistics::TopologicalProperties     props_to_compute) const = 0;

        virtual HoleClosingResult close_holes( const Statistics::TopologicalStatistics&     topo_stats ) = 0;
        virtual void remove_self_intersections( const Statistics::TopologicalStatistics&     topo_stats ) = 0;
    };

    enum class TriangleMeshBuilderResultCodes
    {
        SUCCESS = 0,

        VERTEX_INDEX_OUT_OF_BOUNDS,

        MESH_NOT_2_MANIFOLD
    };

    using TriangleMeshBuilderResult = SEFUtility::ResultWithReturnUniquePtr<TriangleMeshBuilderResultCodes, TriangleMesh>;

    class IncrementalVertexIndexTriangleMeshBuilder
    {
       public:
        static std::unique_ptr<IncrementalVertexIndexTriangleMeshBuilder> GetBuilder(size_t numVertices = 0,
                                                                                     size_t numTriangles = 0);

        virtual ~IncrementalVertexIndexTriangleMeshBuilder(){};

        virtual size_t num_vertices() const = 0;

        virtual Primitives::VertexIndex AddVertex(const Primitives::Vertex3D& vertexToAdd) = 0;
        virtual TriangleMeshBuilderResultCodes AddTriangle(Primitives::VertexIndex a, Primitives::VertexIndex b, Primitives::VertexIndex c) = 0;

        virtual std::unique_ptr<TriangleMesh> Mesh() = 0;
    };

}  // namespace Cork::Meshes
