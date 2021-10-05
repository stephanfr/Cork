// +-------------------------------------------------------------------------
// | MeshBase.h
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

#include <functional>
#include <optional>
#include <vector>

#include "cork.h"
#include "intersection/quantization.h"
#include "math/Primitives.h"
#include "triangle_mesh.h"

namespace Cork
{
    class SolverPerfStats : public SolverPerformanceStatisticsIfx
    {
       public:
        SolverPerfStats()
            : m_numberOfTrianglesInDisjointUnion(0),
              m_numberOfTrianglesInFinalMesh(0),
              m_elapsedCPUTimeInNanoSeconds(0),
              m_elapsedWallTimeInNanoSeconds(0),
              m_startingVirtualMemorySizeInMB(0),
              m_endingVirtualMemorySizeInMB(0)
        {
        }

        uint64_t numberOfTrianglesInDisjointUnion() const { return (m_numberOfTrianglesInDisjointUnion); }

        uint64_t numberOfTrianglesInFinalMesh() const { return (m_numberOfTrianglesInFinalMesh); }

        uint64_t elapsedCPUTimeInNanoSeconds() const { return (m_elapsedCPUTimeInNanoSeconds); }

        uint64_t elapsedWallTimeInNanoSeconds() const { return (m_elapsedWallTimeInNanoSeconds); }

        uint64_t startingVirtualMemorySizeInMB() const { return (m_startingVirtualMemorySizeInMB); }

        uint64_t endingVirtualMemorySizeInMB() const { return (m_endingVirtualMemorySizeInMB); }

        void setNumberOfTrianglesInDisjointUnion(uint64_t numberOfTrianglesInDisjointUnion)
        {
            m_numberOfTrianglesInDisjointUnion = numberOfTrianglesInDisjointUnion;
        }

        void setNumberOfTrianglesInFinalMesh(uint64_t numberOfTrianglesInFinalMesh)
        {
            m_numberOfTrianglesInFinalMesh = numberOfTrianglesInFinalMesh;
        }

        void setElapsedCPUTimeInNanoSeconds(uint64_t elapsedCPUTimeInNanoSeconds)
        {
            m_elapsedCPUTimeInNanoSeconds = elapsedCPUTimeInNanoSeconds;
        }

        void setElapsedWallTimeInNanoSeconds(uint64_t elapsedWallTimeInNanoSeconds)
        {
            m_elapsedWallTimeInNanoSeconds = elapsedWallTimeInNanoSeconds;
        }

        void setStartingVirtualMemorySizeInMB(uint64_t startingVirtualMemorySizeInMB)
        {
            m_startingVirtualMemorySizeInMB = startingVirtualMemorySizeInMB;
        }

        void setEndingVirtualMemorySizeInMB(uint64_t endingVirtualMemorySizeInMB)
        {
            m_endingVirtualMemorySizeInMB = endingVirtualMemorySizeInMB;
        }

       private:
        uint64_t m_numberOfTrianglesInDisjointUnion;
        uint64_t m_numberOfTrianglesInFinalMesh;

        uint64_t m_elapsedCPUTimeInNanoSeconds;
        uint64_t m_elapsedWallTimeInNanoSeconds;

        uint64_t m_startingVirtualMemorySizeInMB;
        uint64_t m_endingVirtualMemorySizeInMB;
    };

    class CorkTriangle : public Math::TriangleByIndicesBase
    {
       public:
        CorkTriangle() {}
        /*
                CorkTriangle( const CorkTriangle&		triangleToCopy )
                    : Math::TriangleByIndicesBase( (const Math::TriangleByIndicesBase&)triangleToCopy ),
                      m_boolAlgData( triangleToCopy.m_boolAlgData )
                {
                    memcpy( &m_a, &triangleToCopy.m_a, sizeof( unsigned int ) * 3 );
                }
        */

        CorkTriangle(const TriangleByIndices& triangleToCopy, uint32_t boolAlgData, uint32_t triangle_id)
            : triangle_id_( triangle_id ), Math::TriangleByIndicesBase(triangleToCopy), m_boolAlgData(boolAlgData)
        {
        }

        uint32_t        triangle_id() const { return triangle_id_; }

        const uint32_t boolAlgData() const { return (m_boolAlgData); }

        uint32_t& boolAlgData() { return (m_boolAlgData); }

        void flip() { std::swap(m_a, m_b); }

        void offsetIndices(size_t offsetValue)
        {
            m_a += offsetValue;
            m_b += offsetValue;
            m_c += offsetValue;
        }

       private:
        uint32_t m_boolAlgData;  // internal use by algorithm - value must be copied when the triangle is subdivided

        uint32_t        triangle_id_;
    };

    typedef Math::Vector3D CorkVertex;

    class MeshBase
    {
       public:
        typedef std::vector<CorkTriangle> TriangleVector;
        typedef Math::Vertex3DVector VertexVector;

        MeshBase() {}

        MeshBase(const MeshBase& meshBaseToCopy, const SolverControlBlock& controlBlock)
            : m_boundingBox(meshBaseToCopy.m_boundingBox),
              min_and_max_edge_lengths_(meshBaseToCopy.min_and_max_edge_lengths_),
              max_vertex_magnitude_(meshBaseToCopy.max_vertex_magnitude_),
              m_tris(meshBaseToCopy.m_tris),
              m_verts(meshBaseToCopy.m_verts),
              m_controlBlock(controlBlock)
        {
        }

        virtual ~MeshBase() {}

        const SolverControlBlock& solverControlBlock() const { return (*m_controlBlock); }

        TriangleVector& triangles() { return (m_tris); }

        const TriangleVector& triangles() const { return (m_tris); }

        VertexVector& vertices() { return (m_verts); }

        const VertexVector& vertices() const { return (m_verts); }

        const Math::BBox3D& boundingBox() const { return m_boundingBox; }
        Math::MinAndMaxEdgeLengths min_and_max_edge_lengths() const { return min_and_max_edge_lengths_; }
        double max_vertex_magnitude() const { return max_vertex_magnitude_; }

        const Quantization::Quantizer::GetQuantizerResult getQuantizer() const
        {
            return Quantization::Quantizer::get_quantizer(max_vertex_magnitude_, min_and_max_edge_lengths_.min());
        }

        void for_raw_tris(std::function<void(IndexType, IndexType, IndexType)> func)
        {
            for (auto& tri : m_tris)
            {
                func(tri.a(), tri.b(), tri.c());
            }
        }

        void for_raw_tris(std::function<void(IndexType, IndexType, IndexType)> func) const
        {
            for (auto& tri : m_tris)
            {
                func(tri.a(), tri.b(), tri.c());
            }
        }

       protected:
        TriangleVector m_tris;
        VertexVector m_verts;

        Math::BBox3D m_boundingBox;

        Math::MinAndMaxEdgeLengths min_and_max_edge_lengths_;
        double max_vertex_magnitude_;

        std::optional<SolverControlBlock> m_controlBlock;

        SolverPerfStats m_performanceStats;
    };

}  // namespace Cork