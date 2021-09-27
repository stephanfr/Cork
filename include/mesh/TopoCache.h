// +-------------------------------------------------------------------------
// | TopoCache.h
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

#include <tbb/spin_mutex.h>

#include <array>
#include <boost/container/small_vector.hpp>
#include <boost/container/static_vector.hpp>
#include <boost/dynamic_bitset.hpp>
#include <optional>

#include "MeshBase.h"
#include "intersection/empty3d.h"
#include "intersection/quantization.h"
#include "util/ManagedIntrusiveList.h"
#include "util/SparseVector.h"

namespace Cork
{
    /*
     *  Allows for topological algorithms to manipulate
     *  a more familiar pointer data structure based on a simplicial complex.
     *  This structure can be regenerated from the more basic
     *  vertex/triangle arrays using
     *      createTopoCache()
     *  Once manipulations have been satisfactorily performed,
     *  the underlying vertex/triangle arrays can be cleaned up for
     *  further use by topologically insensitive algorithms by
     *      commitTopoCache()
     */

#define INVALID_ID uint(-1)

    //	We need a couple forward declarations

    class TopoTri;
    class TopoEdge;

    typedef SEFUtility::SearchablePointerList<TopoTri, 10> TopoTrianglePointerList;
    typedef SEFUtility::SearchablePointerList<TopoEdge, 10> TopoEdgePointerList;

    class TopoVert final : public boost::noncopyable, public IntrusiveListHookNoDestructorOnElements
    {
       public:
        TopoVert() : m_ref(INVALID_ID), m_data(nullptr) {}

        explicit TopoVert(IndexType ref) : m_ref(ref), m_data(nullptr) {}

        ~TopoVert() {}

        IndexType ref() const { return (m_ref); }

        void setRef(IndexType newValue) { m_ref = newValue; }

        const Cork::Math::Vector3D* quantizedValue() const { return (m_data); }

        void setQuantizedValue(Cork::Math::Vector3D* newValue) { m_data = newValue; }

        void addTriangle(TopoTri& triangle) { m_tris.insert(&triangle); }

        const TopoTrianglePointerList& triangles() const { return (m_tris); }

        TopoTrianglePointerList& triangles() { return (m_tris); }

        const TopoEdgePointerList& edges() const { return (m_edges); }

        TopoEdgePointerList& edges() { return (m_edges); }

       private:
        IndexType m_ref;               // index to actual data
        Cork::Math::Vector3D* m_data;  // algorithm specific handle

        TopoTrianglePointerList m_tris;  // triangles this vertex is incident on
        TopoEdgePointerList m_edges;     // edges this vertex is incident on
    };

    typedef ManagedIntrusiveValueList<TopoVert> TopoVertexList;

    class TopoEdge final : public IntrusiveListHookNoDestructorOnElements
    {
       public:
        TopoEdge() {}

        TopoEdge(TopoVert* vertex0, TopoVert* vertex1) : m_verts({{vertex0, vertex1}})
        {
            vertex0->edges().insert(this);
            vertex1->edges().insert(this);
        }

        ~TopoEdge() {}

        void* data() const { return (m_data); }

        void setData(void* newValue) { m_data = newValue; }

        uint32_t boolAlgData() const { return (m_boolAlgData); }

        void setBoolAlgData(uint32_t newValue) { m_boolAlgData = newValue; }

        const std::array<TopoVert*, 2>& verts() const { return (m_verts); }

        std::array<TopoVert*, 2>& verts() { return (m_verts); }

        const TopoTrianglePointerList& triangles() const { return (m_tris); }

        TopoTrianglePointerList& triangles() { return (m_tris); }

        Cork::Math::BBox3D boundingBox() const
        {
            const Cork::Math::Vector3D& p0 = *(m_verts[0]->quantizedValue());
            const Cork::Math::Vector3D& p1 = *(m_verts[1]->quantizedValue());

            return Cork::Math::BBox3D(p0.min(p1), p0.max(p1));
        }

        NUMERIC_PRECISION length() const
        {
            const Cork::Math::Vector3D& p0 = *(m_verts[0]->quantizedValue());
            const Cork::Math::Vector3D& p1 = *(m_verts[1]->quantizedValue());

            return ((p0 - p1).len());
        }

        operator Empty3d::EdgeIn() const
        {
            return (Empty3d::EdgeIn(*(m_verts[0]->quantizedValue()), *(m_verts[1]->quantizedValue())));
        }

        ExteriorCalculusR4::GMPExt4_2 edgeExactCoordinates(const Quantization::Quantizer& quantizer) const
        {
            ExteriorCalculusR4::GMPExt4_1 ep[2];

            ep[0] = ExteriorCalculusR4::GMPExt4_1(*(m_verts[0]->quantizedValue()), quantizer);
            ep[1] = ExteriorCalculusR4::GMPExt4_1(*(m_verts[1]->quantizedValue()), quantizer);

            return ep[0].join(ep[1]);
        }

       private:
        void* m_data;  // algorithm specific handle

        uint32_t m_boolAlgData;

        std::array<TopoVert*, 2> m_verts;  // endpoint vertices
        TopoTrianglePointerList m_tris;    // incident triangles
    };

    typedef ManagedIntrusiveValueList<TopoEdge> TopoEdgeList;

    typedef boost::container::small_vector<const TopoEdge*, 24> TopoEdgePointerVector;

    // support structure for cache construction

    class TopoEdgePrototype : public SEFUtility::SparseVectorEntry
    {
       public:
        TopoEdgePrototype(IndexType v) : SparseVectorEntry(v), m_edge(nullptr) {}

        IndexType vid() const { return (index()); }

        TopoEdge* edge() { return (m_edge); }

        TopoEdge* setEdge(TopoEdge* edge)
        {
            m_edge = edge;

            return (m_edge);
        }

       private:
        TopoEdge* m_edge;
    };

    typedef SEFUtility::SparseVector<TopoEdgePrototype, 10> TopoEdgePrototypeVector;

    class TopoTri final : public boost::noncopyable, public IntrusiveListHookNoDestructorOnElements
    {
       public:
        TopoTri() {}

        explicit TopoTri(IndexType ref) : m_ref(ref) {}

        TopoTri(IndexType ref, TopoVert& vertex0, TopoVert& vertex1, TopoVert& vertex2) : m_ref(ref)
        {
            m_verts[0] = &vertex0;
            m_verts[1] = &vertex1;
            m_verts[2] = &vertex2;

            vertex0.triangles().insert(this);
            vertex1.triangles().insert(this);
            vertex2.triangles().insert(this);
        }

        TopoTri(const TopoTri&) = delete;

        ~TopoTri() {}

        IndexType ref() const { return (m_ref); }

        void setRef(IndexType newValue) { m_ref = newValue; }

        void* data() const { return (m_data); }

        void setData(void* newValue) { m_data = newValue; }

        void initVertices(TopoVert& vertex0, TopoVert& vertex1, TopoVert& vertex2)
        {
            m_verts[0] = &vertex0;
            m_verts[1] = &vertex1;
            m_verts[2] = &vertex2;

            vertex0.triangles().insert(this);
            vertex1.triangles().insert(this);
            vertex2.triangles().insert(this);
        }

#ifndef __AVX_AVAILABLE__
        //	Without SSE, the min/max computations as slow enough that caching the computed value is most efficient

        const Cork::Math::BBox3D& boundingBox() const
        {
            if (m_boundingBox.has_value())
            {
                return (m_boundingBox.value());
            }

            const Cork::Math::Vector3D& p0 = *(m_verts[0]->quantizedValue());
            const Cork::Math::Vector3D& p1 = *(m_verts[1]->quantizedValue());
            const Cork::Math::Vector3D& p2 = *(m_verts[2]->quantizedValue());

            const_cast<std::optional<Cork::Math::BBox3D>&>(m_boundingBox).emplace(p0.min(p1, p2), p0.max(p1, p2));

            return (m_boundingBox.value());
        }
#else
        //	With SSE, the min/max functions and bounding box computation is quick enough that computing the
        //		value every time is actually most efficient.

        const Cork::Math::BBox3D boundingBox() const
        {
            const Cork::Math::Vector3D& p0 = *(m_verts[0]->quantizedValue());
            const Cork::Math::Vector3D& p1 = *(m_verts[1]->quantizedValue());
            const Cork::Math::Vector3D& p2 = *(m_verts[2]->quantizedValue());

            return (Cork::Math::BBox3D(p0.min(p1, p2), p0.max(p1, p2)));
        }
#endif

        const NUMERIC_PRECISION minimumEdgeLength() const
        {
            return (std::min(m_edges[0]->length(), std::min(m_edges[1]->length(), m_edges[2]->length())));
        }

        const ExteriorCalculusR4::GMPExt4_3 triangleExactCoordinates(const Quantization::Quantizer& quantizer) const
        {
            ExteriorCalculusR4::GMPExt4_3 value;

            ExteriorCalculusR4::GMPExt4_1 p[3];

            p[0] = ExteriorCalculusR4::GMPExt4_1(*(m_verts[0]->quantizedValue()), quantizer);
            p[1] = ExteriorCalculusR4::GMPExt4_1(*(m_verts[1]->quantizedValue()), quantizer);
            p[2] = ExteriorCalculusR4::GMPExt4_1(*(m_verts[2]->quantizedValue()), quantizer);

            return (p[0].join(p[1])).join(p[2]);
        }

        uint32_t boolAlgData() const { return (m_boolAlgData); }

        void setBoolAlgData(uint32_t newValue) { m_boolAlgData = newValue; }

        void setVertices(std::array<TopoVert*, 3>& vertices)
        {
            memcpy(&m_verts, &vertices, sizeof(std::array<TopoVert*, 3>));

            m_verts[0]->triangles().insert(this);
            m_verts[1]->triangles().insert(this);
            m_verts[2]->triangles().insert(this);

#ifndef __AVX_AVAILABLE__
            m_boundingBox.reset();
#endif
        }

        const std::array<TopoVert*, 3>& verts() const { return (m_verts); }

        void setEdges(std::array<TopoEdge*, 3>& edges)
        {
            memcpy(&m_edges, &edges, sizeof(std::array<TopoEdge*, 3>));

            edges[0]->triangles().insert(this);
            edges[1]->triangles().insert(this);
            edges[2]->triangles().insert(this);
        }

        const std::array<TopoEdge*, 3>& edges() const { return (m_edges); }

        void flip()
        {
            std::swap(m_verts[0], m_verts[1]);
            std::swap(m_edges[0], m_edges[1]);
        }

        void AssignEdges(TopoVert* v0, TopoVert* v1, TopoVert* v2, TopoEdge* edge01, TopoEdge* edge02, TopoEdge* edge12)
        {
            if ((v0 != m_verts[0]) && (v1 != m_verts[0]))
            {
                m_edges[0] = edge01;

                if ((v0 != m_verts[1]) && (v2 != m_verts[1]))
                {
                    m_edges[1] = edge02;
                    m_edges[2] = edge12;
                }
                else
                {
                    m_edges[2] = edge02;
                    m_edges[1] = edge12;
                }
            }
            else if ((v0 != m_verts[1]) && (v1 != m_verts[1]))
            {
                m_edges[1] = edge01;

                if ((v0 != m_verts[0]) && (v2 != m_verts[0]))
                {
                    m_edges[0] = edge02;
                    m_edges[2] = edge12;
                }
                else
                {
                    m_edges[2] = edge02;
                    m_edges[0] = edge12;
                }
            }
            else if ((v0 != m_verts[2]) && (v1 != m_verts[2]))
            {
                m_edges[2] = edge01;

                if ((v0 != m_verts[0]) && (v2 != m_verts[0]))
                {
                    m_edges[0] = edge02;
                    m_edges[1] = edge12;
                }
                else
                {
                    m_edges[1] = edge02;
                    m_edges[0] = edge12;
                }
            }
        }

        bool hasCommonVertex(const TopoTri& triToCheck) const
        {
            return ((m_verts[0] == triToCheck.m_verts[0]) || (m_verts[0] == triToCheck.m_verts[1]) ||
                    (m_verts[0] == triToCheck.m_verts[2]) || (m_verts[1] == triToCheck.m_verts[0]) ||
                    (m_verts[1] == triToCheck.m_verts[1]) || (m_verts[1] == triToCheck.m_verts[2]) ||
                    (m_verts[2] == triToCheck.m_verts[0]) || (m_verts[2] == triToCheck.m_verts[1]) ||
                    (m_verts[2] == triToCheck.m_verts[2]));
        }

        bool findCommonVertex(const TopoTri& triToCheck, TopoVert*& commonVertex) const
        {
            for (uint i = 0; i < 3; i++)
            {
                for (uint j = 0; j < 3; j++)
                {
                    if (m_verts[i] == triToCheck.m_verts[j])
                    {
                        commonVertex = m_verts[i];
                        return (true);
                    }
                }
            }

            commonVertex = nullptr;

            return (false);
        }

        bool hasCommonVertex(const TopoEdge& edgeToCheck) const
        {
            return ((m_verts[0] == edgeToCheck.verts()[0]) || (m_verts[1] == edgeToCheck.verts()[0]) ||
                    (m_verts[2] == edgeToCheck.verts()[0]) || (m_verts[0] == edgeToCheck.verts()[1]) ||
                    (m_verts[1] == edgeToCheck.verts()[1]) || (m_verts[2] == edgeToCheck.verts()[1]));
        }

        operator Empty3d::TriIn() const
        {
            return (Empty3d::TriIn(*(m_verts[0]->quantizedValue()), *(m_verts[1]->quantizedValue()),
                                   *(m_verts[2]->quantizedValue())));
        }

        bool intersectsEdge(const TopoEdge& edgeToCheck, const Quantization::Quantizer& quantizer,
                            Empty3d::ExactArithmeticContext& arithContext) const
        {
            // must check whether the edge and triangle share a vertex
            // if so, then trivially we know they intersect in exactly that vertex
            // so we discard this case from consideration.

            if (hasCommonVertex(edgeToCheck))
            {
                return (false);
            }

            Empty3d::TriEdgeIn input(this->operator Empty3d::TriIn(), edgeToCheck.operator Empty3d::EdgeIn());

            return (!input.emptyExact(quantizer, arithContext));
        }

       private:
        IndexType m_ref;  // index to actual data
        void* m_data;     // algorithm specific handle

        uint32_t m_boolAlgData;

        std::array<TopoVert*, 3> m_verts;  // vertices of this triangle
        std::array<TopoEdge*, 3> m_edges;  // edges of this triangle opposite to the given vertex

#ifndef __AVX_AVAILABLE__
        std::optional<Cork::Math::BBox3D> m_boundingBox;
#endif
    };

    typedef ManagedIntrusiveValueList<TopoTri> TopoTriList;

    class TopoCacheWorkspace
    {
       public:
        TopoCacheWorkspace()
        {
            m_vertexListPool.reserve(1000000);
            m_edgeListPool.reserve(1000000);
            m_triListPool.reserve(1000000);
        }

        virtual ~TopoCacheWorkspace() {}

        void reset()
        {
            m_vertexListPool.clear();
            m_edgeListPool.clear();
            m_triListPool.clear();
        }

        operator TopoVertexList::PoolType &() { return (m_vertexListPool); }

        operator TopoEdgeList::PoolType &() { return (m_edgeListPool); }

        operator TopoTriList::PoolType &() { return (m_triListPool); }

       private:
        TopoVertexList::PoolType m_vertexListPool;
        TopoEdgeList::PoolType m_edgeListPool;
        TopoTriList::PoolType m_triListPool;
    };

    class TopoCache : public boost::noncopyable
    {
       public:
        TopoCache(MeshBase& owner, TopoCacheWorkspace& workspace);

        virtual ~TopoCache() {}

        // until commit() is called, the Mesh::verts and Mesh::tris
        // arrays will still contain garbage entries

        void commit();

        bool isValid();
        void print();

        MeshBase& ownerMesh() { return (m_mesh); }

        const MeshBase& ownerMesh() const { return (m_mesh); }

        const TopoTriList& triangles() const { return (m_topoTriList); }

        TopoTriList& triangles() { return (m_topoTriList); }

        const TopoEdgeList& edges() const { return (m_topoEdgeList); }

        TopoEdgeList& edges() { return (m_topoEdgeList); }

        const TopoVertexList& vertices() const { return (m_topoVertexList); }

        TopoVertexList& vertices() { return (m_topoVertexList); }

        // helpers to create bits and pieces

        TopoVert* newVert()
        {
            size_t ref = m_meshVertices.size();

            m_meshVertices.emplace_back();

            return (m_topoVertexList.emplace_back(ref));
        }

        TopoEdge* newEdge() { return (m_topoEdgeList.emplace_back()); }

        TopoTri* newTri()
        {
            IndexType ref = m_meshTriangles.size();

            m_meshTriangles.push_back(CorkTriangle());

            return (m_topoTriList.emplace_back(ref));
        }

        // helpers to release bits and pieces

        void freeVert(TopoVert* v) { m_topoVertexList.free(v); }

        void freeEdge(TopoEdge* e) { m_topoEdgeList.free(e); }

        void freeTri(TopoTri* t) { m_topoTriList.free(t); }

        // helper to delete geometry in a structured way

        void deleteTri(TopoTri* tri)
        {
            // first, unhook the triangle from its faces

            for (uint k = 0; k < 3; k++)
            {
                tri->verts()[k]->triangles().erase(tri);
                tri->edges()[k]->triangles().erase(tri);
            }

            // now, let's check for any edges which no longer border triangles

            for (uint k = 0; k < 3; k++)
            {
                TopoEdge* e = tri->edges()[k];

                if (e->triangles().empty())
                {
                    //	Unhook the edge from its vertices and delete it

                    e->verts()[0]->edges().erase(e);
                    e->verts()[1]->edges().erase(e);

                    freeEdge(e);
                }
            }

            // now, let's check for any vertices which no longer border triangles

            for (uint k = 0; k < 3; k++)
            {
                TopoVert* v = tri->verts()[k];

                if (v->triangles().empty())
                {
                    freeVert(v);
                }
            }

            // finally, release the triangle

            freeTri(tri);
        }

        // helper to flip triangle orientation

        void flipTri(TopoTri* t)
        {
            t->flip();
            m_meshTriangles[t->ref()].flip();
        }

       private:
        //	Data Members

        MeshBase& m_mesh;
        TopoCacheWorkspace& m_workspace;

        MeshBase::TriangleVector& m_meshTriangles;
        MeshBase::VertexVector& m_meshVertices;

        TopoVertexList m_topoVertexList;
        TopoEdgeList m_topoEdgeList;
        TopoTriList m_topoTriList;

        //	Methods

        void init();

        void initInternalSerial();
    };

    std::ostream& operator<<(std::ostream& out, const TopoVert& vertex);
    std::ostream& operator<<(std::ostream& out, const TopoEdge& edge);
    std::ostream& operator<<(std::ostream& out, const TopoTri& tri);

}  // namespace Cork
