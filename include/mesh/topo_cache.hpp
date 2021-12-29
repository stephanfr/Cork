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

#include <array>
#include <boost/container/small_vector.hpp>
#include <limits>
#include <type_traits>
#include <unordered_set>

#include "intersection/empty3d.hpp"
#include "math/quantization.hpp"
#include "primitives/boundary_edge.hpp"
#include "util/caching_factory.hpp"
#include "util/managed_intrusive_list.hpp"
#include "util/sparse_vector.hpp"

//  Forward declare the TriangleProblem class
//      I do not like doing this but the alternative gets messy wrt includes.

namespace Cork::Intersection
{
    class TriangleProblem;
}

namespace Cork::Meshes
{
    //
    //  Forward declare the MeshBase
    //

    class MeshBase;

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

    //	We need a couple forward declarations

    class TopoTri;
    class TopoEdge;

    using TopoTrianglePointerVector = boost::container::small_vector<const TopoTri*, 12>;
    using TopoEdgePointerVector = boost::container::small_vector<const TopoEdge*, 25>;

    class TopoVert final : public boost::noncopyable, public IntrusiveListHookNoDestructorOnElements
    {
       public:
        explicit TopoVert(VertexIndex index, Vertex3D quantized_coordinates)
            : index_(index), quantized_coordinates_(quantized_coordinates)
        {
        }

        TopoVert(const TopoVert&) = delete;
        TopoVert(TopoVert&&) = delete;

        ~TopoVert() {}

        TopoVert& operator=(const TopoVert&) = delete;
        TopoVert& operator=(TopoVert&&) = delete;

        VertexIndex index() const { return (index_); }

        void set_index(VertexIndex newValue) { index_ = newValue; }

        const Vertex3D& quantizedValue() const { return (quantized_coordinates_); }
        void perturb(const Vertex3D& perturbation) { quantized_coordinates_ += perturbation; }

        const TopoTrianglePointerVector& triangles() const { return triangles_; }
        void add_triangle(const TopoTri* triangle) { triangles_.emplace_back(triangle); }
        void remove_triangle(const TopoTri* tri) { std::remove(triangles_.begin(), triangles_.end(), tri); }

        const TopoEdgePointerVector& edges() const { return edges_; }
        void add_edge(const TopoEdge* edge) { edges_.emplace_back(edge); }
        void remove_edge(const TopoEdge* edge) { std::remove(edges_.begin(), edges_.end(), edge); }

        TopoEdgePointerVector& edges() { return edges_; }

       private:
        VertexIndex index_;  // index to actual data
        Vertex3D quantized_coordinates_;

        TopoTrianglePointerVector triangles_;  // triangles this vertex is incident on
        TopoEdgePointerVector edges_;    // edges this vertex is incident on
    };

    typedef ManagedIntrusiveValueList<TopoVert, VertexIndex> TopoVertexList;

    class TopoEdge final : public IntrusiveListHookNoDestructorOnElements
    {
       public:
        TopoEdge(TopoVert& vertex0, TopoVert& vertex1)
            : source_triangle_id_(Primitives::UNINTIALIZED_INDEX), vertices_({{&vertex0, &vertex1}})
        {
            vertex0.add_edge(this);
            vertex1.add_edge(this);
        }

        TopoEdge(TriangleByIndicesIndex source_triangle_id, TriangleEdgeId tri_edge_id, TopoVert& vertex0,
                 TopoVert& vertex1)
            : source_triangle_id_(source_triangle_id), tri_edge_id_(tri_edge_id), vertices_({{&vertex0, &vertex1}})
        {
            vertex0.add_edge(this);
            vertex1.add_edge(this);
        }

        ~TopoEdge() {}

        TriangleByIndicesIndex source_triangle_id() const { return source_triangle_id_; }
        TriangleEdgeId edge_index() const { return tri_edge_id_; }

        uint32_t boolean_algorithm_data() const { return (boolean_algorithm_data_); }

        void set_boolean_algorithm_data(uint32_t newValue) { boolean_algorithm_data_ = newValue; }

        const TopoVert& vert_0() const { return *(vertices_[0]); }
        const TopoVert& vert_1() const { return *(vertices_[1]); }

        const std::array<TopoVert*, 2>& verts() const { return vertices_; }

        std::array<TopoVert*, 2>& verts() { return vertices_; }

        const TopoTrianglePointerVector& triangles() const { return triangles_; }

        void add_triangle(const TopoTri* tri) { triangles_.emplace_back(tri); }
        void remove_triangle(const TopoTri* tri) { std::remove(triangles_.begin(), triangles_.end(), tri); }

        BBox3D boundingBox() const
        {
            const Vector3D& p0 = vertices_[0]->quantizedValue();
            const Vector3D& p1 = vertices_[1]->quantizedValue();

            return BBox3D(p0.min(p1), p0.max(p1));
        }

        NUMERIC_PRECISION length() const
        {
            const Vector3D& p0 = vertices_[0]->quantizedValue();
            const Vector3D& p1 = vertices_[1]->quantizedValue();

            return ((p0 - p1).len());
        }

        operator Empty3d::IntersectingEdge() const
        {
            return Empty3d::IntersectingEdge(vertices_[0]->quantizedValue(), vertices_[1]->quantizedValue());
        }

        Math::ExteriorCalculusR4::GMPExt4_2 edgeExactCoordinates(const Math::Quantizer& quantizer) const
        {
            Math::ExteriorCalculusR4::GMPExt4_1 ep[2];

            ep[0] = Math::ExteriorCalculusR4::GMPExt4_1(vertices_[0]->quantizedValue(), quantizer);
            ep[1] = Math::ExteriorCalculusR4::GMPExt4_1(vertices_[1]->quantizedValue(), quantizer);

            return ep[0].join(ep[1]);
        }

       private:
        TriangleByIndicesIndex source_triangle_id_;
        TriangleEdgeId tri_edge_id_;

        uint32_t boolean_algorithm_data_;

        std::array<TopoVert*, 2> vertices_;  // endpoint vertices
        TopoTrianglePointerVector triangles_;   // incident triangles
    };

    typedef ManagedIntrusiveValueList<TopoEdge> TopoEdgeList;

    typedef boost::container::small_vector<std::reference_wrapper<const TopoEdge>, 24> TopoEdgeReferenceVector;

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
        explicit TopoTri(IndexType ref) : ref_(ref) {}

        TopoTri(uint32_t source_triangle_id, IndexType ref, TopoVert& vertex0, TopoVert& vertex1, TopoVert& vertex2)
            : source_triangle_id_(source_triangle_id), ref_(ref)
        {
            m_verts[0] = &vertex0;
            m_verts[1] = &vertex1;
            m_verts[2] = &vertex2;

            vertex0.add_triangle(this);
            vertex1.add_triangle(this);
            vertex2.add_triangle(this);
        }

        TopoTri(const TopoTri&) = delete;

        ~TopoTri() {}

        uint32_t source_triangle_id() const { return source_triangle_id_; }

        TriangleByIndicesIndex ref() const { return ref_; }

        void set_ref(TriangleByIndicesIndex new_value) { ref_ = new_value; }

        const std::optional<std::reference_wrapper<Intersection::TriangleProblem>>& get_associated_triangle_problem()
            const
        {
            return associated_triangle_problem_;
        }

        void associate_triangle_problem(Intersection::TriangleProblem& associated_problem)
        {
            associated_triangle_problem_.emplace(associated_problem);
        }

        void clear_triangle_problem_association() { associated_triangle_problem_.reset(); }

#ifndef __AVX_AVAILABLE__
        //	Without SSE, the min/max computations as slow enough that caching the computed value is most efficient

        const BBox3D& boundingBox() const
        {
            if (m_boundingBox.has_value())
            {
                return (m_boundingBox.value());
            }

            const Vector3D& p0 = m_verts[0]->quantizedValue();
            const Vector3D& p1 = m_verts[1]->quantizedValue();
            const Vector3D& p2 = m_verts[2]->quantizedValue();

            const_cast<std::optional<BBox3D>&>(m_boundingBox).emplace(p0.min(p1, p2), p0.max(p1, p2));

            return (m_boundingBox.value());
        }
#else
        //	With SSE, the min/max functions and bounding box computation is quick enough that computing the
        //		value every time is actually most efficient.

        const BBox3D boundingBox() const
        {
            const Vector3D& p0 = m_verts[0]->quantizedValue();
            const Vector3D& p1 = m_verts[1]->quantizedValue();
            const Vector3D& p2 = m_verts[2]->quantizedValue();

            return (BBox3D(p0.min(p1, p2), p0.max(p1, p2)));
        }
#endif

        const NUMERIC_PRECISION minimumEdgeLength() const
        {
            return (std::min(m_edges[0]->length(), std::min(m_edges[1]->length(), m_edges[2]->length())));
        }

        const Math::ExteriorCalculusR4::GMPExt4_3 triangleExactCoordinates(const Math::Quantizer& quantizer) const
        {
            Math::ExteriorCalculusR4::GMPExt4_3 value;

            Math::ExteriorCalculusR4::GMPExt4_1 p[3];

            p[0] = Math::ExteriorCalculusR4::GMPExt4_1(m_verts[0]->quantizedValue(), quantizer);
            p[1] = Math::ExteriorCalculusR4::GMPExt4_1(m_verts[1]->quantizedValue(), quantizer);
            p[2] = Math::ExteriorCalculusR4::GMPExt4_1(m_verts[2]->quantizedValue(), quantizer);

            return (p[0].join(p[1])).join(p[2]);
        }

        uint32_t boolAlgData() const { return (m_boolAlgData); }

        void setBoolAlgData(uint32_t newValue) { m_boolAlgData = newValue; }

        void setVertices(std::array<TopoVert*, 3>& vertices)
        {
            memcpy(&m_verts, &vertices, sizeof(std::array<TopoVert*, 3>));

            m_verts[0]->add_triangle(this);
            m_verts[1]->add_triangle(this);
            m_verts[2]->add_triangle(this);

#ifndef __AVX_AVAILABLE__
            m_boundingBox.reset();
#endif
        }

        const std::array<TopoVert*, 3>& verts() const { return (m_verts); }

        void setEdges(std::array<TopoEdge*, 3>& edges)
        {
            memcpy(&m_edges, &edges, sizeof(std::array<TopoEdge*, 3>));

            edges[0]->add_triangle(this);
            edges[1]->add_triangle(this);
            edges[2]->add_triangle(this);
        }

        const std::array<TopoEdge*, 3>& edges() const { return (m_edges); }

        void flip()
        {
            std::swap(m_verts[0], m_verts[1]);
            std::swap(m_edges[0], m_edges[1]);
        }

        void AssignEdges(TopoVert& v0, TopoVert& v1, TopoVert& v2, TopoEdge& edge01, TopoEdge& edge02, TopoEdge& edge12)
        {
            if ((&v0 != m_verts[0]) && (&v1 != m_verts[0]))
            {
                m_edges[0] = &edge01;

                if ((&v0 != m_verts[1]) && (&v2 != m_verts[1]))
                {
                    m_edges[1] = &edge02;
                    m_edges[2] = &edge12;
                }
                else
                {
                    m_edges[2] = &edge02;
                    m_edges[1] = &edge12;
                }
            }
            else if ((&v0 != m_verts[1]) && (&v1 != m_verts[1]))
            {
                m_edges[1] = &edge01;

                if ((&v0 != m_verts[0]) && (&v2 != m_verts[0]))
                {
                    m_edges[0] = &edge02;
                    m_edges[2] = &edge12;
                }
                else
                {
                    m_edges[2] = &edge02;
                    m_edges[0] = &edge12;
                }
            }
            else if ((&v0 != m_verts[2]) && (&v1 != m_verts[2]))
            {
                m_edges[2] = &edge01;

                if ((&v0 != m_verts[0]) && (&v2 != m_verts[0]))
                {
                    m_edges[0] = &edge02;
                    m_edges[1] = &edge12;
                }
                else
                {
                    m_edges[1] = &edge02;
                    m_edges[0] = &edge12;
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

        operator Empty3d::IntersectingTriangle() const
        {
            return (Empty3d::IntersectingTriangle(m_verts[0]->quantizedValue(), m_verts[1]->quantizedValue(),
                                                  m_verts[2]->quantizedValue()));
        }

        bool intersectsEdge(const TopoEdge& edgeToCheck, const Math::Quantizer& quantizer,
                            Empty3d::ExactArithmeticContext& arithContext) const
        {
            // must check whether the edge and triangle share a vertex
            // if so, then trivially we know they intersect in exactly that vertex
            // so we discard this case from consideration.

            if (hasCommonVertex(edgeToCheck))
            {
                return (false);
            }

            Empty3d::TriangleEdgeIntersection input(this->operator Empty3d::IntersectingTriangle(),
                                                    edgeToCheck.operator Empty3d::IntersectingEdge());

            return input.emptyExact(quantizer, arithContext) != Empty3d::HasIntersection::YES;
        }

       private:
        TriangleByIndicesIndex ref_;  // index to actual data

        std::optional<std::reference_wrapper<Intersection::TriangleProblem>> associated_triangle_problem_;

        uint32_t source_triangle_id_;

        uint32_t m_boolAlgData;

        std::array<TopoVert*, 3> m_verts;  // vertices of this triangle
        std::array<TopoEdge*, 3> m_edges;  // edges of this triangle opposite to the given vertex

#ifndef __AVX_AVAILABLE__
        std::optional<BBox3D> m_boundingBox;
#endif
    };

    typedef ManagedIntrusiveValueList<TopoTri> TopoTriList;

    class TopoCacheWorkspace : public SEFUtility::Resettable
    {
       public:
        TopoCacheWorkspace()
        {
            m_vertexListPool.reserve(100000);
            m_edgeListPool.reserve(100000);
            m_triListPool.reserve(100000);
        }

        virtual ~TopoCacheWorkspace() {}

        void reset() { clear(); }

        void reset(size_t num_vertices, size_t num_edges, size_t num_triangles)
        {
            clear();

            m_vertexListPool.reserve(num_vertices);
            m_edgeListPool.reserve(num_edges);
            m_triListPool.reserve(num_triangles);
        }

        void clear()
        {
            m_vertexListPool.clear();
            m_edgeListPool.clear();
            m_triListPool.clear();
        }

        operator TopoVertexList::PoolType &() { return m_vertexListPool; }

        operator TopoEdgeList::PoolType &() { return m_edgeListPool; }

        operator TopoTriList::PoolType &() { return m_triListPool; }

        TopoVertexList::PoolType m_vertexListPool;
        TopoEdgeList::PoolType m_edgeListPool;
        TopoTriList::PoolType m_triListPool;
    };

    template <typename T>
    class TopoCacheBase
    {
       public:
        TopoCacheBase(T& triangles, Vertex3DVector& vertices, uint32_t num_edges, const Math::Quantizer& quantizer)
            : m_workspace(SEFUtility::CachingFactory<TopoCacheWorkspace>::GetInstance()),
              quantizer_(quantizer),
              mesh_triangles_(triangles),
              mesh_vertices_(vertices),
              num_edges_(num_edges),
              m_topoVertexList(*(m_workspace.get())),
              m_topoEdgeList(*(m_workspace.get())),
              m_topoTriList(*(m_workspace.get()))
        {
            m_workspace.get()->reset(mesh_vertices_.size(), num_edges_, mesh_triangles_.size());

            init();
        }

        virtual ~TopoCacheBase(){};

        const TopoTriList& triangles() const { return (m_topoTriList); }

        TopoTriList& triangles() { return (m_topoTriList); }

        const TopoEdgeList& edges() const { return (m_topoEdgeList); }

        TopoEdgeList& edges() { return (m_topoEdgeList); }

        const TopoVertexList& vertices() const { return (m_topoVertexList); }

        TopoVertexList& vertices() { return (m_topoVertexList); }

        const Math::Quantizer quantizer() const { return quantizer_; }

       private:
        TopoCacheBase() = delete;

        TopoCacheBase(const TopoCacheBase&) = delete;
        TopoCacheBase(TopoCacheBase&&) = delete;

        TopoCacheBase& operator=(const TopoCacheBase&) = delete;
        TopoCacheBase& operator=(TopoCacheBase&&) = delete;

        //	Data Members

       protected:
        SEFUtility::CachingFactory<TopoCacheWorkspace>::UniquePtr m_workspace;

        const Math::Quantizer quantizer_;

        T& mesh_triangles_;
        Vertex3DVector& mesh_vertices_;

        const uint32_t num_edges_;

        TopoVertexList m_topoVertexList;
        TopoEdgeList m_topoEdgeList;
        TopoTriList m_topoTriList;

        //	Methods

        void init()
        {
            //	First lay out vertices

            for (uint i = 0; i < mesh_vertices_.size(); i++)
            {
                m_topoVertexList.emplace_back(i, quantizer_.quantize(mesh_vertices_[VertexIndex(i)]));
            }

            // We need to still do the following
            //  * Generate TopoTris
            //  * Generate TopoEdges
            // ---- Hook up references between
            //  * Triangles and Vertices
            //  * Triangles and Edges
            //  * Vertices and Edges

            // We handle two of these items in a pass over the triangles,
            //  * Generate TopoTris
            //  * Hook up Triangles and Vertices
            // building a structure to handle the edges as we go:

            std::vector<TopoEdgePrototypeVector> edgeacc(mesh_vertices_.size());

            uint32_t i = -1;

            for (const TriangleByIndices& ref_tri : mesh_triangles_)
            {
                i++;

                // triangles <--> verts

                VertexIndex vertex0_index = ref_tri[0];
                VertexIndex vertex1_index = ref_tri[1];
                VertexIndex vertex2_index = ref_tri[2];

                TriangleVertexId vertex0_id = TriangleVertexId::A;
                TriangleVertexId vertex1_id = TriangleVertexId::B;
                TriangleVertexId vertex2_id = TriangleVertexId::C;

                TopoTri* tri = m_topoTriList.emplace_back(i, i, m_topoVertexList.getPool()[vertex0_index],
                                                          m_topoVertexList.getPool()[vertex1_index],
                                                          m_topoVertexList.getPool()[vertex2_index]);

                // then, put these in arbitrary but globally consistent order

                if (vertex0_index > vertex1_index)
                {
                    std::swap(vertex0_index, vertex1_index);
                    std::swap(vertex0_id, vertex1_id);
                }

                if (vertex1_index > vertex2_index)
                {
                    std::swap(vertex1_index, vertex2_index);
                    std::swap(vertex1_id, vertex2_id);
                }

                if (vertex0_index > vertex1_index)
                {
                    std::swap(vertex0_index, vertex1_index);
                    std::swap(vertex0_id, vertex1_id);
                }

                // and accrue in structure

                TopoVert& v0 = m_topoVertexList.getPool()[vertex0_index];
                TopoVert& v1 = m_topoVertexList.getPool()[vertex1_index];
                TopoVert& v2 = m_topoVertexList.getPool()[vertex2_index];

                //	Create edges and link them to the triangle

                TopoEdge* edge01;
                TopoEdge* edge02;
                TopoEdge* edge12;

                {
                    TopoEdgePrototype& edge01Proto = edgeacc[VertexIndex::integer_type(vertex0_index)].find_or_add(
                        VertexIndex::integer_type(vertex1_index));

                    edge01 = edge01Proto.edge();

                    if (edge01 == nullptr)
                    {
                        edge01 = edge01Proto.setEdge(m_topoEdgeList.emplace_back(
                            tri->source_triangle_id(), from_vertices(vertex0_id, vertex1_id), v0, v1));
                    }

                    edge01->add_triangle(tri);

                    TopoEdgePrototype& edge02Proto = edgeacc[VertexIndex::integer_type(vertex0_index)].find_or_add(
                        VertexIndex::integer_type(vertex2_index));

                    edge02 = edge02Proto.edge();

                    if (edge02 == nullptr)
                    {
                        edge02 = edge02Proto.setEdge(m_topoEdgeList.emplace_back(
                            tri->source_triangle_id(), from_vertices(vertex0_id, vertex2_id), v0, v2));
                    }

                    edge02->add_triangle(tri);

                    TopoEdgePrototype& edge12Proto = edgeacc[VertexIndex::integer_type(vertex1_index)].find_or_add(
                        VertexIndex::integer_type(vertex2_index));

                    edge12 = edge12Proto.edge();

                    if (edge12 == nullptr)
                    {
                        edge12 = edge12Proto.setEdge(m_topoEdgeList.emplace_back(
                            tri->source_triangle_id(), from_vertices(vertex1_id, vertex2_id), v1, v2));
                    }

                    edge12->add_triangle(tri);
                }
                //	We swapped around indices, so now fix the edge assignments

                tri->AssignEdges(v0, v1, v2, *edge01, *edge02, *edge12);
            }
        }
    };

    class TriangleByIndicesVectorTopoCache : public TopoCacheBase<TriangleByIndicesVector>
    {
       public:
        TriangleByIndicesVectorTopoCache(TriangleByIndicesVector& triangles, Vertex3DVector& vertices,
                                         uint32_t num_edges, const Math::Quantizer& quantizer);

        virtual ~TriangleByIndicesVectorTopoCache();

        void print();

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
                tri->verts()[k]->remove_triangle(tri);
                tri->edges()[k]->remove_triangle(tri);
            }

            // now, let's check for any edges which no longer border triangles

            for (uint k = 0; k < 3; k++)
            {
                TopoEdge* e = tri->edges()[k];

                if (e->triangles().empty())
                {
                    //	Unhook the edge from its vertices and delete it

                    e->verts()[0]->remove_edge(e);
                    e->verts()[1]->remove_edge(e);

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

       private:
        TriangleByIndicesVectorTopoCache() = delete;

        TriangleByIndicesVectorTopoCache(const TriangleByIndicesVectorTopoCache&) = delete;
        TriangleByIndicesVectorTopoCache(TriangleByIndicesVectorTopoCache&&) = delete;

        TriangleByIndicesVectorTopoCache& operator=(const TriangleByIndicesVectorTopoCache&) = delete;
        TriangleByIndicesVectorTopoCache& operator=(TriangleByIndicesVectorTopoCache&&) = delete;
    };

    class MeshTopoCache : public TriangleByIndicesVectorTopoCache
    {
       public:
        MeshTopoCache(MeshBase& owner, const Math::Quantizer& quantizer);

        virtual ~MeshTopoCache();

        // until commit() is called, the Mesh::verts and Mesh::tris
        // arrays will still contain garbage entries

        void commit();

        MeshBase& ownerMesh() { return (mesh_); }

        const MeshBase& ownerMesh() const { return (mesh_); }

        // helpers to create bits and pieces

        TopoVert* newVert()
        {
            VertexIndex::integer_type ref = mesh_vertices_.size();

            mesh_vertices_.emplace_back();

            return m_topoVertexList.emplace_back(ref, mesh_vertices_.back());
        }

        TopoEdge* newEdge(TopoVert& v0, TopoVert& v1) { return m_topoEdgeList.emplace_back(v0, v1); }

        TopoTri* newTri()
        {
            IndexType ref = mesh_triangles_.size();

            mesh_triangles_.push_back(TriangleByIndices());

            return (m_topoTriList.emplace_back(ref));
        }

        // helper to flip triangle orientation

        void flipTri(TopoTri* t)
        {
            t->flip();
            mesh_triangles_[t->ref()].flip();
        }

        //  Boundaries

        std::vector<const TopoEdge*> topo_edge_boundary(const BoundaryEdge& boundary) const;
        std::set<const TopoTri*> tris_along_edges(const std::vector<const TopoEdge*>& boundary) const;

       private:
        MeshTopoCache() = delete;

        MeshTopoCache(const MeshTopoCache&) = delete;
        MeshTopoCache(MeshTopoCache&&) = delete;

        MeshTopoCache& operator=(const MeshTopoCache&) = delete;
        MeshTopoCache& operator=(MeshTopoCache&&) = delete;

        //	Data Members

        MeshBase& mesh_;
    };

    /*
        class TriangleByIndicesVectorTopoCache : public TopoCacheBase<TriangleByIndicesVector>
        {
           public:
            TriangleByIndicesVectorTopoCache(TriangleByIndicesVector& triangles, Vertex3DVector& vertices, uint32_t
       num_edges, const Math::Quantizer& quantizer);

            virtual ~TriangleByIndicesVectorTopoCache();

            //        TopoEdge* newEdge() { return (m_topoEdgeList.emplace_back()); }

            TopoEdge* newEdge(TopoVert& v0, TopoVert& v1) { return m_topoEdgeList.emplace_back(v0, v1); }

            TopoTri* newTri()
            {
                return (m_topoTriList.emplace_back(-1));
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

           private:
            TriangleByIndicesVectorTopoCache() = delete;

            TriangleByIndicesVectorTopoCache(const TriangleByIndicesVectorTopoCache&) = delete;
            TriangleByIndicesVectorTopoCache(TriangleByIndicesVectorTopoCache&&) = delete;

            TriangleByIndicesVectorTopoCache& operator=(const TriangleByIndicesVectorTopoCache&) = delete;
            TriangleByIndicesVectorTopoCache& operator=(TriangleByIndicesVectorTopoCache&&) = delete;
        };
    */

    std::ostream& operator<<(std::ostream& out, const TopoVert& vertex);
    std::ostream& operator<<(std::ostream& out, const TopoEdge& edge);
    std::ostream& operator<<(std::ostream& out, const TopoTri& tri);

}  // namespace Cork::Meshes
