// +-------------------------------------------------------------------------
// | intersection.cpp
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

#include <boost/container/small_vector.hpp>
#include <optional>

#include "mesh/topo_cache.hpp"
#include "util/managed_intrusive_list.hpp"

namespace Cork::Intersection
{
    const uint32_t DEFAULT_INTERSECTION_VERTEX_COLLECTION_SIZE = 32;
    const uint32_t DEFAULT_EDGE_COLLECTION_SIZE = 32;
    const uint32_t DEFAULT_INTERSECTION_VERTEX_LIST_SIZE = 32;

    //	A couple of forward declarations.

    class GluePointMarker;
    class GenericEdgeType;

    class GenericVertType : public IntrusiveListHookNoDestructorOnElements
    {
        using TopoVert = Meshes::TopoVert;
        using TopoEdge = Meshes::TopoEdge;
        using TopoTri = Meshes::TopoTri;

       public:
        enum class VertexType
        {
            INTERSECTION,
            ORIGINAL
        };

        using EdgeCollection = boost::container::small_vector<GenericEdgeType*, DEFAULT_EDGE_COLLECTION_SIZE>;

        GenericVertType(VertexType vertex_type, TopoVert& concrete_vertex, const Primitives::Vector3D& coordinate)
            : vertex_type_(vertex_type),
              concrete_vertex_(&concrete_vertex),
              coordinate_(coordinate),
              is_boundary_(false)
        {
        }

        GenericVertType(VertexType vertex_type, TopoVert& concrete_vertex, const Primitives::Vector3D& coordinate,
                        bool is_boundary)
            : vertex_type_(vertex_type),
              concrete_vertex_(&concrete_vertex),
              coordinate_(coordinate),
              is_boundary_(is_boundary)
        {
        }

        GenericVertType(VertexType vertex_type, TopoVert& concrete_vertex, const Primitives::Vector3D& coordinate,
                        bool is_boundary, GluePointMarker& glue_marker);

        GenericVertType(VertexType vertex_type, const Primitives::Vector3D& coordinate, bool is_boundary,
                        GluePointMarker& glue_marker);

        GenericVertType() = delete;

        GenericVertType(const GenericVertType&) = delete;
        GenericVertType(GenericVertType&&) = delete;

        ~GenericVertType() = default;

        GenericVertType& operator=(const GenericVertType&) = delete;
        GenericVertType& operator=(GenericVertType&&) = delete;

        [[nodiscard]] VertexType vertex_type() const { return (vertex_type_); }

        [[nodiscard]] TopoVert& concrete_vertex()
        {
            assert(concrete_vertex_.has_value());
            return *(concrete_vertex_.value());
        }

        void set_concrete_vertex(TopoVert& concreteVertex) { concrete_vertex_.emplace(&concreteVertex); }

        [[nodiscard]] const Primitives::Vector3D& coordinate() const { return (coordinate_); }

        [[nodiscard]] bool is_boundary() const { return is_boundary_; }

        void set_boundary(bool new_value) { is_boundary_ = new_value; }

        [[nodiscard]] const EdgeCollection& edges() const { return (edges_); }

        [[nodiscard]] EdgeCollection& edges() { return (edges_); }

        [[nodiscard]] uint index() const { return (index_); }

        void setIndex(uint index) { index_ = index; }

        [[nodiscard]] const GluePointMarker& glueMarker() const
        {
            assert(glue_marker_.has_value());
            return *(glue_marker_.value());
        }

        [[nodiscard]] GluePointMarker& glueMarker() { return *(glue_marker_.value()); }

        inline void removeFromGlueMarkerCopies();

       private:
        VertexType vertex_type_;
        std::optional<TopoVert*> concrete_vertex_;
        bool is_boundary_;

        uint index_ = 0;  // temporary for triangulation marshalling

        Primitives::Vector3D coordinate_;

        EdgeCollection edges_;

        std::optional<GluePointMarker*> glue_marker_;
    };

    using GenericVertTypeList = ManagedIntrusiveValueList<GenericVertType>;

    using OrigVertType = GenericVertType;
    using IsctVertType = GenericVertType;

    using IsctVertTypeList = ManagedIntrusiveValueList<IsctVertType>;
    using IntersectionVertexPointerList = ManagedIntrusivePointerList<IsctVertType>;
    using OrigVertTypeList = ManagedIntrusiveValueList<OrigVertType>;

    class GluePointMarker : public IntrusiveListHookNoDestructorOnElements
    {
        using TopoVert = Meshes::TopoVert;
        using TopoEdge = Meshes::TopoEdge;
        using TopoTri = Meshes::TopoTri;

       public:
        using IntersectionVertexCollection =
            boost::container::small_vector<IsctVertType*, DEFAULT_INTERSECTION_VERTEX_COLLECTION_SIZE>;

        enum class IntersectionType
        {
            EDGE_TRIANGLE,
            TRIANGLE_TRIANGLE_TRIANGLE
        };

        GluePointMarker() = delete;
        GluePointMarker(const GluePointMarker&) = delete;
        GluePointMarker(GluePointMarker&&) = delete;

        GluePointMarker(IntersectionType intersection_type, const TopoEdge& edge, const TopoTri& tisct)
            : intersection_type_(intersection_type), edge_(&edge), triangles_({{&tisct, nullptr, nullptr}})
        {
        }

        GluePointMarker(IntersectionType intersection_type, const TopoTri& tisct0, const TopoTri& tisct1,
                        const TopoTri& tisct2)
            : intersection_type_(intersection_type), triangles_({{&tisct0, &tisct1, &tisct2}})
        {
        }

        ~GluePointMarker() = default;

        GluePointMarker& operator=(const GluePointMarker&) = delete;
        GluePointMarker& operator=(GluePointMarker&&) = delete;

        [[nodiscard]] IntersectionType intersection_type() const { return intersection_type_; }

        [[nodiscard]] const TopoEdge& edge()
        {
            assert(edge_.has_value());
            return *(edge_.value());
        }

        [[nodiscard]] const std::array<const TopoTri*, 3>& triangles() const { return triangles_; }

        [[nodiscard]] const IntersectionVertexCollection& vertices_to_be_glued() const { return vertices_to_be_glued_; }

        void add_vertex_to_be_glued(IsctVertType* vertex) { vertices_to_be_glued_.push_back(vertex); }

        void remove_vertex_to_be_glued(IsctVertType* vertex)
        {
            vertices_to_be_glued_.erase(std::find(vertices_to_be_glued_.begin(), vertices_to_be_glued_.end(), vertex));
        }

       private:
        IntersectionVertexCollection vertices_to_be_glued_;  //  all of the vertices which will need to be glued
        const std::array<const TopoTri*, 3> triangles_;

        IntersectionType intersection_type_;

        std::optional<const TopoEdge*> edge_;
    };

    using GluePointMarkerList = ManagedIntrusiveValueList<GluePointMarker>;

    class GenericEdgeType : public IntrusiveListHookNoDestructorOnElements
    {
        using TopoVert = Meshes::TopoVert;
        using TopoEdge = Meshes::TopoEdge;
        using TopoTri = Meshes::TopoTri;

       public:
        enum class EdgeType
        {
            INTERSECTION,
            ORIGINAL,
            SPLIT
        };

        using IntersectionVertexList =
            boost::container::small_vector<IsctVertType*, DEFAULT_INTERSECTION_VERTEX_LIST_SIZE>;

        GenericEdgeType(EdgeType edge_type, bool is_boundary, GenericVertType* endpoint)
            : edge_type_(edge_type), is_boundary_(is_boundary), ends_{endpoint, nullptr}
        {
            endpoint->edges().push_back(this);
        }

        GenericEdgeType(EdgeType edge_type, const TopoEdge& concrete, bool is_boundary, GenericVertType* endpoint)
            : edge_type_(edge_type), concrete_(&concrete), is_boundary_(is_boundary), ends_{endpoint, nullptr}
        {
            endpoint->edges().push_back(this);
        }

        GenericEdgeType(EdgeType edge_type, bool is_boundary, GenericVertType* endpoint1, GenericVertType* endpoint2)
            : edge_type_(edge_type), is_boundary_(is_boundary), ends_{endpoint1, endpoint2}
        {
            endpoint1->edges().push_back(this);
            endpoint2->edges().push_back(this);
        }

        GenericEdgeType(EdgeType edge_type, const TopoEdge& concrete, bool is_boundary, GenericVertType* endpoint1,
                        GenericVertType* endpoint2)
            : edge_type_(edge_type), concrete_(&concrete), is_boundary_(is_boundary), ends_{endpoint1, endpoint2}
        {
            endpoint1->edges().push_back(this);
            endpoint2->edges().push_back(this);
        }

        GenericEdgeType(EdgeType edge_type, const TopoEdge& concrete, bool is_boundary, GenericVertType* endpoint,
                        const TopoTri& other_triangle_key)
            : edge_type_(edge_type),
              concrete_(&concrete),
              is_boundary_(is_boundary),
              other_triangle_key_(&other_triangle_key),
              ends_{endpoint, nullptr}
        {
            endpoint->edges().push_back(this);
        }

        GenericEdgeType(EdgeType edge_type, bool is_boundary, GenericVertType* endpoint,
                        const TopoTri& other_triangle_key)
            : edge_type_(edge_type),
              is_boundary_(is_boundary),
              other_triangle_key_(&other_triangle_key),
              ends_{endpoint, nullptr}
        {
            endpoint->edges().push_back(this);
        }

        GenericEdgeType() = delete;

        GenericEdgeType(const GenericEdgeType&) = delete;
        GenericEdgeType(GenericEdgeType&&) = delete;

        ~GenericEdgeType() = default;

        GenericEdgeType& operator=(const GenericEdgeType&) = delete;
        GenericEdgeType& operator=(GenericEdgeType&&) = delete;

        [[nodiscard]] EdgeType edgeType() const { return (edge_type_); }

        [[nodiscard]] const TopoEdge& concrete() const
        {
            assert(concrete_.has_value());
            return *(concrete_.value());
        }

        [[nodiscard]] bool is_boundary() const { return (is_boundary_); }

        [[nodiscard]] const std::array<GenericVertType*, 2>& ends() const { return ends_; }

        [[nodiscard]] std::array<GenericVertType*, 2>& ends() { return ends_; }

        [[nodiscard]] const IntersectionVertexList& interior() const { return interior_; }

        [[nodiscard]] IntersectionVertexList& interior() { return interior_; }

        [[nodiscard]] const TopoTri& otherTriKey() const
        {
            assert(other_triangle_key_.has_value());
            return *(other_triangle_key_.value());
        }

        void disconnect()
        {
            ends_[0]->edges().erase(std::find(ends_[0]->edges().begin(), ends_[0]->edges().end(), this));
            ends_[1]->edges().erase(std::find(ends_[1]->edges().begin(), ends_[1]->edges().end(), this));

            for (IsctVertType* iv : interior_)
            {
                iv->edges().erase(std::find(iv->edges().begin(), iv->edges().end(), this));
            }
        }

       private:
        EdgeType edge_type_;

        std::optional<const TopoEdge*> concrete_;

        bool is_boundary_;

        std::array<GenericVertType*, 2> ends_;

        IntersectionVertexList interior_;

        std::optional<const TopoTri*> other_triangle_key_;  // use to detect duplicate instances within a triangle
    };

    using GenericEdgeTypeList = ManagedIntrusiveValueList<GenericEdgeType>;

    using IsctEdgeType = GenericEdgeType;
    using OrigEdgeType = GenericEdgeType;
    using SplitEdgeType = GenericEdgeType;

    using IsctEdgeTypeList = GenericEdgeTypeList;
    using OrigEdgeTypeList = GenericEdgeTypeList;
    using SplitEdgeTypeList = GenericEdgeTypeList;

    using IntersectionEdgePointerList = ManagedIntrusivePointerList<IsctEdgeType>;

    class GenericTriType : public IntrusiveListHookNoDestructorOnElements
    {
        using TopoVert = Meshes::TopoVert;
        using TopoEdge = Meshes::TopoEdge;
        using TopoTri = Meshes::TopoTri;

       public:
        GenericTriType(GenericVertType* v0, GenericVertType* v1, GenericVertType* v2) : vertices_{v0, v1, v2} {}

        GenericTriType() = delete;

        GenericTriType(const GenericTriType&) = delete;
        GenericTriType(GenericTriType&&) = delete;

        ~GenericTriType() = default;

        GenericTriType& operator=(const GenericTriType&) = delete;
        GenericTriType& operator=(GenericTriType&&) = delete;

        [[nodiscard]] const TopoTri& concrete_triangle() const
        {
            assert(concrete_triangle_.has_value());
            return *(concrete_triangle_.value());
        }

        void set_concrete_triangle(TopoTri* concrete_triangle) { concrete_triangle_ = concrete_triangle; }

        [[nodiscard]] const std::array<GenericVertType*, 3>& vertices() const { return (vertices_); }

       private:
        std::optional<TopoTri*> concrete_triangle_;

        std::array<GenericVertType*, 3> vertices_;
    };

    using GenericTriTypeList = ManagedIntrusiveValueList<GenericTriType>;
    using GenericTriPointerList = ManagedIntrusivePointerList<GenericTriType>;

    //
    //  Inlined constructor and methods from above
    //

    inline GenericVertType::GenericVertType(VertexType vertex_type, TopoVert& concrete_vertex,
                                            const Primitives::Vector3D& coordinate, bool is_boundary,
                                            GluePointMarker& glue_marker)
        : vertex_type_(vertex_type),
          concrete_vertex_(&concrete_vertex),
          coordinate_(coordinate),
          is_boundary_(is_boundary),
          glue_marker_(&glue_marker)
    {
        glue_marker_.value()->add_vertex_to_be_glued(this);
    }

    inline GenericVertType::GenericVertType(VertexType vertex_type, const Primitives::Vector3D& coordinate,
                                            bool is_boundary, GluePointMarker& glue_marker)
        : vertex_type_(vertex_type), coordinate_(coordinate), is_boundary_(is_boundary), glue_marker_(&glue_marker)
    {
        glue_marker_.value()->add_vertex_to_be_glued(this);
    }

    inline void GenericVertType::removeFromGlueMarkerCopies() { glue_marker_.value()->remove_vertex_to_be_glued(this); }

}  // namespace Cork::Intersection