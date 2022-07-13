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
    //	A handful of forward declarations.
    //
    //      The following classes are somewhat intertwined and though I could pry them apart, it would end up moving
    //      code out of the class declarations which would make things a bit less transparent.

    class GenericVertType;
    using IsctVertType = GenericVertType;

    class GenericEdgeType;

    class GluePointMarker : public IntrusiveListHook
    {
        using TopoVert = Meshes::TopoVert;
        using TopoEdge = Meshes::TopoEdge;
        using TopoTri = Meshes::TopoTri;

       public:
        using IntersectionVertexCollection = boost::container::small_vector<IsctVertType*, 32>;

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

        IntersectionType intersection_type() const { return intersection_type_; }

        const TopoEdge& edge()
        {
            assert(edge_.has_value());
            return *(edge_.value());
        }

        const std::array<const TopoTri*, 3>& triangles() const { return triangles_; }

        const IntersectionVertexCollection& vertices_to_be_glued() const { return vertices_to_be_glued_; }

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

    class GenericVertType : public IntrusiveListHook
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

        using EdgeCollection = boost::container::small_vector<GenericEdgeType*, 32>;

        GenericVertType(VertexType vertex_type, TopoVert& concrete_vertex, const Primitives::Vector3D& coordinate)
            : vertex_type_(vertex_type), concrete_vertex_(&concrete_vertex), coordinate_(coordinate)
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
                        bool is_boundary, GluePointMarker& glue_marker)
            : vertex_type_(vertex_type),
              concrete_vertex_(&concrete_vertex),
              coordinate_(coordinate),
              is_boundary_(is_boundary),
              glue_marker_(&glue_marker)
        {
            glue_marker_.value()->add_vertex_to_be_glued(this);
        }

        GenericVertType(VertexType vertex_type, const Primitives::Vector3D& coordinate, bool is_boundary,
                        GluePointMarker& glue_marker)
            : vertex_type_(vertex_type), coordinate_(coordinate), is_boundary_(is_boundary), glue_marker_(&glue_marker)
        {
            glue_marker_.value()->add_vertex_to_be_glued(this);
        }

        GenericVertType() = delete;

        GenericVertType(const GenericVertType&) = delete;
        GenericVertType(GenericVertType&&) = delete;

        ~GenericVertType() {}

        GenericVertType& operator=(const GenericVertType&) = delete;
        GenericVertType& operator=(GenericVertType&&) = delete;

        const VertexType vertex_type() const { return (vertex_type_); }

        TopoVert& concrete_vertex()
        {
            assert(concrete_vertex_.has_value());
            return *(concrete_vertex_.value());
        }

        void set_concrete_vertex(TopoVert& concreteVertex) { concrete_vertex_.emplace(&concreteVertex); }

        const Primitives::Vector3D& coordinate() const { return (coordinate_); }

        bool is_boundary() const { return is_boundary_; }

        void set_boundary(bool new_value) { is_boundary_ = new_value; }

        const EdgeCollection& edges() const { return (edges_); }

        EdgeCollection& edges() { return (edges_); }

        uint index() const { return (index_); }

        void setIndex(uint index) { index_ = index; }

        const GluePointMarker& glueMarker() const { assert( glue_marker_.has_value() ); return *(glue_marker_.value()); }

        GluePointMarker& glueMarker() { return *(glue_marker_.value()); }

        void removeFromGlueMarkerCopies() { glue_marker_.value()->remove_vertex_to_be_glued(this); }

       private:
        VertexType vertex_type_;
        std::optional<TopoVert*> concrete_vertex_;
        bool is_boundary_;

        uint index_;  // temporary for triangulation marshalling

        Primitives::Vector3D coordinate_;

        EdgeCollection edges_;

        std::optional<GluePointMarker*> glue_marker_;
    };

    using OrigVertType = GenericVertType;

    using IsctVertTypeList = ManagedIntrusiveValueList<IsctVertType>;
    using IntersectionVertexPointerList = ManagedIntrusivePointerList<IsctVertType>;
    using OrigVertTypeList = ManagedIntrusiveValueList<OrigVertType>;

    class GenericEdgeType : public IntrusiveListHook
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

        using IntersectionVertexList = boost::container::small_vector<IsctVertType*, 32>;

        GenericEdgeType(EdgeType edge_type, bool is_boundary, GenericVertType* endpoint)
            : edge_type_(edge_type), is_boundary_(is_boundary)
        {
            ends_[0] = endpoint;
            endpoint->edges().push_back(this);

            ends_[1] = nullptr;
        }

        GenericEdgeType(EdgeType edge_type, const TopoEdge& concrete, bool is_boundary, GenericVertType* endpoint)
            : edge_type_(edge_type), concrete_(&concrete), is_boundary_(is_boundary)
        {
            ends_[0] = endpoint;
            endpoint->edges().push_back(this);

            ends_[1] = nullptr;
        }

        GenericEdgeType(EdgeType edge_type, bool is_boundary, GenericVertType* endpoint1, GenericVertType* endpoint2)
            : edge_type_(edge_type), is_boundary_(is_boundary)
        {
            ends_[0] = endpoint1;
            endpoint1->edges().push_back(this);

            ends_[1] = endpoint2;
            endpoint2->edges().push_back(this);
        }

        GenericEdgeType(EdgeType edge_type, const TopoEdge& concrete, bool is_boundary, GenericVertType* endpoint1,
                        GenericVertType* endpoint2)
            : edge_type_(edge_type), concrete_(&concrete), is_boundary_(is_boundary)
        {
            ends_[0] = endpoint1;
            endpoint1->edges().push_back(this);

            ends_[1] = endpoint2;
            endpoint2->edges().push_back(this);
        }

        GenericEdgeType(EdgeType edge_type, const TopoEdge& concrete, bool is_boundary, GenericVertType* endpoint,
                        const TopoTri& other_triangle_key)
            : edge_type_(edge_type),
              concrete_(&concrete),
              is_boundary_(is_boundary),
              other_triangle_key_(&other_triangle_key)
        {
            ends_[0] = endpoint;
            endpoint->edges().push_back(this);

            ends_[1] = nullptr;
        }

        GenericEdgeType(EdgeType edge_type, bool is_boundary, GenericVertType* endpoint,
                        const TopoTri& other_triangle_key)
            : edge_type_(edge_type), is_boundary_(is_boundary), other_triangle_key_(&other_triangle_key)
        {
            ends_[0] = endpoint;
            endpoint->edges().push_back(this);

            ends_[1] = nullptr;
        }

        GenericEdgeType() = delete;

        GenericEdgeType(const GenericEdgeType&) = delete;
        GenericEdgeType(GenericEdgeType&&) = delete;

        ~GenericEdgeType() {}

        GenericEdgeType& operator=(const GenericEdgeType&) = delete;
        GenericEdgeType& operator=(GenericEdgeType&&) = delete;

        EdgeType edgeType() const { return (edge_type_); }

        const TopoEdge& concrete() const
        {
            assert(concrete_.has_value());
            return *(concrete_.value());
        }

        bool is_boundary() const { return (is_boundary_); }

        const std::array<GenericVertType*, 2>& ends() const { return ends_; }

        std::array<GenericVertType*, 2>& ends() { return ends_; }

        const IntersectionVertexList& interior() const { return interior_; }

        IntersectionVertexList& interior() { return interior_; }

        const TopoTri& otherTriKey() const
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

    using IsctEdgeType = GenericEdgeType;
    using OrigEdgeType = GenericEdgeType;
    using SplitEdgeType = GenericEdgeType;

    using IsctEdgeTypeList = ManagedIntrusiveValueList<IsctEdgeType>;
    using OrigEdgeTypeList = ManagedIntrusiveValueList<OrigEdgeType>;
    using SplitEdgeTypeList = ManagedIntrusiveValueList<SplitEdgeType>;

    using IntersectionEdgePointerList = ManagedIntrusivePointerList<IsctEdgeType>;

    class GenericTriType : public IntrusiveListHook
    {
        using TopoVert = Meshes::TopoVert;
        using TopoEdge = Meshes::TopoEdge;
        using TopoTri = Meshes::TopoTri;

       public:
        GenericTriType(GenericVertType* v0, GenericVertType* v1, GenericVertType* v2)
        {
            vertices_[0] = v0;
            vertices_[1] = v1;
            vertices_[2] = v2;
        }

        GenericTriType() = delete;

        GenericTriType(const GenericTriType&) = delete;
        GenericTriType(GenericTriType&&) = delete;

        ~GenericTriType() {}

        GenericTriType& operator=(const GenericTriType&) = delete;
        GenericTriType& operator=(GenericTriType&&) = delete;

        const TopoTri& concrete_triangle() const
        {
            assert(concrete_triangle_.has_value());
            return *(concrete_triangle_.value());
        }

        void set_concrete_triangle(TopoTri* concrete_triangle) { concrete_triangle_ = concrete_triangle; }

        const std::array<GenericVertType*, 3> vertices() const { return (vertices_); }

       private:
        std::optional<TopoTri*> concrete_triangle_;

        std::array<GenericVertType*, 3> vertices_;
    };

    using GenericTriTypeList = ManagedIntrusiveValueList<GenericTriType>;
    using GenericTriPointerList = ManagedIntrusivePointerList<GenericTriType>;

}  // namespace Cork::Intersection