// +-------------------------------------------------------------------------
// | Primitives.h
// |
// | Author: Stephan Friedl
// +-------------------------------------------------------------------------
// | COPYRIGHT:
// |    Copyright Stephan Friedl 2015
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

#include "type_safe/integer.hpp"

//	Setup the numeric precision and the vector implementation for the build.

#include "CorkDefs.h"

//  Include some basic templates and types we will use to create the primitives for Cork

#include "math/bounding_box_3D_template.hpp"
#include "math/min_and_max_lengths.hpp"
#include "math/ray_3D_template.hpp"
#include "math/vector_2D_template.hpp"
#include "math/vector_3D_template.hpp"

namespace Cork::Primitives
{
    constexpr uint32_t UNINTIALIZED_INDEX = -1;

    using Vector2D = Math::Vector2DTemplate<NUMERIC_PRECISION>;
    using Vector3D = Math::Vector3DTemplate<NUMERIC_PRECISION>;

    using Vector3DVector = std::vector<Primitives::Vector3D>;

    //	Vertices and vectors share the same implementation of a numeric 3-tuple

    using Vertex3D = Vector3D;

    using Ray3D = Math::Ray3DTemplate<NUMERIC_PRECISION>;
    using Ray3DWithInverseDirection = Math::Ray3DWithInverseDirectionTemplate<NUMERIC_PRECISION>;

    using BBox3D = Math::BBox3DTemplate<NUMERIC_PRECISION>;

    using MinAndMaxEdgeLengths = Math::MinAndMaxLengths;

    using IndexType = uint32_t;
    using IndexVector = std::vector<IndexType>;

    using VertexIndex = type_safe::integer<uint32_t>;

    using TriangleByIndicesIndex = type_safe::integer<uint32_t>;

    //  Define some enums to track vertex and edge assignments

    enum class TriangleVertexId : uint8_t
    {
        A = 0,
        B,
        C
    };

    enum class TriangleEdgeId : uint8_t
    {
        AB = 0,
        BC,
        CA
    };

    inline TriangleEdgeId from_vertices(TriangleVertexId vert_one, TriangleVertexId vert_two)
    {
        TriangleEdgeId result;

        switch (vert_one)
        {
            case TriangleVertexId::A:

                switch (vert_two)
                {
                    case TriangleVertexId::B:
                        result = TriangleEdgeId::AB;
                        break;
                    case TriangleVertexId::C:
                        result = TriangleEdgeId::CA;
                        break;
                }
                break;

            case TriangleVertexId::B:

                switch (vert_two)
                {
                    case TriangleVertexId::A:
                        result = TriangleEdgeId::AB;
                        break;
                    case TriangleVertexId::C:
                        result = TriangleEdgeId::BC;
                        break;
                }
                break;

            case TriangleVertexId::C:

                switch (vert_two)
                {
                    case TriangleVertexId::A:
                        result = TriangleEdgeId::CA;
                        break;
                    case TriangleVertexId::B:
                        result = TriangleEdgeId::BC;
                        break;
                }
                break;
        }

        return result;
    }

    inline std::ostream& operator<<(std::ostream& out, TriangleVertexId tri_vertex_id)
    {
        constexpr std::array<const char*, 3> labels{{"VertexA", "VertexB", "VertexC"}};

        out << labels[static_cast<size_t>(tri_vertex_id)];

        return out;
    }

    inline std::ostream& operator<<(std::ostream& out, TriangleEdgeId tri_edge_id)
    {
        constexpr std::array<const char*, 3> labels{{"EdgeAB", "EdgeBC", "EdgeCA"}};

        out << labels[static_cast<size_t>(tri_edge_id)];

        return out;
    }

    class Vertex3DVector : public std::vector<Vertex3D>
    {
       public:
        const Vertex3D& operator[](size_t) const = delete;
        Vertex3D& operator[](size_t) = delete;

        const Vertex3D& operator[](VertexIndex idx) const { return data()[VertexIndex::integer_type(idx)]; }

        Vertex3D& operator[](VertexIndex idx) { return data()[VertexIndex::integer_type(idx)]; }
    };

    struct Vertex3DMapCompare
    {
        bool operator()(const Vertex3D& vertex1, const Vertex3D& vertex2) const
        {
            //	Equality is by x, then y and finally z

            if (vertex1.x() < vertex2.x())
            {
                return (true);
            }

            if (vertex1.x() > vertex2.x())
            {
                return (false);
            }

            //	X values are equal

            if (vertex1.y() < vertex2.y())
            {
                return (true);
            }

            if (vertex1.y() > vertex2.y())
            {
                return (false);
            }

            //	X and Y values are equal

            if (vertex1.z() < vertex2.z())
            {
                return (true);
            }

            if (vertex1.z() > vertex2.z())
            {
                return (false);
            }

            //	The only way we should end up down here is if the two vertices are equal

            return (false);
        }
    };

    //
    //	Define some base classes for key mesh classes: Triangle by vertex indices, Edges and Triangle by vertex points.
    //

    class TriangleByIndices
    {
       public:
        using UIDType = uint32_t;

        TriangleByIndices()
            : uid_(UNINTIALIZED_INDEX), a_(UNINTIALIZED_INDEX), b_(UNINTIALIZED_INDEX), c_(UNINTIALIZED_INDEX)
        {
        }

        TriangleByIndices(UIDType uid, VertexIndex a, VertexIndex b, VertexIndex c) : uid_(uid), a_(a), b_(b), c_(c) {}

        virtual ~TriangleByIndices() {}

        const VertexIndex operator[](size_t index) const
        {
            assert(index < 3);
            return (reinterpret_cast<const VertexIndex*>(&a_))[index];
        }

        VertexIndex& operator[](size_t index)
        {
            assert(index < 3);
            return (reinterpret_cast<VertexIndex*>(&a_))[index];
        }

        UIDType uid() const { return uid_; }

        const VertexIndex a() const { return a_; }

        VertexIndex& a() { return a_; }

        const VertexIndex b() const { return b_; }

        VertexIndex& b() { return b_; }

        const VertexIndex c() const { return c_; }

        VertexIndex& c() { return c_; }

       protected:
        UIDType uid_;

        VertexIndex a_;
        VertexIndex b_;
        VertexIndex c_;
    };

    inline std::ostream& operator<<(std::ostream& out, const TriangleByIndices& tri_by_indices)
    {
        out << "(" << tri_by_indices.a() << ", " << tri_by_indices.b() << ", " << tri_by_indices.c() << ")";
        return out;
    }

    class TriangleByIndicesVector : public std::vector<TriangleByIndices>
    {
       public:
        const TriangleByIndices& operator[](size_t) const = delete;
        TriangleByIndices& operator[](size_t) = delete;

        const TriangleByIndices& operator[](Primitives::TriangleByIndicesIndex idx) const
        {
            return data()[Primitives::TriangleByIndicesIndex::integer_type(idx)];
        }
        TriangleByIndices& operator[](Primitives::TriangleByIndicesIndex idx)
        {
            return data()[Primitives::TriangleByIndicesIndex::integer_type(idx)];
        }
    };

    class EdgeByIndices
    {
       public:
        //  Edges are always smaller vertex index first, larger index second

        EdgeByIndices(VertexIndex first_vertex, VertexIndex second_vertex)
            : first_vertex_(std::min(first_vertex, second_vertex)),
              second_vertex_(std::max(first_vertex, second_vertex))
        {
        }

        virtual ~EdgeByIndices() {}

        VertexIndex first() const { return (first_vertex_); }

        VertexIndex second() const { return (second_vertex_); }

        bool operator==(const EdgeByIndices& edgeToCompare) const
        {
            return ((first() == edgeToCompare.first()) && (second() == edgeToCompare.second()));
        }

        bool contains_vertex(VertexIndex vert_index) const
        {
            return (first_vertex_ == vert_index) || (second_vertex_ == vert_index);
        }

        bool same_segment(const EdgeByIndices& edgeToCompare) const
        {
            return ((first() == edgeToCompare.first()) && (second() == edgeToCompare.second())) ||
                   ((first() == edgeToCompare.second()) && (second() == edgeToCompare.first()));
        }

        bool shares_vertex(const EdgeByIndices& edgeToCompare) const
        {
            if (same_segment(edgeToCompare))
            {
                return false;
            }

            return (first() == edgeToCompare.first()) || (first() == edgeToCompare.second()) ||
                   (second() == edgeToCompare.first()) || (second() == edgeToCompare.second());
        }

        EdgeByIndices flip_segment() const { return EdgeByIndices(second_vertex_, first_vertex_); }

       private:
        VertexIndex first_vertex_;
        VertexIndex second_vertex_;
    };

    using EdgeByIndicesVector = std::vector<EdgeByIndices>;

    class EdgeByVertices
    {
       public:
        EdgeByVertices(const Vertex3D& first_vertex, const Vertex3D& second_vertex)
            : first_vertex_(first_vertex), second_vertex_(second_vertex)
        {
        }

        virtual ~EdgeByVertices() {}

        const Vertex3D& first() const { return (first_vertex_); }

        const Vertex3D& second() const { return (second_vertex_); }

        bool operator==(const EdgeByVertices& edgeToCompare) const
        {
            return ((first() == edgeToCompare.first()) && (second() == edgeToCompare.second()));
        }

       private:
        const Vertex3D first_vertex_;
        const Vertex3D second_vertex_;
    };

    inline std::ostream& operator<<(std::ostream& out, const EdgeByVertices& edge_by_vertices)
    {
        out << "(" << edge_by_vertices.first() << ", " << edge_by_vertices.second() << ")";
        return out;
    }

    class TriangleByVertices
    {
       public:
        TriangleByVertices(const Vertex3D& firstVertex, const Vertex3D& secondVertex, const Vertex3D& thirdVertex)
            : a_(firstVertex), b_(secondVertex), c_(thirdVertex)
        {
        }

        TriangleByVertices(const TriangleByIndices& triangle, const Vertex3DVector& vertices)
            : a_(vertices[triangle.a()]), b_(vertices[triangle.b()]), c_(vertices[triangle.c()])
        {
        }

        const Vertex3D& vertexA() const { return a_; }

        const Vertex3D& vertexB() const { return b_; }

        const Vertex3D& vertexC() const { return c_; }

        Vector3D edgeAB_from_origin() const { return (a_ - b_); }

        Vector3D edgeAC_from_origin() const { return (a_ - c_); }

        Vector3D edgeBC_from_origin() const { return (b_ - c_); }

        EdgeByVertices edge(TriangleEdgeId edge_id)
        {
            switch (edge_id)
            {
                case TriangleEdgeId::AB:
                    return EdgeByVertices(a_, b_);
                    break;

                case TriangleEdgeId::BC:
                    return EdgeByVertices(b_, c_);
                    break;
            }

            return EdgeByVertices(c_, a_);
        }

        //        TriangleEdgeId

        BBox3D bounding_box() const
        {
            return BBox3D(
                Vector3D(std::min(a_.x(), std::min(b_.x(), c_.x())), std::min(a_.y(), std::min(b_.y(), c_.y())),
                         std::min(a_.z(), std::min(b_.z(), c_.z()))),
                Vector3D(std::max(a_.x(), std::max(b_.x(), c_.x())), std::max(a_.y(), std::max(b_.y(), c_.y())),
                         std::max(a_.z(), std::max(b_.z(), c_.z()))));
        }

        MinAndMaxEdgeLengths min_and_max_edge_lengths()
        {
            double edge_ab_len = edgeAB_from_origin().len_squared();
            double edge_ac_len = edgeAC_from_origin().len_squared();
            double edge_bc_len = edgeBC_from_origin().len_squared();

            double min_squared = std::min(edge_ab_len, std::min(edge_ac_len, edge_bc_len));
            double max_squared = std::max(edge_ab_len, std::max(edge_ac_len, edge_bc_len));

            return MinAndMaxEdgeLengths::from_squares(min_squared, max_squared);
        }

        double max_magnitude_vertex() const
        {
            return std::max(c_.abs().max(), std::max(a_.abs().max(), b_.abs().max()));
        }

       private:
        Vertex3D a_;
        Vertex3D b_;
        Vertex3D c_;
    };

    inline std::ostream& operator<<(std::ostream& out, const TriangleByVertices& tri_by_vertices)
    {
        out << "(" << tri_by_vertices.vertexA() << ", " << tri_by_vertices.vertexB() << ", "
            << tri_by_vertices.vertexC() << ")";
        return out;
    }

}  // namespace Cork::Primitives
