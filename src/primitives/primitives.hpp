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

#include <algorithm>
#include <set>

#include "type_safe/integer.hpp"
#include "type_safe/strong_typedef.hpp"

//	Setup the numeric precision and the vector implementation for the build.

#include "cork_defs.hpp"

//  Include some basic templates and types we will use to create the primitives for Cork

#include "math/bounding_box_3D_template.hpp"
#include "math/min_and_max_lengths.hpp"
#include "math/ray_3D_template.hpp"
#include "math/vector_2D_template.hpp"
#include "math/vector_3D_template.hpp"

namespace Cork::Primitives
{
    constexpr uint32_t UNINITIALIZED_INDEX = -1;

    using Vector2D = Math::Vector2DTemplate<NUMERIC_PRECISION>;
    using Vector3D = Math::Vector3DTemplate<NUMERIC_PRECISION>;

    using Vector3DVector = std::vector<Primitives::Vector3D>;

    //	Vertices and vectors share the same implementation of a numeric 3-tuple

    using Vertex2D = Vector2D;
    using Vertex3D = Vector3D;

    using Ray3D = Math::Ray3DTemplate<NUMERIC_PRECISION>;
    using Ray3DWithInverseDirection = Math::Ray3DWithInverseDirectionTemplate<NUMERIC_PRECISION>;

    using BBox3D = Math::BBox3DTemplate<NUMERIC_PRECISION>;

    using MinAndMaxEdgeLengths = Math::MinAndMaxLengths;

    using IndexType = uint32_t;
    using IndexVector = std::vector<IndexType>;

    struct VertexIndex : type_safe::strong_typedef<VertexIndex, size_t>,
                         type_safe::strong_typedef_op::equality_comparison<VertexIndex>,
                         type_safe::strong_typedef_op::relational_comparison<VertexIndex>,
                         type_safe::strong_typedef_op::integer_arithmetic<VertexIndex>
    {
        using strong_typedef::strong_typedef;

        bool operator<(size_t value_to_compare) const { return this->value_ < value_to_compare; }

        bool operator>=(size_t value_to_compare) const { return this->value_ >= value_to_compare; }

        VertexIndex operator+(size_t value_to_add) const { return VertexIndex(this->value_ + value_to_add); }

        friend std::ostream& operator<<(std::ostream& os, const VertexIndex& vi)
        {
            os << vi.value_;
            return os;
        }

        friend std::istream& operator>>(std::istream& is, VertexIndex& vi)
        {
            is >> vi.value_;
            return is;
        }
    };

    constexpr VertexIndex UNINITIALIZED_VERTEX_INDEX{size_t(-1)};

    using VertexIndexVector = std::vector<VertexIndex>;

    struct TriangleByIndicesIndex : type_safe::strong_typedef<TriangleByIndicesIndex, uint32_t>,
                                    type_safe::strong_typedef_op::equality_comparison<TriangleByIndicesIndex>,
                                    type_safe::strong_typedef_op::relational_comparison<TriangleByIndicesIndex>,
                                    type_safe::strong_typedef_op::integer_arithmetic<TriangleByIndicesIndex>
    {
        using strong_typedef::strong_typedef;

        TriangleByIndicesIndex() = default;

        explicit TriangleByIndicesIndex(size_t value) { value_ = value; }

        TriangleByIndicesIndex(TriangleByIndicesIndex&&) = default;
        TriangleByIndicesIndex(const TriangleByIndicesIndex&) = default;

        ~TriangleByIndicesIndex() = default;

        TriangleByIndicesIndex& operator=(const TriangleByIndicesIndex&) = default;
        TriangleByIndicesIndex& operator=(TriangleByIndicesIndex&&) = delete;

        explicit operator size_t() const { return value_; }

        bool operator<(size_t value_to_compare) const { return this->value_ < value_to_compare; }

        TriangleByIndicesIndex operator+(size_t value_to_add) const
        {
            return TriangleByIndicesIndex(this->value_ + value_to_add);
        }

        TriangleByIndicesIndex operator+(uint32_t value_to_add) const
        {
            return TriangleByIndicesIndex(this->value_ + value_to_add);
        }

        friend std::ostream& operator<<(std::ostream& os, const TriangleByIndicesIndex& vi)
        {
            os << vi.value_;
            return os;
        }

        friend std::istream& operator>>(std::istream& is, TriangleByIndicesIndex& vi)
        {
            is >> vi.value_;
            return is;
        }
    };

    using TriangleUID = type_safe::integer<uint64_t>;

    using TriangleBooleanAlgData = uint32_t;

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

    //  The from_vertices method has a number of issues with the linter, but is fine - so we will simply not lint

    // NOLINTBEGIN
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
    //  NOLINTEND

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
        Vertex3DVector() = default;

        Vertex3DVector(const std::vector<Vertex3D>& vec_to_copy) : std::vector<Vertex3D>(vec_to_copy) {}

        Vertex3DVector(std::vector<Vertex3D>&& vec_to_move) : std::vector<Vertex3D>(std::move(vec_to_move)) {}

        const Vertex3D& operator[](size_t) const = delete;
        Vertex3D& operator[](size_t) = delete;

        const Vertex3D& operator[](VertexIndex idx) const { return data()[static_cast<size_t>(idx)]; }

        Vertex3D& operator[](VertexIndex idx) { return data()[static_cast<size_t>(idx)]; }
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

    class EdgeByIndices
    {
       public:
        EdgeByIndices() = default;
        EdgeByIndices(const EdgeByIndices&) = default;
        EdgeByIndices(EdgeByIndices&&) = default;

        //  Edges are always smaller vertex index first, larger index second

        EdgeByIndices(VertexIndex first_vertex, VertexIndex second_vertex)
            : first_vertex_(std::min(first_vertex, second_vertex)),
              second_vertex_(std::max(first_vertex, second_vertex))
        {
        }

        virtual ~EdgeByIndices() = default;

        EdgeByIndices& operator=(const EdgeByIndices&) = default;
        EdgeByIndices& operator=(EdgeByIndices&&) = default;

        [[nodiscard]] VertexIndex first() const { return (first_vertex_); }

        [[nodiscard]] VertexIndex second() const { return (second_vertex_); }

        bool operator==(const EdgeByIndices& edgeToCompare) const
        {
            return ((first() == edgeToCompare.first()) && (second() == edgeToCompare.second()));
        }

        [[nodiscard]] bool contains_vertex(VertexIndex vert_index) const
        {
            return (first_vertex_ == vert_index) || (second_vertex_ == vert_index);
        }

        [[nodiscard]] bool same_segment(const EdgeByIndices& edgeToCompare) const
        {
            return ((first() == edgeToCompare.first()) && (second() == edgeToCompare.second())) ||
                   ((first() == edgeToCompare.second()) && (second() == edgeToCompare.first()));
        }

        [[nodiscard]] bool shares_vertex(const EdgeByIndices& edgeToCompare) const
        {
            if (same_segment(edgeToCompare))
            {
                return false;
            }

            return (first() == edgeToCompare.first()) || (first() == edgeToCompare.second()) ||
                   (second() == edgeToCompare.first()) || (second() == edgeToCompare.second());
        }

        [[nodiscard]] EdgeByIndices flip_segment() const { return EdgeByIndices(second_vertex_, first_vertex_); }

        struct HashFunction
        {
            std::size_t operator()(const Primitives::EdgeByIndices& k) const
            {
                return (static_cast<size_t>(k.first()) * 10000019 ^ static_cast<size_t>(k.second()));
            }
        };

       private:
        VertexIndex first_vertex_;
        VertexIndex second_vertex_;
    };

    struct EdgeByIndicesMapCompare
    {
        bool operator()(const EdgeByIndices& edge1, const EdgeByIndices& edge2) const
        {
            //	Compare the smallest indices first and then the largest indices

            if (std::min(edge1.first(), edge1.second()) < std::min(edge2.first(), edge2.second()))
            {
                return (true);
            }

            if (std::min(edge1.first(), edge1.second()) > std::min(edge2.first(), edge2.second()))
            {
                return (false);
            }

            //	Smallest indices are equal - check the largest next

            if (std::max(edge1.first(), edge1.second()) < std::max(edge2.first(), edge2.second()))
            {
                return (true);
            }

            if (std::max(edge1.first(), edge1.second()) > std::max(edge2.first(), edge2.second()))
            {
                return (false);
            }

            //	The only way we should end up down here is if the two edges are the same

            return (false);
        }
    };

    class TriangleByIndices
    {
       public:
        TriangleByIndices()
            : uid_(UNINITIALIZED_INDEX),
              a_(UNINITIALIZED_VERTEX_INDEX),
              b_(UNINITIALIZED_VERTEX_INDEX),
              c_(UNINITIALIZED_VERTEX_INDEX)
        {
        }

        TriangleByIndices(TriangleUID uid, VertexIndex a, VertexIndex b, VertexIndex c) : uid_(uid), a_(a), b_(b), c_(c)
        {
        }

        TriangleByIndices(TriangleUID uid, VertexIndex a, VertexIndex b, VertexIndex c,
                          TriangleBooleanAlgData bool_alg_data)
            : uid_(uid), a_(a), b_(b), c_(c), bool_alg_data_(bool_alg_data)
        {
        }

        TriangleByIndices(const TriangleByIndices& tri_by_indices, TriangleBooleanAlgData bool_alg_data)
            : uid_(tri_by_indices.uid_),
              a_(tri_by_indices.a_),
              b_(tri_by_indices.b_),
              c_(tri_by_indices.c_),
              bool_alg_data_(bool_alg_data)
        {
        }

        TriangleByIndices(const TriangleByIndices&) = default;
        TriangleByIndices(TriangleByIndices&&) = default;

        virtual ~TriangleByIndices() {}

        TriangleByIndices& operator=(const TriangleByIndices&) = default;
        TriangleByIndices& operator=(TriangleByIndices&&) = default;

        VertexIndex operator[](size_t index) const
        {
            assert(index < 3);
            return (reinterpret_cast<const VertexIndex*>(&a_))[index];
        }

        VertexIndex& operator[](size_t index)
        {
            assert(index < 3);
            return (reinterpret_cast<VertexIndex*>(&a_))[index];
        }

        [[nodiscard]] TriangleUID uid() const { return uid_; }

        [[nodiscard]] VertexIndex a() const { return a_; }

        [[nodiscard]] VertexIndex& a() { return a_; }

        [[nodiscard]] VertexIndex b() const { return b_; }

        [[nodiscard]] VertexIndex& b() { return b_; }

        [[nodiscard]] VertexIndex c() const { return c_; }

        [[nodiscard]] VertexIndex& c() { return c_; }

        [[nodiscard]] EdgeByIndices edge(TriangleEdgeId edge_id) const
        {
            switch (edge_id)
            {
                case TriangleEdgeId::AB:
                    return EdgeByIndices(a_, b_);
                    break;

                case TriangleEdgeId::BC:
                    return EdgeByIndices(b_, c_);
                    break;
            }

            return EdgeByIndices(c_, a_);
        }

        [[nodiscard]] TriangleBooleanAlgData bool_alg_data() const { return bool_alg_data_.value(); }

        void set_bool_alg_data(uint32_t new_value) { bool_alg_data_ = new_value; }

        void flip() { std::swap(a_, b_); }

        void offset_indices(size_t offset_value)
        {
            a_ += VertexIndex(offset_value);
            b_ += VertexIndex(offset_value);
            c_ += VertexIndex(offset_value);
        }

       protected:
        TriangleUID uid_;

        VertexIndex a_;
        VertexIndex b_;
        VertexIndex c_;

        std::optional<TriangleBooleanAlgData>
            bool_alg_data_;  // internal use by algorithm - value must be copied when the triangle is subdivided
    };

    inline std::ostream& operator<<(std::ostream& out, const TriangleByIndices& tri_by_indices)
    {
        out << "(" << tri_by_indices.a() << ", " << tri_by_indices.b() << ", " << tri_by_indices.c() << ")";
        return out;
    }

    class TriangleByIndicesVector : public std::vector<TriangleByIndices>
    {
       public:
        TriangleByIndicesVector() = default;

        TriangleByIndicesVector(const TriangleByIndicesVector&) = default;
        TriangleByIndicesVector(TriangleByIndicesVector&&) = default;

        TriangleByIndicesVector(size_t count) : std::vector<TriangleByIndices>(count) {}

        const TriangleByIndices& operator[](size_t) const = delete;
        TriangleByIndices& operator[](size_t) = delete;

        const TriangleByIndices& operator[](TriangleByIndicesIndex idx) const
        {
            return std::vector<TriangleByIndices>::operator[](static_cast<size_t>(idx));
        }

        TriangleByIndices& operator[](TriangleByIndicesIndex idx)
        {
            return std::vector<TriangleByIndices>::operator[](static_cast<size_t>(idx));
        }
    };

    class TriangleByIndicesIndexVector : public std::vector<TriangleByIndicesIndex>
    {
       public:
        TriangleByIndicesIndexVector() = default;
        TriangleByIndicesIndexVector(TriangleByIndicesIndexVector&&) = default;

        TriangleByIndicesIndexVector(size_t count, const TriangleByIndicesIndex& default_value)
            : std::vector<TriangleByIndicesIndex>(count, default_value)
        {
        }

        TriangleByIndicesIndex& operator[](TriangleByIndicesIndex index)
        {
            return std::vector<TriangleByIndicesIndex>::operator[](static_cast<size_t>(index));
        }
    };

    class TriangleByIndicesIndexSet : public std::set<TriangleByIndicesIndex>
    {
       public:
        TriangleByIndicesIndexSet() = default;
        TriangleByIndicesIndexSet(TriangleByIndicesIndexSet&&) = default;

        TriangleByIndicesIndexSet(const TriangleByIndicesIndexSet& set_to_copy)
            : std::set<TriangleByIndicesIndex>(set_to_copy)
        {
        }

        template <class InputIterator>
        TriangleByIndicesIndexSet(InputIterator first, InputIterator last)
            : std::set<TriangleByIndicesIndex>(first, last)
        {
        }

        TriangleByIndicesIndexSet(const TriangleByIndicesIndexSet& set_to_copy,
                                  const TriangleByIndicesIndexSet& set_to_merge)
            : std::set<TriangleByIndicesIndex>(set_to_copy)
        {
            merge(set_to_merge);
        }

        TriangleByIndicesIndexSet& operator=(const TriangleByIndicesIndexSet&) = default;

        bool intersects(const TriangleByIndicesIndexSet& set_to_check) const
        {
            for (auto index : set_to_check)
            {
                if (contains(index))
                {
                    return true;
                }
            }

            return false;
        }

        bool is_proper_subset(const TriangleByIndicesIndexSet& set_to_check) const
        {
            for (auto index : set_to_check)
            {
                if (!contains(index))
                {
                    return false;
                }
            }

            return true;
        }

        TriangleByIndicesIndexSet& merge(const TriangleByIndicesIndexSet& set_to_add)
        {
            for (auto index : set_to_add)
            {
                emplace(index);
            }

            return *this;
        }

        TriangleByIndicesIndexSet difference(const TriangleByIndicesIndexSet& set_to_diff)
        {
            TriangleByIndicesIndexSet difference;

            std::set_difference(begin(), end(), set_to_diff.begin(), set_to_diff.end(),
                                std::inserter(difference, difference.begin()));

            return difference;
        }
    };

    using TriangleByIndicesSet = std::set<TriangleByIndices>;

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

        [[nodiscard]] const Vertex3D& vertexA() const { return a_; }

        [[nodiscard]] const Vertex3D& vertexB() const { return b_; }

        [[nodiscard]] const Vertex3D& vertexC() const { return c_; }

        [[nodiscard]] Vector3D edgeAB_from_origin() const { return (a_ - b_); }

        [[nodiscard]] Vector3D edgeAC_from_origin() const { return (a_ - c_); }

        [[nodiscard]] Vector3D edgeBC_from_origin() const { return (b_ - c_); }

        [[nodiscard]] EdgeByVertices edge(TriangleEdgeId edge_id)
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

        [[nodiscard]] BBox3D bounding_box() const
        {
            return BBox3D(
                Vector3D(std::min(a_.x(), std::min(b_.x(), c_.x())), std::min(a_.y(), std::min(b_.y(), c_.y())),
                         std::min(a_.z(), std::min(b_.z(), c_.z()))),
                Vector3D(std::max(a_.x(), std::max(b_.x(), c_.x())), std::max(a_.y(), std::max(b_.y(), c_.y())),
                         std::max(a_.z(), std::max(b_.z(), c_.z()))));
        }

        [[nodiscard]] MinAndMaxEdgeLengths min_and_max_edge_lengths() const
        {
            double edge_ab_len = edgeAB_from_origin().len_squared();
            double edge_ac_len = edgeAC_from_origin().len_squared();
            double edge_bc_len = edgeBC_from_origin().len_squared();

            double min_squared = std::min(edge_ab_len, std::min(edge_ac_len, edge_bc_len));
            double max_squared = std::max(edge_ab_len, std::max(edge_ac_len, edge_bc_len));

            return MinAndMaxEdgeLengths::from_squares(min_squared, max_squared);
        }

        [[nodiscard]] double max_magnitude_vertex() const
        {
            return std::max(c_.abs().max(), std::max(a_.abs().max(), b_.abs().max()));
        }

        [[nodiscard]] Vector3D normal() const { return (b_ - a_).cross(c_ - a_); }

        [[nodiscard]] double tri_area() const { return ((b_ - a_).cross(c_ - a_).len()); }

        [[nodiscard]] double tri_area_squared() const { return ((b_ - a_).cross(c_ - a_).len_squared()); }

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

    template <typename IndexType>
    class BooleanVector
    {
       public:
        BooleanVector(std::size_t size) : vector_(size, false) {}
        BooleanVector(std::size_t size, bool initial_value) : vector_(size, initial_value) {}

        void resize(IndexType new_size) { vector_.resize(size_t(new_size)); }

        [[nodiscard]] size_t size() const { return vector_.size(); }

        unsigned char& operator[](IndexType index)
        {
            unsigned char& return_value = vector_[static_cast<size_t>(index)];
            return return_value;
        }

        bool operator[](IndexType index) const { return vector_[static_cast<size_t>(index)]; }

       private:
        std::vector<unsigned char> vector_;
    };

}  // namespace Cork::Primitives

namespace Cork
{
    using IndexType = Primitives::IndexType;
    using VertexIndex = Primitives::VertexIndex;
    using TriangleByIndicesIndex = Primitives::TriangleByIndicesIndex;

    using TriangleVertexId = Primitives::TriangleVertexId;
    using TriangleEdgeId = Primitives::TriangleEdgeId;

    using TriangleUID = Primitives::TriangleUID;

    using Vector2D = Primitives::Vector2D;
    using Vertex2D = Primitives::Vertex2D;

    using Vector3D = Primitives::Vector3D;
    using Vertex3D = Primitives::Vertex3D;

    using Vector3DVector = Primitives::Vector3DVector;

    using Ray3D = Primitives::Ray3D;
    using Ray3DWithInverseDirection = Primitives::Ray3DWithInverseDirection;
    using BBox3D = Primitives::BBox3D;
    using MinAndMaxEdgeLengths = Primitives::MinAndMaxEdgeLengths;

    using EdgeByIndices = Primitives::EdgeByIndices;

    using TriangleByVertices = Primitives::TriangleByVertices;
    using TriangleByIndices = Primitives::TriangleByIndices;

    using Vertex3DVector = Primitives::Vertex3DVector;
    using VertexIndexVector = Primitives::VertexIndexVector;
    using TriangleByIndicesVector = Primitives::TriangleByIndicesVector;
    using EdgeByIndicesVector = Primitives::EdgeByIndicesVector;

    using TriangleByIndicesSet = Primitives::TriangleByIndicesSet;
    using TriangleByIndicesIndexVector = Primitives::TriangleByIndicesIndexVector;
    using TriangleByIndicesIndexSet = Primitives::TriangleByIndicesIndexSet;

    template <typename IndexType>
    using BooleanVector = Primitives::BooleanVector<IndexType>;
}  // namespace Cork
