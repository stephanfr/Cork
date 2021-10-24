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
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <map>
#include <vector>

#include "CorkDefs.h"
#include "type_safe/integer.hpp"

//
//	Setup the numeric precision and the vector implementation for the build.
//

#include "math/vector_2D_template.hpp"
#include "math/vector_3D_template.hpp"

namespace Cork::Math
{
    constexpr double DOUBLE_TWO = 2.0;
    constexpr uint32_t UNINTIALIZED_INDEX = -1;

    using Vector2D = Vector2DTemplate<NUMERIC_PRECISION>;
    using Vector3D = Vector3DTemplate<NUMERIC_PRECISION>;

    using IndexType = uint32_t;
    using IndexVector = std::vector<IndexType>;

    //	Vertices and vectors share the same implementation of a numeric 3-tuple

    using Vertex3D = Vector3D;

    using Vector3DVector = std::vector<Vector3D>;

    using VertexIndex = type_safe::integer<uint32_t>;
    using TriangleByIndicesIndex = type_safe::integer<uint32_t>;


    class Vertex3DVector : public std::vector<Vertex3D>
    {
       public:
        const Vertex3D& operator[](size_t) const = delete;
        Vertex3D& operator[](size_t) = delete;

        const Vertex3D& operator[](VertexIndex idx) const { return data()[VertexIndex::integer_type(idx)]; }
        Vertex3D& operator[](VertexIndex idx) { return data()[VertexIndex::integer_type(idx)]; }
    };

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

    //	The Ray and Bounding Box classes depend on the Vector3D and Vertex3D classes

    class Ray3D
    {
       public:
        Ray3D() {}

        Ray3D(const Vector3D& point, const Vector3D& dir) : m_origin(point), m_direction(dir) {}

        Ray3D(const Ray3D& cp) : m_origin(cp.m_origin), m_direction(cp.m_direction) {}

        const Vector3D& origin() const { return (m_origin); }

        const Vector3D& direction() const { return (m_direction); }

       private:
        Vector3D m_origin;
        Vector3D m_direction;
    };

    class Ray3DWithInverseDirection : public Ray3D
    {
       public:
        Ray3DWithInverseDirection() {}

        Ray3DWithInverseDirection(const Vector3D& point, const Vector3D& dir)
            : Ray3D(point, dir),
              m_inverseDirection(1.0 / dir.x(), 1.0 / dir.y(), 1.0 / dir.z()),
              m_signs({(m_inverseDirection.x() < 0), (m_inverseDirection.y() < 0), (m_inverseDirection.z() < 0)})
        {
        }

        Ray3DWithInverseDirection(const Ray3D& cp)
            : Ray3D(cp),
              m_inverseDirection(1.0 / cp.direction().x(), 1.0 / cp.direction().y(), 1.0 / cp.direction().z()),
              m_signs({(m_inverseDirection.x() < 0), (m_inverseDirection.y() < 0), (m_inverseDirection.z() < 0)})
        {
        }

        const Vector3D& inverseDirection() const { return (m_inverseDirection); }

        const std::array<int, 3>& signs() const { return (m_signs); }

       private:
        Vector3D m_inverseDirection;

        std::array<int, 3> m_signs;
    };

    inline std::ostream& operator<<(std::ostream& out, const Ray3D& ray)
    {
        return out << '[' << ray.origin() << ';' << ray.direction() << ']';
    }

    // **************************************************************************
    // *  BBox3 stores 3-dimensional axis aligned bounding boxes
    // **************************************************************************

    static inline constexpr __m256d cnstexpr_mm256_set1_pd(double value)
    {
        return (__m256d){value, value, value, value};
    };

#ifdef __AVX_AVAILABLE__
    //    constexpr  __m256d AVXtwo = _mm256_load_pd(&DOUBLE_TWO);
    static constexpr __m256d AVXtwo = cnstexpr_mm256_set1_pd(DOUBLE_TWO);
#endif

    class BBox3D final
    {
       public:
        BBox3D() : m_minp(FLT_MAX, FLT_MAX, FLT_MAX), m_maxp(-FLT_MAX, -FLT_MAX, -FLT_MAX) {}

        BBox3D(const Vector3D& minpp, const Vector3D& maxpp) : m_minp(minpp), m_maxp(maxpp) {}

        BBox3D(const BBox3D& bb) : m_minp(bb.m_minp), m_maxp(bb.m_maxp) {}

        const Vector3D& minima() const { return (m_minp); }

        const Vector3D& maxima() const { return (m_maxp); }

        Vector3D center() const
        {
#ifdef __AVX_AVAILABLE__
            Vector3D result;

            _mm256_store_pd((double*)&result.ymm_,
                            _mm256_add_pd(m_minp, _mm256_div_pd(_mm256_sub_pd(m_maxp, m_minp), AVXtwo)));

            return (result);
#else
            return (m_minp + ((m_maxp - m_minp) / (NUMERIC_PRECISION)2.0));
#endif
        }

        bool isEmpty() const { return ((m_maxp[0] < m_minp[0]) || (m_maxp[1] < m_minp[1]) || (m_maxp[2] < m_minp[2])); }

        bool isIn(const Vector3D& pointToTest) const
        {
            return ((m_minp[0] <= pointToTest[0]) && (pointToTest[0] <= m_maxp[0]) && (m_minp[1] <= pointToTest[1]) &&
                    (pointToTest[1] <= m_maxp[1]) && (m_minp[2] <= pointToTest[2]) && (pointToTest[2] <= m_maxp[2]));
        }

#ifdef __AVX_AVAILABLE__

        //	SSE2 implementation

        bool intersects(const BBox3D& rhs) const
        {
            return (_mm256_movemask_pd(_mm256_and_pd(_mm256_cmp_pd(m_minp, rhs.m_maxp, _CMP_LE_OQ),
                                                     _mm256_cmp_pd(m_maxp, rhs.m_minp, _CMP_GE_OQ))) == 0x0F);
        }

        inline bool doesNotIntersect(const BBox3D& rhs) const
        {
            return (_mm256_movemask_pd(_mm256_and_pd(_mm256_cmp_pd(m_minp, rhs.m_maxp, _CMP_LE_OQ),
                                                     _mm256_cmp_pd(m_maxp, rhs.m_minp, _CMP_GE_OQ))) != 0x0F);
        }

#else

        inline bool intersects(const BBox3D& rhs) const
        {
            return ((m_minp[0] <= rhs.m_maxp[0]) && (m_maxp[0] >= rhs.m_minp[0]) && (m_minp[1] <= rhs.m_maxp[1]) &&
                    (m_maxp[1] >= rhs.m_minp[1]) && (m_minp[2] <= rhs.m_maxp[2]) && (m_maxp[2] >= rhs.m_minp[2]));
        }

        inline bool doesNotIntersect(const BBox3D& rhs) const
        {
            return ((m_minp[0] >= rhs.m_maxp[0]) || (m_maxp[0] <= rhs.m_minp[0]) || (m_minp[1] >= rhs.m_maxp[1]) ||
                    (m_maxp[1] <= rhs.m_minp[1]) || (m_minp[2] >= rhs.m_maxp[2]) || (m_maxp[2] <= rhs.m_minp[2]));
        }

#endif

        void scale(const Vector3D& scaling)
        {
            m_minp = Vector3D(m_minp.x() * scaling.x(), m_minp.y() * scaling.y(), m_minp.z() * scaling.z());
            m_maxp = Vector3D(m_maxp.x() * scaling.x(), m_maxp.y() * scaling.y(), m_maxp.z() * scaling.z());
        }

        void convex(const BBox3D& rhs)
        {
#ifdef __AVX_AVAILABLE__
            m_minp = _mm256_min_pd(m_minp, rhs.m_minp);
            m_maxp = _mm256_max_pd(m_maxp, rhs.m_maxp);
#else
            m_minp = min(m_minp, rhs.m_minp);
            m_maxp = max(m_maxp, rhs.m_maxp);
#endif
        }

        void convex(const BBox3D& rhs, BBox3D& result) const
        {
#ifdef __AVX_AVAILABLE__
            _mm256_store_pd((double*)&result.m_minp, _mm256_min_pd(m_minp, rhs.m_minp));
            _mm256_store_pd((double*)&result.m_maxp, _mm256_max_pd(m_maxp, rhs.m_maxp));
#else
            result.m_minp = min(m_minp, rhs.m_minp);
            result.m_maxp = max(m_maxp, rhs.m_maxp);
#endif
        }

        BBox3D intersection(const BBox3D& rhs) const
        {
            return (BBox3D(m_minp.max(rhs.m_minp), m_maxp.min(rhs.m_maxp)));
        }

        Vector3D dim() const { return (m_maxp - m_minp); }

        NUMERIC_PRECISION surfaceArea() const
        {
            Vector3D d = dim();
            return (2 * (d[1] * d[2] + d[0] * d[2] + d[0] * d[1]));
        }

        bool intersects(Ray3DWithInverseDirection& ray) const
        {
            NUMERIC_PRECISION txmin, txmax, tymin, tymax, tzmin, tzmax, txymin, txymax;

            const std::array<Vector3D, 2>& bounds = reinterpret_cast<const std::array<Vector3D, 2>&>(m_minp);

            txmin = (bounds[ray.signs()[0]].x() - ray.origin().x()) * ray.inverseDirection().x();
            tymax = (bounds[1 - ray.signs()[1]].y() - ray.origin().y()) * ray.inverseDirection().y();

            if (txmin > tymax)
            {
                return false;
            }

            txmax = (bounds[1 - ray.signs()[0]].x() - ray.origin().x()) * ray.inverseDirection().x();
            tymin = (bounds[ray.signs()[1]].y() - ray.origin().y()) * ray.inverseDirection().y();

            if (tymin > txmax)
            {
                return false;
            }

            txymin = std::max(tymin, txmin);
            txymax = std::min(txmax, tymax);

            tzmin = (bounds[ray.signs()[2]].z() - ray.origin().z()) * ray.inverseDirection().z();

            if (tzmin > txymax)
            {
                return (false);
            }

            tzmax = (bounds[1 - ray.signs()[2]].z() - ray.origin().z()) * ray.inverseDirection().z();

            return (txymin <= tzmax);
        }

       private:
        Vector3D m_minp;
        Vector3D m_maxp;
    };

    inline std::ostream& operator<<(std::ostream& out, const BBox3D& bb)
    {
        return out << "[min" << bb.minima() << ";max" << bb.maxima() << ']';
    }

    static const BBox3D gEmptyBoundingBox(Math::Vector3D(0.0, 0.0, 0.0), Vector3D(0.0, 0.0, 0.0));

    class MinAndMaxEdgeLengths
    {
       public:
        static MinAndMaxEdgeLengths from_squares(double min_squared, double max_squared)
        {
            return MinAndMaxEdgeLengths(min_squared, max_squared);
        }

        MinAndMaxEdgeLengths() : min_squared_(NUMERIC_PRECISION_MAX), max_squared_(NUMERIC_PRECISION_MIN) {}

        MinAndMaxEdgeLengths(const MinAndMaxEdgeLengths& obj_to_copy)
            : min_squared_(obj_to_copy.min_squared_), max_squared_(obj_to_copy.max_squared_)
        {
        }

        ~MinAndMaxEdgeLengths() = default;

        double min() const { return sqrt(min_squared_); }

        double max() const { return sqrt(max_squared_); }

        void update(const MinAndMaxEdgeLengths& second)
        {
            min_squared_ = std::min(min_squared_, second.min_squared_);
            max_squared_ = std::max(max_squared_, second.max_squared_);
        }

        void update_with_squares(double second_min_squared, double second_max_squared)
        {
            min_squared_ = std::min(min_squared_, second_min_squared);
            max_squared_ = std::max(max_squared_, second_max_squared);
        }

       private:
        double min_squared_;
        double max_squared_;

        MinAndMaxEdgeLengths(double min_squared, double max_squared)
            : min_squared_(min_squared), max_squared_(max_squared)
        {
        }
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
    //	Finally, define some base classes for key mesh classes: Triangle by vertex indices, Edges and Triangle by
    // vertex points
    //

    class TriangleByIndices
    {
       public:
        TriangleByIndices() : m_a(UNINTIALIZED_INDEX), m_b(UNINTIALIZED_INDEX), m_c(UNINTIALIZED_INDEX) {}

        TriangleByIndices(VertexIndex a, VertexIndex b, VertexIndex c) : m_a(a), m_b(b), m_c(c) {}

        virtual ~TriangleByIndices() {}

        const VertexIndex operator[](size_t index) const
        {
            assert(index < 3);
            return (reinterpret_cast<const VertexIndex*>(&m_a))[index];
        }

        VertexIndex& operator[](size_t index)
        {
            assert(index < 3);
            return (reinterpret_cast<VertexIndex*>(&m_a))[index];
        }

        const VertexIndex a() const { return (m_a); }

        VertexIndex& a() { return (m_a); }

        const VertexIndex b() const { return (m_b); }

        VertexIndex& b() { return (m_b); }

        const VertexIndex c() const { return (m_c); }

        VertexIndex& c() { return (m_c); }

       protected:
        VertexIndex m_a;
        VertexIndex m_b;
        VertexIndex m_c;
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

        const TriangleByIndices& operator[](TriangleByIndicesIndex idx) const
        {
            return data()[TriangleByIndicesIndex::integer_type(idx)];
        }
        TriangleByIndices& operator[](TriangleByIndicesIndex idx)
        {
            return data()[TriangleByIndicesIndex::integer_type(idx)];
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

    //
    //  Setup some types
    //

    using EdgeByIndicesVector = std::vector<Math::EdgeByIndices>;

}  // namespace Cork::Math
