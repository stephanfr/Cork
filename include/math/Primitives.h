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

//
//	Setup the numeric precision and the vector implementation for the build.
//

#include "Vector2DTemplate.h"
#include "Vector3DTemplate.h"

namespace Cork::Math
{
    using Vector2D = Vector2DTemplate<NUMERIC_PRECISION>;
    using Vector3D = Vector3DTemplate<NUMERIC_PRECISION>;

    using IndexType = size_t;
    using IndexVector = std::vector<IndexType>;

    //	Vertices and vectors share the same implementation of a numeric 3-tuple

    using Vertex3D = Vector3D;

    using Vector3DVector = std::vector<Vector3D>;
    using Vertex3DVector = std::vector<Vertex3D>;

    //  Define some enums to track vertex and edge assignments

    enum class TriangleVertexId : size_t
    {
        A = 0,
        B,
        C
    };

    enum class TriangleEdgeId : size_t
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

    constexpr double two = 2.0;

    static inline constexpr __m256d cnstexpr_mm256_set1_pd(double value)
    {
        return (__m256d){value, value, value, value};
    };

#ifdef __AVX_AVAILABLE__
    //    constexpr  __m256d AVXtwo = _mm256_load_pd(&two);
    static constexpr __m256d AVXtwo = cnstexpr_mm256_set1_pd(two);
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

    class TriangleByIndicesBase
    {
       public:
        TriangleByIndicesBase() : m_a(-1), m_b(-1), m_c(-1) {}

        TriangleByIndicesBase(IndexType a, IndexType b, IndexType c) : m_a(a), m_b(b), m_c(c) {}

        virtual ~TriangleByIndicesBase() {}

        const IndexType operator[](size_t index) const { return (m_indices[index]); }

        IndexType& operator[](size_t index) { return (m_indices[index]); }

        const IndexType a() const { return (m_a); }

        IndexType& a() { return (m_a); }

        const IndexType b() const { return (m_b); }

        IndexType& b() { return (m_b); }

        const IndexType c() const { return (m_c); }

        IndexType& c() { return (m_c); }

       protected:
        union
        {
            struct
            {
                IndexType m_a;
                IndexType m_b;
                IndexType m_c;
            };

            std::array<IndexType, 3> m_indices;
        };
    };

    inline std::ostream& operator<<(std::ostream& out, const TriangleByIndicesBase& tri_by_indices)
    {
        out << "(" << tri_by_indices.a() << ", " << tri_by_indices.b() << ", " << tri_by_indices.c() << ")";
        return out;
    }

    class EdgeByIndicesBase
    {
       public:
        //  Edges are always smaller vertex index first, larger index second

        EdgeByIndicesBase(IndexType first_vertex, IndexType second_vertex)
            : first_vertex_(std::min(first_vertex, second_vertex)),
              second_vertex_(std::max(first_vertex, second_vertex))
        {
        }

        virtual ~EdgeByIndicesBase() {}

        IndexType first() const { return (first_vertex_); }

        IndexType second() const { return (second_vertex_); }

        bool operator==(const EdgeByIndicesBase& edgeToCompare) const
        {
            return ((first() == edgeToCompare.first()) && (second() == edgeToCompare.second()));
        }

       private:
        IndexType first_vertex_;
        IndexType second_vertex_;
    };

    class EdgeByVerticesBase
    {
       public:
        EdgeByVerticesBase(const Vertex3D& first_vertex, const Vertex3D& second_vertex)
            : first_vertex_(first_vertex), second_vertex_(second_vertex)
        {
        }

        virtual ~EdgeByVerticesBase() {}

        const Vertex3D& first() const { return (first_vertex_); }

        const Vertex3D& second() const { return (second_vertex_); }

        bool operator==(const EdgeByVerticesBase& edgeToCompare) const
        {
            return ((first() == edgeToCompare.first()) && (second() == edgeToCompare.second()));
        }

       private:
        const Vertex3D first_vertex_;
        const Vertex3D second_vertex_;
    };

    inline std::ostream& operator<<(std::ostream& out, const EdgeByVerticesBase& edge_by_vertices)
    {
        out << "(" << edge_by_vertices.first() << ", " << edge_by_vertices.second() << ")";
        return out;
    }

    class TriangleByVerticesBase
    {
       public:
        TriangleByVerticesBase(const Vertex3D& firstVertex, const Vertex3D& secondVertex, const Vertex3D& thirdVertex)
            : m_vertices({firstVertex, secondVertex, thirdVertex})
        {
        }

        TriangleByVerticesBase(const TriangleByIndicesBase& triangle, const Vertex3DVector& vertices)
            : m_vertices({vertices.at(triangle.a()), vertices.at(triangle.b()), vertices.at(triangle.c())})
        {
        }

        const Vertex3D& operator[](unsigned int index) const { return (m_vertices[index]); }

        const Vertex3D& vertexA() const { return (m_vertices[0]); }

        const Vertex3D& vertexB() const { return (m_vertices[1]); }

        const Vertex3D& vertexC() const { return (m_vertices[2]); }

        Vector3D edgeAB_from_origin() const { return (m_vertices[0] - m_vertices[1]); }

        Vector3D edgeAC_from_origin() const { return (m_vertices[0] - m_vertices[2]); }

        Vector3D edgeBC_from_origin() const { return (m_vertices[1] - m_vertices[2]); }

        EdgeByVerticesBase edge(TriangleEdgeId edge_id)
        {
            switch (edge_id)
            {
                case TriangleEdgeId::AB:
                    return EdgeByVerticesBase(m_vertices[0], m_vertices[1]);
                    break;

                case TriangleEdgeId::BC:
                    return EdgeByVerticesBase(m_vertices[1], m_vertices[2]);
                    break;
            }
            
            return EdgeByVerticesBase(m_vertices[2], m_vertices[0]);
        }

        //        TriangleEdgeId

        BBox3D bounding_box() const
        {
            return BBox3D(Vector3D(std::min(m_vertices[0].x(), std::min(m_vertices[1].x(), m_vertices[2].x())),
                                   std::min(m_vertices[0].y(), std::min(m_vertices[1].y(), m_vertices[2].y())),
                                   std::min(m_vertices[0].z(), std::min(m_vertices[1].z(), m_vertices[2].z()))),
                          Vector3D(std::max(m_vertices[0].x(), std::max(m_vertices[1].x(), m_vertices[2].x())),
                                   std::max(m_vertices[0].y(), std::max(m_vertices[1].y(), m_vertices[2].y())),
                                   std::max(m_vertices[0].z(), std::max(m_vertices[1].z(), m_vertices[2].z()))));
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
            return std::max(m_vertices[2].abs().max(), std::max(m_vertices[0].abs().max(), m_vertices[1].abs().max()));
        }

       private:
        std::array<Vertex3D, 3> m_vertices;
    };

    inline std::ostream& operator<<(std::ostream& out, const TriangleByVerticesBase& tri_by_vertices)
    {
        out << "(" << tri_by_vertices.vertexA() << ", " << tri_by_vertices.vertexB() << ", "
            << tri_by_vertices.vertexC() << ")";
        return out;
    }

    //
    //  Setup some types
    //

    using EdgeByIndicesVector = std::vector<Math::EdgeByIndicesBase>;

}  // namespace Cork::Math
