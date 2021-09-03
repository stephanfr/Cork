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
}  // namespace Cork::Math

//	The Ray and Bounding Box classes depend on the Vector3D and Vertex3D classes

#include "BoundingBox.h"
#include "Ray3D.h"

namespace Cork::Math
{
    using IndexType = size_t;

    //	Vertices and vectors share the same implementation of a numeric 3-tuple

    using Vertex3D = Vector3D;
    
    using Vector3DVector = std::vector<Vector3D>;
    using Vertex3DVector = std::vector<Vertex3D>;


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

    class EdgeBase
    {
       public:
        EdgeBase(IndexType a, IndexType b) : m_vertexA(std::min(a, b)), m_vertexB(std::max(a, b)) {}

        virtual ~EdgeBase() {}

        IndexType vertexA() const { return (m_vertexA); }

        IndexType vertexB() const { return (m_vertexB); }

        bool operator==(const EdgeBase& edgeToCompare) const
        {
            return ((vertexA() == edgeToCompare.vertexA()) && (vertexB() == edgeToCompare.vertexB()));
        }

       private:
        IndexType m_vertexA;
        IndexType m_vertexB;
    };

    class TriangleByVerticesBase
    {
       public:
        TriangleByVerticesBase(const Vertex3D& firstVertex, const Vertex3D& secondVertex, const Vertex3D& thirdVertex)
            : m_vertices({firstVertex, secondVertex, thirdVertex})
        {
        }

        const Vertex3D& operator[](unsigned int index) const { return (m_vertices[index]); }

        const Vertex3D& vertexA() const { return (m_vertices[0]); }

        const Vertex3D& vertexB() const { return (m_vertices[1]); }

        const Vertex3D& vertexC() const { return (m_vertices[2]); }

        Vector3D edgeAB() const { return (m_vertices[0] - m_vertices[1]); }

        Vector3D edgeAC() const { return (m_vertices[0] - m_vertices[2]); }

        Vector3D edgeBC() const { return (m_vertices[1] - m_vertices[2]); }

       private:
        std::array<Vertex3D, 3> m_vertices;
    };

}  // namespace Cork::Math
