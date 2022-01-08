// Copyright (c) 2021 Stephan Friedl
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files (the "Software"), to deal in
// the Software without restriction, including without limitation the rights to
// use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
// the Software, and to permit persons to whom the Software is furnished to do so,
// subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
// IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
// CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#pragma once

#include "2d_geometry/polygon.hpp"
#include "primitives/primitives.hpp"

namespace Cork::Meshes
{
    class BoundaryEdgeBuilder;
}

namespace Cork::Primitives
{
    class BoundaryEdge
    {
       public:
        BoundaryEdge(Vertex3DVector&& vertices, VertexIndexVector&& vertex_indices)
            : vertices_(std::move(vertices)), vertex_indices_(std::move(vertex_indices))
        {
        }
        BoundaryEdge(const BoundaryEdge&) = default;

        BoundaryEdge& operator=(const BoundaryEdge&) = default;

        const Vertex3DVector& vertices() const { return vertices_; }
        const std::vector<VertexIndex>& vertex_indices() const { return vertex_indices_; }

        double length() const
        {
            double len = 0;

            for (VertexIndex i = 0u; i < vertex_indices_.size() - 1; i++)
            {
                Vector3D segment = vertices_[i] - vertices_[i + 1u];

                len += sqrt((segment.x() * segment.x()) + (segment.y() * segment.y()) + (segment.z() * segment.z()));
            }

            return len;
        }

        BBox3D bounding_box() const
        {
            BBox3D bounding_box;

            for (auto current_vertex : vertices_)
            {
                bounding_box.convex(current_vertex);
            }

            return bounding_box;
        }

        Vertex3D centroid() const
        {
            Vertex3D centroid;

            for (auto current_vertex : vertices_)
            {
                centroid += current_vertex;
            }

            centroid /= vertices_.size();

            return centroid;
        }

        Vector3D best_fit_normal() const;

        TwoD::Polygon project(const Vector3D projection_surface_normal, const Vertex3D normal_surface_origin) const;

       private:
        Vertex3DVector vertices_;

        std::vector<VertexIndex> vertex_indices_;
    };

    std::ostream& operator<<(std::ostream& out, const BoundaryEdge& boundary);

}  // namespace Cork::Primitives

namespace Cork
{
    using BoundaryEdge = Primitives::BoundaryEdge;
}  // namespace Cork
