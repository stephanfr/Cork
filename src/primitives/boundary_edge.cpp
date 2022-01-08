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

#include "primitives/boundary_edge.hpp"

#include <Eigen/Dense>

#include "math/normal_projector.hpp"

namespace Cork::Primitives
{
    Vector3D BoundaryEdge::best_fit_normal() const
    {
        //  We will build an Nx3 matrix (A) and an N dimensional vector (B) and solve for x in Ax=B.  x will
        //      be a best-fit surface normal.

        Vertex3D surf_centroid = centroid();

        Eigen::MatrixXd A(vertices_.size(), 3);
        Eigen::VectorXd B(vertices_.size());

        int k = 0;
        for (auto current_vertex : vertices_)
        {
            Vector3D translated_point = current_vertex - surf_centroid;

            A(k, 0) = translated_point.x();
            A(k, 1) = translated_point.y();
            A(k, 2) = 1;

            B(k) = translated_point.z();

            k++;
        }

        Eigen::VectorXd normal = (A.transpose() * A).ldlt().solve(A.transpose() * B);

        return Vector3D(normal(0), normal(1), normal(2)).normalized();
    }

    TwoD::Polygon BoundaryEdge::project(const Vector3D projection_surface_normal,
                                        const Vertex3D normal_surface_origin) const
    {
        std::vector<Vertex3D> projected_vertices;

        for (auto current_vertex : vertices_)
        {
            Vertex3D projected_vertex =
                current_vertex -
                ((current_vertex - normal_surface_origin).dot(projection_surface_normal) * projection_surface_normal);

            projected_vertices.emplace_back(projected_vertex);
        }

        Math::NormalProjector normal_projector(projected_vertices[0], projected_vertices[1], projected_vertices[2]);

        std::vector<Vertex2D> projection_2D;

        for (auto current_vert : projected_vertices)
        {
            projection_2D.emplace_back(normal_projector.project(current_vert));
        }

        return TwoD::Polygon(std::move(projection_2D));
    }

}  // namespace Cork::Primitives
