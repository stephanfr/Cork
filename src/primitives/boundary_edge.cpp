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

//#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>

#include "math/normal_projector.hpp"
#include "math/util_3D.hpp"

namespace Cork::Primitives
{
    /*
        template <typename T>
        Vector3D BoundaryEdge::best_fit_normal( uint32_t    num_verts,
                                                T           itr_begin,
                                                T           itr_end)
        {
            //  We will build an Nx3 matrix (A) and an N dimensional vector (B) and solve for x in Ax=B.  x will
            //      be a best-fit surface normal.

            Vertex3D surf_centroid = centroid( itr_begin, itr_end );

            std::cout << surf_centroid << std::endl;

            Eigen::MatrixXd A(num_verts, 3);
            Eigen::VectorXd B(num_verts);

            Eigen::MatrixXd C(num_verts, 3);

            int k = 0;
            for (auto itr = itr_begin; itr != itr_end; itr++ )
            {
                Vector3D translated_point = *itr - surf_centroid;

        std::cout << *itr << "    " << translated_point << std::endl;

                A(k, 0) = translated_point.x();
                A(k, 1) = translated_point.y();
                A(k, 2) = 1;

                B(k) = translated_point.z();

                C(k, 0) = translated_point.x();
                C(k, 1) = translated_point.y();
                C(k, 2) = translated_point.z();

                k++;
            }

            std::cout << A << std::endl;
            std::cout << B << std::endl;

            Eigen::VectorXd normal = (A.transpose() * A).ldlt().solve(A.transpose() * B);

            Vector3D    result( normal(0), normal(1), normal(2) );
            result.normalize();

            Eigen::BDCSVD<Eigen::MatrixXd> svd(C, Eigen::ComputeFullV);

            std::cout << svd.singularValues() << std::endl;
            std::cout << svd.matrixV() << std::endl;

            return result;
        }
    */

    //    template Vector3D Cork::Math::Utility3D::best_fit_normal<BoundaryEdge::circularIterator>( uint32_t num_verts,
    //    const BoundaryEdge::circularIterator, const BoundaryEdge::circularIterator);

    Vertex3D BoundaryEdge::centroid() const { return Math::Utility3D::centroid(vertices_.begin(), vertices_.end()); }

    Vector3D BoundaryEdge::best_fit_normal() const
    {
        return Math::Utility3D::best_fit_normal<Cork::Primitives::Vector3DVector::const_iterator>(
            vertices_.size(), vertices_.cbegin(), vertices_.cend());
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

    Vector3DVector BoundaryEdge::get_normal_vectors(uint32_t adjacent_points)
    {
        Vector3DVector normals;

        for (int32_t i = 0; i < vertices_.size(); i++)
        {
            auto normal = Math::Utility3D::best_fit_normal((adjacent_points * 2 + 1), ci_location(i - adjacent_points),
                                                           ci_location(i + adjacent_points + 1));
            normals.emplace_back(normal);
        }

        return normals;
    }

}  // namespace Cork::Primitives
