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

#include "boundary_edge.hpp"

#include "math/normal_projector.hpp"
#include "math/plane.hpp"

namespace Cork::Primitives
{
    //    template Vector3D Cork::Math::Utility3D::best_fit_normal<BoundaryEdge::circularIterator>( uint32_t num_verts,
    //    const BoundaryEdge::circularIterator, const BoundaryEdge::circularIterator);

    Vertex3D BoundaryEdge::centroid() const { return Math::centroid(vertices_.begin(), vertices_.end()); }

    BestFitPlaneEquation BoundaryEdge::best_fit_plane() const
    {
        return BestFitPlaneEquation(vertices_.size(), vertices_.cbegin(), vertices_.cend());
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

    std::vector<double> BoundaryEdge::get_point_deviations(const PlaneEquation&    plane)
    {
        std::vector<double>      deviations;

        for ( auto current_vertex : vertices_ )
        {
            deviations.emplace_back( plane.distance( current_vertex ) );
        }

        return deviations;
    }

}  // namespace Cork::Primitives
