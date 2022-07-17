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

#include <cstddef>
#include <iterator>

#include "2d_geometry/polygon.hpp"
#include "primitives.hpp"
#include "writeable_interfaces.hpp"
#include "math/plane.hpp"

namespace Cork::Meshes
{
    class BoundaryEdgeBuilder;
}

namespace Cork::Primitives
{
    class BoundaryEdge : public Writeable3DPolyline
    {
        struct circularIterator
        {
            using iterator_category = std::forward_iterator_tag;
            using value_type = Vertex3D;
            using const_pointer = const Vertex3D*;
            using const_reference = const Vertex3D&;

            circularIterator(const Vertex3DVector& vec, int32_t offset)
                : vec_(vec), itr_(vec_.begin() + fix_offset(vec_.size(), offset))
            {
            }

            const_reference operator*() const { return *itr_; }
            const_pointer operator->() const { return &(*itr_); }
            circularIterator& operator++()
            {
                itr_++;
                if (itr_ == vec_.end())
                {
                    itr_ = vec_.begin();
                };
                return *this;
            }
            circularIterator operator++(int)
            {
                circularIterator tmp = *this;
                ++(*this);
                return tmp;
            }
            friend bool operator==(const circularIterator& a, const circularIterator& b) { return a.itr_ == b.itr_; };
            friend bool operator!=(const circularIterator& a, const circularIterator& b) { return a.itr_ != b.itr_; };

           private:
            const Vertex3DVector& vec_;
            Vertex3DVector::const_iterator itr_;

            static uint32_t fix_offset(int32_t vec_size, int32_t offset)
            {
                if (offset < 0)
                {
                    std::cout << (offset % vec_size ) << "    " << vec_size + (offset % vec_size) << std::endl;
                    return vec_size + (offset % vec_size);
                }

                return offset % vec_size;
            };
        };

        circularIterator ci_location(int32_t offset) const { return circularIterator(this->vertices_, offset); }

       public:
        BoundaryEdge(Vertex3DVector&& vertices, VertexIndexVector&& vertex_indices)
            : vertices_(std::move(vertices)), vertex_indices_(std::move(vertex_indices))
        {
        }
        BoundaryEdge(const BoundaryEdge&) = default;

        BoundaryEdge& operator=(const BoundaryEdge&) = default;

        const Vertex3DVector& vertices() const { return vertices_; }
        const std::vector<VertexIndex>& vertex_indices() const { return vertex_indices_; }

        std::vector<BoundaryEdge> divide(size_t divisions)
        {
            std::vector<BoundaryEdge> new_boundaries;
            uint32_t points_per_division = vertex_indices_.size() / divisions;

            for (size_t i = 0; i < divisions; i++)
            {
                Vertex3DVector division_vertices;
                VertexIndexVector division_indicies;

                for (uint32_t j = i * points_per_division;
                     j < std::min(vertices_.size(), (i + 1) * points_per_division); j++)
                {
                    division_vertices.emplace_back(vertices_[VertexIndex(j)]);
                    division_indicies.emplace_back(vertex_indices_[j]);
                }

                new_boundaries.emplace_back(std::move(division_vertices), std::move(division_indicies));
            }

            return new_boundaries;
        }

        void append(Vertex3D vertex, VertexIndex index)
        {
            vertices_.emplace_back(vertex);
            vertex_indices_.emplace_back(index);
        }

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

        Vertex3D centroid() const;

        BestFitPlaneEquation best_fit_plane() const;

        TwoD::Polygon project(const Vector3D projection_surface_normal, const Vertex3D normal_surface_origin) const;

        std::vector<double> get_point_deviations(const PlaneEquationBase&    plane);

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
