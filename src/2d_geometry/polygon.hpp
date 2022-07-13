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

#include <vector>

#include "primitives/primitives.hpp"

namespace Cork::TwoD
{
    class Edge2D
    {
       public:
        Edge2D() = delete;

        Edge2D(const Vertex2D& v0, const Vertex2D& v1, uint32_t reference_index)
            : v0_(v0), v1_(v1), reference_index_(reference_index)
        {
        }

        Edge2D(const Edge2D& edge_to_copy)
            : v0_(edge_to_copy.v0_), v1_(edge_to_copy.v1_), reference_index_(edge_to_copy.reference_index_)
        {
        }

        Edge2D(Edge2D&& edge_to_move)
            : v0_(std::move(edge_to_move.v0_)),
              v1_(std::move(edge_to_move.v1_)),
              reference_index_(edge_to_move.reference_index_)
        {
        }

        Vertex2D v0() const { return v0_; }
        Vertex2D v1() const { return v1_; }

        uint32_t reference_index() const { return reference_index_; }

        bool intersects(const Edge2D& edge_to_check) const;

       private:
        Vertex2D v0_;
        Vertex2D v1_;

        uint32_t reference_index_;
    };

    class IntersectingEdges2D
    {
       public:
        IntersectingEdges2D(const Edge2D& edge1, const Edge2D& edge2) : edge1_(edge1), edge2_(edge2) {}

        Edge2D edge1_;
        Edge2D edge2_;
    };

    class Polygon
    {
       public:
        Polygon(const std::vector<Vertex2D>& vertices) : centroid_(0, 0)
        {
            edges_.reserve(vertices.size() + 4);

            for (uint32_t i = 0; i < vertices.size() - 1; i++)
            {
                edges_.emplace_back(vertices[i], vertices[i + 1], i);
                centroid_ += vertices[i];
            }

            edges_.emplace_back(vertices.back(), vertices.front(), vertices.size() - 1);

            centroid_ += vertices.back();

            centroid_ /= vertices.size();
        }

        Polygon(Polygon&& polygon_to_move)
            : edges_(std::move(polygon_to_move.edges_)), centroid_(polygon_to_move.centroid_)
        {
        }

        const std::vector<Edge2D>& edges() const { return edges_; }

        Vertex2D centroid() const { return centroid_; }

        Polygon translate(const Vertex2D& offset)
        {
            std::vector<Vertex2D> translated_vertices;

            translated_vertices.reserve(edges_.size() + 4);

            for (uint32_t i = 0; i < edges_.size(); i++)
            {
                translated_vertices.emplace_back(edges_[i].v0() + offset);
            }

            return Polygon(translated_vertices);
        }

        std::vector<IntersectingEdges2D> self_intersections() const;

       private:
        std::vector<Edge2D> edges_;

        Vertex2D centroid_;
    };
}  // namespace Cork::TwoD