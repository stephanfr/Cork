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

#include "primitives.hpp"

namespace Cork::Primitives
{
    class SelfIntersectingEdge
    {
       public:
        SelfIntersectingEdge() = delete;

        SelfIntersectingEdge(const SelfIntersectingEdge&) = default;
        SelfIntersectingEdge(SelfIntersectingEdge&&) = default;

        SelfIntersectingEdge(TriangleByIndicesIndex edge_triangle_id, TriangleEdgeId edge_index,
                             TriangleByIndicesIndex triangle_instersected_id)
            : edge_triangle_id_(edge_triangle_id),
              edge_index_(edge_index),
              triangle_instersected_id_(triangle_instersected_id)
        {
        }

        ~SelfIntersectingEdge() = default;

        SelfIntersectingEdge& operator=(const SelfIntersectingEdge&) = default;
        SelfIntersectingEdge& operator=(SelfIntersectingEdge&&) = delete;

        [[nodiscard]] TriangleByIndicesIndex edge_triangle_id() const { return edge_triangle_id_; }
        [[nodiscard]] TriangleEdgeId edge_index() const { return edge_index_; }
        [[nodiscard]] TriangleByIndicesIndex triangle_instersected_id() const { return triangle_instersected_id_; }

       private:
        TriangleByIndicesIndex edge_triangle_id_;
        TriangleEdgeId edge_index_;
        TriangleByIndicesIndex triangle_instersected_id_;
    };
}  // namespace Cork::Primitives

namespace Cork
{
    using SelfIntersectingEdge = Primitives::SelfIntersectingEdge;
}  // namespace Cork
