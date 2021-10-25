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

#include "primitives/primitives.hpp"

namespace Cork::Intersection
{
    struct IntersectionInfo
    {
        IntersectionInfo(Primitives::TriangleByIndicesIndex edge_triangle_id, Primitives::TriangleEdgeId edge_index,
                         Primitives::TriangleByIndicesIndex triangle_instersected_id,
                         const std::vector<Primitives::TriangleByIndicesIndex>& triangles_sharing_edge)
            : edge_triangle_id_(edge_triangle_id),
              edge_index_(edge_index),
              triangle_instersected_id_(triangle_instersected_id),
              triangles_sharing_edge_(triangles_sharing_edge)
        {
        }

        Primitives::TriangleByIndicesIndex edge_triangle_id_;
        Primitives::TriangleEdgeId edge_index_;
        Primitives::TriangleByIndicesIndex triangle_instersected_id_;
        std::vector<Primitives::TriangleByIndicesIndex> triangles_sharing_edge_;
    };

}  // namespace Cork::Intersection
