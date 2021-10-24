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
        IntersectionInfo(Math::TriangleByIndicesIndex edge_triangle_id, Math::TriangleEdgeId edge_index,
                         Math::TriangleByIndicesIndex triangle_instersected_id,
                         const std::vector<Math::TriangleByIndicesIndex>& triangles_sharing_edge)
            : edge_triangle_id_(edge_triangle_id),
              edge_index_(edge_index),
              triangle_instersected_id_(triangle_instersected_id),
              triangles_sharing_edge_(triangles_sharing_edge)
        {
        }

        Math::TriangleByIndicesIndex edge_triangle_id_;
        Math::TriangleEdgeId edge_index_;
        Math::TriangleByIndicesIndex triangle_instersected_id_;
        std::vector<Math::TriangleByIndicesIndex> triangles_sharing_edge_;
    };

}  // namespace Cork::Intersection
