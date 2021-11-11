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

#include <set>

#include "primitives/primitives.hpp"
#include "math/quantization.hpp"

namespace Cork::Intersection
{
    class IntersectionInfo
    {
       public:
        IntersectionInfo() = delete;

        IntersectionInfo(Primitives::TriangleByIndicesIndex edge_triangle_id, Primitives::TriangleEdgeId edge_index,
                         Primitives::TriangleByIndicesIndex triangle_instersected_id,
                         std::set<Primitives::TriangleByIndicesIndex>&& triangles_including_se_vertex,
                         std::array<std::set<Primitives::TriangleByIndicesIndex>,2>&& neighboring_triangles )
            : edge_triangle_id_(edge_triangle_id),
              edge_index_(edge_index),
              triangle_instersected_id_(triangle_instersected_id),
              triangles_including_se_vertex_(triangles_including_se_vertex),
              neighboring_triangles_(neighboring_triangles)
        {
        }

        IntersectionInfo(const IntersectionInfo&) = default;

        ~IntersectionInfo() = default;

        Primitives::TriangleByIndicesIndex edge_triangle_id() const { return edge_triangle_id_; }

        Primitives::TriangleEdgeId edge_index() const { return edge_index_; }

        Primitives::TriangleByIndicesIndex triangle_instersected_id() const { return triangle_instersected_id_; }

        const std::set<Primitives::TriangleByIndicesIndex>& triangles_including_se_vertex() const
        {
            return triangles_including_se_vertex_;
        }

        const std::array<std::set<Primitives::TriangleByIndicesIndex>,2>& neighboring_triangles() const
        {
            return neighboring_triangles_;
        }

       private:
        Primitives::TriangleByIndicesIndex edge_triangle_id_;
        Primitives::TriangleEdgeId edge_index_;
        Primitives::TriangleByIndicesIndex triangle_instersected_id_;
        std::set<Primitives::TriangleByIndicesIndex> triangles_including_se_vertex_;
        std::array<std::set<Primitives::TriangleByIndicesIndex>,2> neighboring_triangles_;
    };


    class SelfIntersectionFinder
    {
       public:
        static std::unique_ptr<SelfIntersectionFinder> GetFinder(Primitives::TriangleByIndicesVector&  triangles, Primitives::Vertex3DVector& vertices, uint32_t num_edges, const Math::Quantizer& quantizer);

        virtual ~SelfIntersectionFinder() {}

        virtual const std::vector<IntersectionInfo> CheckSelfIntersection() = 0;
    };

}  // namespace Cork::Intersection
