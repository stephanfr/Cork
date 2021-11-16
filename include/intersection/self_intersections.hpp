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

#include "math/quantization.hpp"
#include "primitives/primitives.hpp"

namespace Cork::Intersection
{
    class SelfIntersectingEdge
    {
       public:
        SelfIntersectingEdge() = delete;

        SelfIntersectingEdge(Primitives::TriangleByIndicesIndex edge_triangle_id, Primitives::TriangleEdgeId edge_index,
                             Primitives::TriangleByIndicesIndex triangle_instersected_id)
            : edge_triangle_id_(edge_triangle_id),
              edge_index_(edge_index),
              triangle_instersected_id_(triangle_instersected_id)
        {
        }

        Primitives::TriangleByIndicesIndex edge_triangle_id_;
        Primitives::TriangleEdgeId edge_index_;
        Primitives::TriangleByIndicesIndex triangle_instersected_id_;
    };

    class IntersectionInfo
    {
       public:
        IntersectionInfo() = delete;

        IntersectionInfo(std::vector<SelfIntersectingEdge>&& edges,
                         std::set<Primitives::TriangleByIndicesIndex>&& triangles_including_se_vertex)
            : edges_(edges), triangles_including_se_vertex_(triangles_including_se_vertex)
        {
        }

        IntersectionInfo(const IntersectionInfo&) = default;

        ~IntersectionInfo() = default;

        const std::vector<SelfIntersectingEdge>& edges() const { return edges_; }

        const std::set<Primitives::TriangleByIndicesIndex>& triangles_including_se_vertex() const
        {
            return triangles_including_se_vertex_;
        }

       private:
        std::vector<SelfIntersectingEdge> edges_;
        std::set<Primitives::TriangleByIndicesIndex> triangles_including_se_vertex_;

        friend class SelfIntersectionFinderImpl;
    };

    class SelfIntersectionFinder
    {
       public:
        static std::unique_ptr<SelfIntersectionFinder> GetFinder(Primitives::TriangleByIndicesVector& triangles,
                                                                 Primitives::Vertex3DVector& vertices,
                                                                 uint32_t num_edges, const Math::Quantizer& quantizer);

        virtual ~SelfIntersectionFinder() {}

        virtual const std::vector<IntersectionInfo> CheckSelfIntersection() = 0;

        virtual std::set<Primitives::TriangleByIndicesIndex> find_enclosing_triangles(
            const std::set<Primitives::TriangleByIndicesIndex>& triangles_patch) = 0;
    };

}  // namespace Cork::Intersection
