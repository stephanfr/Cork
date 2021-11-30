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

#include <deque>
#include <vector>

#include "primitives/boundary_edge.hpp"

namespace Cork::Meshes
{
    class BoundaryEdgeBuilder
    {
       public:
        BoundaryEdgeBuilder() = default;

        std::vector<BoundaryEdge> extract_boundaries(const MeshBase& mesh, const TriangleByIndicesIndexSet& tris_in_region);
        std::vector<BoundaryEdge> extract_boundaries( const EdgeByIndicesVector&   edges );

       private:
        std::deque<VertexIndex> vertices_;

        void reset(const EdgeByIndices& starting_edge)
        {
            vertices_.clear();

            vertices_.push_back(starting_edge.first());
            vertices_.push_back(starting_edge.second());
        }

        bool empty() const { return vertices_.empty(); }

        const std::deque<VertexIndex>& vertices() const { return vertices_; }

        bool add_edge(const EdgeByIndices& next_edge);

        bool is_closed() { return vertices_.front() == vertices_.back(); }

        std::vector<BoundaryEdge> get_boundary_edges();

        std::vector<BoundaryEdge> extract_boundaries_recursively(BoundaryEdge boundary);
    };
}  // namespace Cork::Meshes
