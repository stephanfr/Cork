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

#include "CPPResult.hpp"
#include "primitives/boundary_edge.hpp"
#include "result_codes.hpp"

#include "primitives/edge_and_incidence_count.hpp"

namespace Cork::Meshes
{
    using ExtractBoundariesResult =
        SEFUtility::ResultWithReturnUniquePtr<ExtractBoundariesResultCodes, std::vector<BoundaryEdge>>;

    class MeshBase;

    class BoundaryEdgeBuilder
    {
       public:
        explicit BoundaryEdgeBuilder(const MeshBase& mesh) : mesh_(mesh) {};

        [[nodiscard]] ExtractBoundariesResult extract_boundaries(const TriangleByIndicesIndexSet& tris_in_region);
        [[nodiscard]] ExtractBoundariesResult extract_boundaries(const TriangleByIndicesIndexVector& tris_in_region);
        [[nodiscard]] ExtractBoundariesResult extract_boundaries(const EdgeIncidenceSet& region_edges);
        [[nodiscard]] ExtractBoundariesResult extract_boundaries(const EdgeByIndicesVector& region_edges);

       private:

        const MeshBase& mesh_;
        std::deque<VertexIndex> vertices_;

        void reset(const EdgeByIndices& starting_edge)
        {
            vertices_.clear();

            vertices_.push_back(starting_edge.first());
            vertices_.push_back(starting_edge.second());
        }

        [[nodiscard]] bool empty() const { return vertices_.empty(); }

        [[nodiscard]] const std::deque<VertexIndex>& vertices() const { return vertices_; }

        [[nodiscard]] bool add_edge(const EdgeByIndices& next_edge);

        [[nodiscard]] bool is_closed() { return vertices_.front() == vertices_.back(); }

        [[nodiscard]] ExtractBoundariesResult get_boundary_edges();

        [[nodiscard]] std::unique_ptr<std::vector<VertexIndexVector>> extract_boundaries_recursively(VertexIndexVector boundary);
    };
}  // namespace Cork::Meshes
