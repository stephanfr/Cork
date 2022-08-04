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

#include <boost/container/small_vector.hpp>
#include <unordered_set>

#include "../constants.hpp"
#include "primitives.hpp"

namespace Cork::Meshes
{
    using EdgeAndIncidenceCountTriangleVector =
        boost::container::small_vector<std::pair<TriangleByIndicesIndex, TriangleEdgeId>,
                                       EDGE_AND_INCIDENCE_COUNT_NUM_TRIANGLEs_INITIAL_SIZE>;

    class EdgeAndIncidenceCount : public Primitives::EdgeByIndices
    {
       public:
        EdgeAndIncidenceCount() = delete;

        EdgeAndIncidenceCount(const EdgeAndIncidenceCount&) = delete;
        EdgeAndIncidenceCount(EdgeAndIncidenceCount&&) = delete;

        EdgeAndIncidenceCount(const Primitives::VertexIndex a, const Primitives::VertexIndex b)
            : Primitives::EdgeByIndices(a, b), num_incidences_(0)
        {
        }

        ~EdgeAndIncidenceCount() override = default;

        EdgeAndIncidenceCount& operator=(const EdgeAndIncidenceCount&) = delete;
        EdgeAndIncidenceCount& operator=(EdgeAndIncidenceCount&&) = delete;

        int add_incidence(TriangleByIndicesIndex tri_index, TriangleEdgeId edge_id)
        {
            triangles_.emplace_back(std::make_pair(tri_index, edge_id));
            return ++num_incidences_;
        }

        [[nodiscard]] int numIncidences() const { return (num_incidences_); }

        [[nodiscard]] const EdgeAndIncidenceCountTriangleVector& triangles() const { return triangles_; }

       private:
        int num_incidences_;
        EdgeAndIncidenceCountTriangleVector triangles_;
    };

    using EdgeIncidenceSet = std::unordered_set<EdgeAndIncidenceCount, EdgeAndIncidenceCount::HashFunction>;

}  //  namespace Cork::Meshes
