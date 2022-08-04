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
    class NonManifoldEdge
    {
       public:
        NonManifoldEdge() = delete;

        NonManifoldEdge(TriangleByIndicesIndex triangle_id, TriangleEdgeId edge_id)
            : triangle_id_(triangle_id), edge_id_(edge_id)
        {
        }

        NonManifoldEdge(const NonManifoldEdge&) = default;
        NonManifoldEdge(NonManifoldEdge&&) = default;

        ~NonManifoldEdge() = default;

        NonManifoldEdge& operator=(const NonManifoldEdge&) = default;
        NonManifoldEdge& operator=(NonManifoldEdge&&) = default;

        [[nodiscard]] TriangleByIndicesIndex triangle_id() const { return triangle_id_; }
        [[nodiscard]] TriangleEdgeId edge_id() const { return edge_id_; }

       private:
        TriangleByIndicesIndex triangle_id_;
        TriangleEdgeId edge_id_;
    };
}  // namespace Cork::Primitives

namespace Cork
{
    using NonManifoldEdge = Primitives::NonManifoldEdge;
}  // namespace Cork
