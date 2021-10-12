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

#include "math/Primitives.h"

namespace Cork::Meshes
{
    using VertexIndex = Math::VertexIndex;
    using EdgeByIndices = Math::EdgeByIndices;

    class Hole
    {
       public:
        Hole(const Hole&) = default;

        Hole& operator=(const Hole&) = default;

        const std::deque<VertexIndex>& vertices() const { return vertices_; }

       private:
        std::deque<VertexIndex> vertices_;

        Hole(const std::deque<VertexIndex>& vertices) : vertices_(vertices) {}

        friend class HoleBuilder;
    };

    class HoleBuilder
    {
       public:
        HoleBuilder(const EdgeByIndices& starting_edge)
        {
            vertices_.push_back(starting_edge.first());
            vertices_.push_back(starting_edge.second());
        }

        bool empty() const { return vertices_.empty(); }

        const std::deque<VertexIndex>& vertices() const { return vertices_; }

        bool add_edge(const EdgeByIndices& next_edge)
        {
            assert(!is_closed());

            if (is_closed())
            {
                return false;
            }

            bool added_edge = false;

            if (next_edge.contains_vertex(vertices_.front()))
            {
                if (next_edge.first() == vertices_.front())
                {
                    vertices_.push_front(next_edge.second());
                }
                else
                {
                    vertices_.push_front(next_edge.first());
                }

                added_edge = true;
            }
            else if (next_edge.contains_vertex(vertices_.back()))
            {
                if (next_edge.first() == vertices_.back())
                {
                    vertices_.push_back(next_edge.second());
                }
                else
                {
                    vertices_.push_back(next_edge.first());
                }

                added_edge = true;
            }

            return added_edge;
        }

        bool is_closed() { return vertices_.front() == vertices_.back(); }

        Hole as_hole()
        {
            assert(is_closed());
            Hole result(vertices_);
            result.vertices_.pop_back();
            return result;
        }

        void reset(const EdgeByIndices& starting_edge)
        {
            vertices_.clear();

            vertices_.push_back(starting_edge.first());
            vertices_.push_back(starting_edge.second());
        }

       private:
        std::deque<VertexIndex> vertices_;
    };

    inline std::ostream& operator<<(std::ostream& out, const Hole& hole)
    {
        bool add_comma = false;

        std::cout << "(";

        for (auto current_vertex : hole.vertices())
        {
            if (add_comma)
            {
                std::cout << ", ";
            }

            std::cout << current_vertex;

            add_comma = true;
        }

        std::cout << " )" << std::endl;

        return out;
    };
}  // namespace Cork::TriangleMesh
