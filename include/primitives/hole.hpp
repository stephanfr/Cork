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

#include "primitives/primitives.hpp"

namespace Cork
{
    class Hole
    {
       public:
        Hole(const Hole&) = default;

        Hole& operator=(const Hole&) = default;

        const std::vector<VertexIndex>& vertices() const { return vertices_; }

       private:
        std::vector<VertexIndex> vertices_;

        Hole(const std::vector<VertexIndex>& vertices) : vertices_(vertices) {}

        friend class HoleBuilder;
    };

    class HoleBuilder
    {
       public :

        static std::vector<Hole>    extract_holes(const std::vector<EdgeByIndices>&    hole_edges )
        {
            //  Return immediately with an empty list if there are no edges

            if( hole_edges.empty() )
            {
                return std::vector<Hole>();
            }

            //  Start extracting holes

            std::vector<Hole> holes;

            std::vector<EdgeByIndices>    edges(hole_edges);

            HoleBuilder hole_builder(edges.back());

            edges.pop_back();

            bool fell_through = true;

            do
            {
                fell_through = true;

                for (size_t i = 0; i < edges.size(); i++)
                {
                    if (hole_builder.add_edge(edges[i]))
                    {
                        edges.erase(edges.begin() + i);

                        if (hole_builder.is_closed())
                        {
                            std::vector<Hole>       new_holes( hole_builder.get_holes() );
                            holes.insert(std::begin(holes), std::begin( new_holes ), std::end( new_holes ));

                            if (!edges.empty())
                            {
                                hole_builder.reset(edges.back());
                                edges.pop_back();
                            }
                        }

                        fell_through = false;
                        break;
                    }
                }
            } while (!fell_through);

            return holes;
        }

    private :

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

        std::vector<Hole> get_holes()
        {
            assert(is_closed());

            //  If the list is not a closed hole - return nothing as something has probably gone wrong.

            if( !is_closed() )
            {
                return std::vector<Hole>();
            }

            //  We are going to use a recursive extraction.  There may be a more efficient technique but
            //      hopefully we do not encounter a lot of holes in practice and hopefully they are neither large
            //      nor complex.  That said, even if they are - this approach of recursively pulling inner holes
            //      out of holes *should* work in general.

            std::vector<Hole> holes =
                extract_holes_recursively(Hole(std::vector(std::begin(vertices_), std::end(vertices_))));

            //  Return the holes we have

            return holes;
        }

        void reset(const EdgeByIndices& starting_edge)
        {
            vertices_.clear();

            vertices_.push_back(starting_edge.first());
            vertices_.push_back(starting_edge.second());
        }

        std::deque<VertexIndex> vertices_;

        std::vector<Hole> extract_holes_recursively(Hole hole)
        {
            std::vector<Hole> holes;

            //  If there is a repeated index, then we have an embedded hole

            while (hole.vertices_.size() > 1)
            {
                bool break_to_while = false;

                for (size_t i = 0; i < hole.vertices_.size() - 1  && !break_to_while; i++)
                {
                    for (size_t j = i + 1; j < hole.vertices_.size() && !break_to_while; j++)
                    {
                        if (hole.vertices_[i] == hole.vertices_[j])
                        {
                            //  We have an embedded hole

                            Hole inner_hole(std::vector(hole.vertices_.begin() + i, hole.vertices_.begin() + j));

                            std::vector<Hole> inner_holes = extract_holes_recursively(
                                Hole(std::vector(hole.vertices_.begin() + i, hole.vertices_.begin() + j)));

                            if (!inner_holes.empty())
                            {
                                holes.insert(std::end(holes), std::begin(inner_holes), std::end(inner_holes));
                            }

                            hole.vertices_.erase(hole.vertices_.begin() + i, hole.vertices_.begin() + j);
                            break_to_while = true;
                        }
                    }
                }

                //  If we are down here and have not been asked to break, then 

                if( !break_to_while )
                {
                    holes.emplace_back( hole );
                    break;
                }
            }

            return holes;
        }
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
}  // namespace Cork::Meshes
