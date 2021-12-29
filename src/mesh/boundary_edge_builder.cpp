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

#include "mesh/boundary_edge_builder.hpp"
#include "mesh/edge_incidence_counter.hpp"

namespace Cork::Meshes
{
    ExtractBoundariesResult BoundaryEdgeBuilder::extract_boundaries(const MeshBase& mesh, const TriangleByIndicesIndexSet& tris_in_region)
    {
        //  Return immediately with an empty list if there are no edges

        if (tris_in_region.empty())
        {
            return ExtractBoundariesResult::failure( ExtractBoundariesResultCodes::EMPTY_REGION, "Empty Region" );
        }

        //  Create the edge incidence map

        EdgeIncidenceCounter        edge_counts( mesh, tris_in_region );

        return extract_boundaries(mesh, edge_counts.edges_and_incidences());
    }

    ExtractBoundariesResult BoundaryEdgeBuilder::extract_boundaries(const MeshBase& mesh, const TriangleByIndicesIndexVector& tris_in_region)
    {
        //  Return immediately with an empty list if there are no edges

        if (tris_in_region.empty())
        {
            return ExtractBoundariesResult::failure( ExtractBoundariesResultCodes::EMPTY_REGION, "Empty Region" );
        }

        //  Create the edge incidence map

        EdgeIncidenceCounter        edge_counts( mesh, tris_in_region );

        return extract_boundaries(mesh, edge_counts.edges_and_incidences());
    }

    ExtractBoundariesResult BoundaryEdgeBuilder::extract_boundaries(const MeshBase& mesh, const EdgeIncidenceSet& region_edges)
    {
        //  Return immediately with an empty list if there are no edges

        if (region_edges.empty())
        {
            return ExtractBoundariesResult::failure( ExtractBoundariesResultCodes::EMPTY_REGION, "Empty Region" );
        }

        //  Start extracting boundaries

        std::vector<EdgeByIndices> edges;

        for (auto edge : region_edges)
        {
            if (edge.numIncidences() == 1)
            {
                edges.emplace_back(mesh.triangles()[edge.triangles()[0U].first].edge(edge.triangles()[0U].second));
            }
        }

        return extract_boundaries( edges );
    }

    ExtractBoundariesResult BoundaryEdgeBuilder::extract_boundaries( const EdgeByIndicesVector& region_edges)
    {
        //  Return immediately with an empty list if there are no edges

        if (region_edges.empty())
        {
            return ExtractBoundariesResult::failure( ExtractBoundariesResultCodes::EMPTY_REGION, "Empty Region" );
        }

        //  Start extracting boundaries

        auto boundaries = std::make_unique<std::vector<BoundaryEdge>>();

        std::vector<EdgeByIndices> edges( region_edges );

        vertices_.push_back(edges.back().first());
        vertices_.push_back(edges.back().second());

        edges.pop_back();

        bool fell_through = true;

        do
        {
            fell_through = true;

            for (size_t i = 0; i < edges.size(); i++)
            {
                if (add_edge(edges[i]))
                {
                    edges.erase(edges.begin() + i);

                    if (is_closed())
                    {
                        ExtractBoundariesResult new_boundaries = get_boundary_edges();

                        if( new_boundaries.failed() )
                        {
                            return new_boundaries;
                        }

                        boundaries->insert(std::begin(*boundaries), std::begin(*(new_boundaries.return_ptr())), std::end(*(new_boundaries.return_ptr())));

                        if (!edges.empty())
                        {
                            reset(edges.back());
                            edges.pop_back();
                        }
                    }

                    fell_through = false;
                    break;
                }
            }
        } while (!fell_through);

        if( !edges.empty() )
        {
            std::cout << "Edges remain after finding boundary" << std::endl;
        }

        if( boundaries->empty() )
        {
            return ExtractBoundariesResult::failure( ExtractBoundariesResultCodes::COULD_NOT_FIND_CLOSED_BOUNDARY, "Could not find closed boundary on exit" );
        }

        return ExtractBoundariesResult::success( std::unique_ptr<std::vector<BoundaryEdge>>( boundaries.release() ));
    }

    bool BoundaryEdgeBuilder::add_edge(const EdgeByIndices& next_edge)
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

    ExtractBoundariesResult BoundaryEdgeBuilder::get_boundary_edges()
    {
        //  If the list is not a closed boundary - return nothing as something has probably gone wrong.

        if (!is_closed())
        {
            return ExtractBoundariesResult::failure( ExtractBoundariesResultCodes::BOUNDARY_IS_NOT_CLOSED, "Boundary is not closed" );
        }

        //  We are going to use a recursive extraction.  There may be a more efficient technique but
        //      hopefully we do not encounter a lot of boundaries in practice and hopefully they are neither large
        //      nor complex.  That said, even if they are - this approach of recursively pulling inner boundaries
        //      out of boundaries *should* work in general.

        auto boundaries =
            extract_boundaries_recursively(BoundaryEdge(std::vector(std::begin(vertices_), std::end(vertices_))));

        //  Return the boundaries we have extracted

        return ExtractBoundariesResult::success( std::unique_ptr<std::vector<BoundaryEdge>>( boundaries.release() ));
    }

    std::unique_ptr<std::vector<BoundaryEdge>> BoundaryEdgeBuilder::extract_boundaries_recursively(BoundaryEdge boundary)
    {
        auto boundaries = std::make_unique<std::vector<BoundaryEdge>>();

        //  If there is a repeated index, then we have an embedded boundary

        while (boundary.vertex_indices_.size() > 1)
        {
            bool break_to_while = false;

            for (size_t i = 0; i < boundary.vertex_indices_.size() - 1 && !break_to_while; i++)
            {
                for (size_t j = i + 1; j < boundary.vertex_indices_.size() && !break_to_while; j++)
                {
                    if (boundary.vertex_indices_[i] == boundary.vertex_indices_[j])
                    {
                        //  We have an embedded boundary

                        std::unique_ptr<std::vector<BoundaryEdge>> inner_boundaries( extract_boundaries_recursively(
                            BoundaryEdge(std::vector(boundary.vertex_indices_.begin() + i, boundary.vertex_indices_.begin() + j))));

                        if (!inner_boundaries->empty())
                        {
                            boundaries->insert(std::end(*boundaries), std::begin(*inner_boundaries), std::end(*inner_boundaries));
                        }

                        boundary.vertex_indices_.erase(boundary.vertex_indices_.begin() + i, boundary.vertex_indices_.begin() + j);
                        break_to_while = true;
                    }
                }
            }

            //  If we are down here and have not been asked to break, then

            if (!break_to_while)
            {
                boundaries->emplace_back(boundary);
                break;
            }
        }

        return boundaries;
    }

    std::ostream& operator<<(std::ostream& out, const BoundaryEdge& boundary)
    {
        bool add_comma = false;

        std::cout << "(";

        for (auto current_vertex : boundary.vertex_indices())
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
