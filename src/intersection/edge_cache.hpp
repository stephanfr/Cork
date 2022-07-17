// +-------------------------------------------------------------------------
// | edge_cache.hpp
// |
// | Author: Gilbert Bernstein
// +-------------------------------------------------------------------------
// | COPYRIGHT:
// |    Copyright Gilbert Bernstein 2013
// |    See the included COPYRIGHT file for further details.
// |
// |    This file is part of the Cork library.
// |
// |    Cork is free software: you can redistribute it and/or modify
// |    it under the terms of the GNU Lesser General Public License as
// |    published by the Free Software Foundation, either version 3 of
// |    the License, or (at your option) any later version.
// |
// |    Cork is distributed in the hope that it will be useful,
// |    but WITHOUT ANY WARRANTY; without even the implied warranty of
// |    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// |    GNU Lesser General Public License for more details.
// |
// |    You should have received a copy
// |    of the GNU Lesser General Public License
// |    along with Cork.  If not, see <http://www.gnu.org/licenses/>.
// +-------------------------------------------------------------------------

#pragma once

#include "intersection_problem_base.hpp"

namespace Cork::Intersection
{
    class EdgeCache
    {

        using TopoVert = Meshes::TopoVert;
        using TopoEdge = Meshes::TopoEdge;
        using TopoTri = Meshes::TopoTri;

       public:
        explicit EdgeCache(IntersectionProblemBase& intersection_problem)
            : intersection_problem_(intersection_problem), edges_(intersection_problem.owner_mesh().vertices().size())
        {
        }

        TopoEdge& operator()(TopoVert& v0, TopoVert& v1)
        {
            auto i = VertexIndex::integer_type(v0.index());
            auto j = VertexIndex::integer_type(v1.index());

            if (i > j)
            {
                std::swap(i, j);
            }

            size_t N = edges_[i].size();

            for (size_t k = 0; k < N; k++)
            {
                if (edges_[i][k].vertex_id() == VertexIndex(j))
                {
                    return edges_[i][k].edge().value();
                }
            }

            // if not existing, create it

            edges_[i].emplace_back(EdgeEntry(j));

            TopoEdge& new_edge = edges_[i][N].set_edge( *(intersection_problem_.topo_cache().new_edge(v0, v1)));

            return new_edge;
        }

        // k = 0, 1, or 2
        TopoEdge& getTriangleEdge(GenericTriType* tri, uint k, const TopoTri& big_tri)
        {
            GenericVertType* gv0 = tri->vertices()[(k + 1) % 3];
            GenericVertType* gv1 = tri->vertices()[(k + 2) % 3];
            TopoVert& v0 = gv0->concrete_vertex();
            TopoVert& v1 = gv1->concrete_vertex();

            // if neither of these are intersection points,
            // then this is a pre-existing edge...

            TopoEdge* edge = nullptr;

            if ((gv0->vertex_type() == GenericVertType::VertexType::ORIGINAL) &&
                (gv1->vertex_type() == GenericVertType::VertexType::ORIGINAL))
            {
                // search through edges of original triangle...
                for (uint c = 0; c < 3; c++)
                {
                    TopoVert* corner0 = big_tri.verts()[(c + 1) % 3];
                    TopoVert* corner1 = big_tri.verts()[(c + 2) % 3];

                    if (((corner0 == &v0) && (corner1 == &v1)) || ((corner0 == &v1) && (corner1 == &v0)))
                    {
                        edge = big_tri.edges()[c];
                    }
                }

                assert(edge != nullptr);
            }
            // otherwise, we need to check the cache to find this edge
            else
            {
                edge = &operator()(v0, v1);
            }

            return *edge;
        }

        std::optional<std::reference_wrapper<TopoEdge>> maybeEdge(const GenericEdgeType& edge)
        {
            size_t i = VertexIndex::integer_type(edge.ends()[0]->concrete_vertex().index());
            size_t j = VertexIndex::integer_type(edge.ends()[1]->concrete_vertex().index());

            if (i > j)
            {
                std::swap(i, j);
            }

            size_t N = edges_[i].size();

            for (size_t k = 0; k < N; k++)
            {
                if (edges_[i][k].vertex_id() == j)
                {
                    return edges_[i][k].edge().value();
                }
            }

            // if we can't find it
            return std::optional<std::reference_wrapper<TopoEdge>>();
        }

       private:
        class EdgeEntry
        {
            public :

            EdgeEntry() = delete;

            explicit EdgeEntry(IndexType vertex_id) : vertex_id_(vertex_id) {}

            [[nodiscard]] VertexIndex     vertex_id() const { return vertex_id_; }

            std::optional<std::reference_wrapper<TopoEdge>>&        edge() { return edge_; }
            TopoEdge&        set_edge( TopoEdge& edge ) { return edge_.emplace( edge ); }

            private :

            VertexIndex vertex_id_;
            std::optional<std::reference_wrapper<TopoEdge>> edge_;
        };

        typedef std::vector<std::vector<EdgeEntry>> VectorOfEdgeEntryVectors;

        IntersectionProblemBase& intersection_problem_;

        VectorOfEdgeEntryVectors edges_;
    };
}  // namespace Cork::Intersection
