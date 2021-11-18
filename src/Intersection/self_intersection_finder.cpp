// +-------------------------------------------------------------------------
// | self_intersection_finder.cpp
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

#include "intersection/self_intersection_finder.hpp"

namespace Cork::Intersection
{
    using TopoEdge = Meshes::TopoEdge;
    using TopoTri = Meshes::TopoTri;
    using TopoEdgePointerVector = Meshes::TopoEdgePointerVector;

    using IntersectionInfo = Statistics::IntersectionInfo;

    SelfIntersectionFinder::SelfIntersectionFinder(const Meshes::TriangleByIndicesVectorTopoCache&   topo_cache)
        : m_intersection_workspace(std::move(SEFUtility::CachingFactory<IntersectionWorkspace>::GetInstance())),
          topo_cache_(topo_cache)
    {
        CreateBoundingVolumeHierarchy();
    }

    void SelfIntersectionFinder::reset() { m_intersection_workspace.get()->reset(); }

    const std::vector<IntersectionInfo> SelfIntersectionFinder::CheckSelfIntersection()
    {
        Empty3d::ExactArithmeticContext localArithmeticContext;
        std::vector<IntersectionInfo> self_intersections;

        for (const TopoTri& t : topo_cache_.triangles())
        {
            Meshes::TopoEdgeReferenceVector edges(
                std::move(m_edgeBVH->EdgesIntersectingTriangle(t, AABVH::IntersectionType::SELF_INTERSECTION)));

            for (const TopoEdge& edge : edges)
            {
                if (t.intersectsEdge(edge, topo_cache_.quantizer(), localArithmeticContext))
                {
                    std::set<TopoTri*> topo_tris_with_se_vertex;

                    for (auto triangle_sharing_edge : edge.vert_0().triangles())
                    {
                        topo_tris_with_se_vertex.insert(triangle_sharing_edge);
                    }

                    for (auto triangle_sharing_edge : edge.vert_1().triangles())
                    {
                        topo_tris_with_se_vertex.insert(triangle_sharing_edge);
                    }

                    std::set<TriangleByIndicesIndex> triangles_with_se_vertex;
                    std::array<std::set<TriangleByIndicesIndex>, 2> neighboring_triangles;

                    for (auto triangle_with_se_vertex : topo_tris_with_se_vertex)
                    {
                        triangles_with_se_vertex.insert(triangle_with_se_vertex->source_triangle_id());
                    }

                    //  Check to see if we have to merge this self intersection with an existing self-intersection

                    bool merged = false;

                    for (auto& current_triangle : triangles_with_se_vertex)
                    {
                        for (auto current_se : self_intersections)
                        {
                            if (current_se.triangles_including_se_vertex().find(current_triangle) !=
                                current_se.triangles_including_se_vertex().end())
                            {
                                current_se.merge( Statistics::SelfIntersectingEdge( edge.source_triangle_id(), edge.edge_index(),
                                                               t.source_triangle_id() ), triangles_with_se_vertex );

                                merged = true;
                                break;
                            }
                        }

                        if (merged)
                        {
                            break;
                        }
                    }

                    if (!merged)
                    {
                        std::vector<Statistics::SelfIntersectingEdge>       edges;

                        edges.emplace_back( edge.source_triangle_id(), edge.edge_index(), t.source_triangle_id() );

                        self_intersections.emplace_back(Intersection::IntersectionInfo( std::move(edges), std::move(triangles_with_se_vertex)));
                    }
                }
            }
        }

        return self_intersections;
    }

    std::set<TriangleByIndicesIndex> SelfIntersectionFinder::find_enclosing_triangles(
        const std::set<TriangleByIndicesIndex>& triangles_patch)
    {
        std::map<TriangleByIndicesIndex, const TopoTri*> topo_tris_by_index;

        for (auto& next_topo_tri : topo_cache_.triangles())
        {
            topo_tris_by_index.insert(std::make_pair(next_topo_tri.source_triangle_id(), &next_topo_tri));
        }

        std::set<const TopoTri*> topo_tris_in_patch;

        for (auto& next_triangle : triangles_patch)
        {
            topo_tris_in_patch.emplace(topo_tris_by_index[next_triangle]);
        }

        std::set<TriangleByIndicesIndex> enclosing_triangles;

        for (auto next_topo_tri : topo_tris_in_patch)
        {
            for (auto touching_edge : next_topo_tri->edges())
            {
                for (auto triangle_touching : touching_edge->triangles())
                {
                    if (!triangles_patch.contains(triangle_touching->source_triangle_id()))
                    {
                        enclosing_triangles.emplace(triangle_touching->source_triangle_id());
                    }
                }
            }
        }

        return enclosing_triangles;
    }
}  // namespace Cork::Intersection