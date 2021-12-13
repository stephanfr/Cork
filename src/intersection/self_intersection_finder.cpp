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

    using SelfIntersectingEdge = Statistics::SelfIntersectingEdge;

    SelfIntersectionFinder::SelfIntersectionFinder(const Meshes::TriangleByIndicesVectorTopoCache& topo_cache)
        : m_intersection_workspace(std::move(SEFUtility::CachingFactory<IntersectionWorkspace>::GetInstance())),
          topo_cache_(topo_cache)
    {
        CreateBoundingVolumeHierarchy();
    }

    void SelfIntersectionFinder::reset() { m_intersection_workspace.get()->reset(); }

    const std::vector<SelfIntersectingEdge> SelfIntersectionFinder::CheckSelfIntersection()
    {
        Empty3d::ExactArithmeticContext localArithmeticContext;
        std::vector<SelfIntersectingEdge> self_intersections;
        std::vector<std::set<TopoTri*>> si_triangle_sets;

        for (const TopoTri& triangle_intersected : topo_cache_.triangles())
        {
            Meshes::TopoEdgeReferenceVector edges(std::move(m_edgeBVH->EdgesIntersectingTriangle(
                triangle_intersected, AABVH::IntersectionType::SELF_INTERSECTION)));

            for (const TopoEdge& edge : edges)
            {
                if (triangle_intersected.intersectsEdge(edge, topo_cache_.quantizer(), localArithmeticContext))
                {
                    //  There will be multiple hits for the same general self intersection.  Combining
                    //      self intersections here will reduce work later as we try to resolve them.
                    //
                    //  We deduplicate by looking for prior self intersections whose vertices share a triangle.
                    //      This seems to work reasonably well.

                    std::set<TopoTri*> topo_tris_with_se_vertex;

                    for (auto triangle_sharing_edge : edge.vert_0().triangles())
                    {
                        topo_tris_with_se_vertex.insert(triangle_sharing_edge);
                    }

                    for (auto triangle_sharing_edge : edge.vert_1().triangles())
                    {
                        topo_tris_with_se_vertex.insert(triangle_sharing_edge);
                    }

                    bool duplicate = false;

                    for (const auto& current_triangle_set : si_triangle_sets)
                    {
                        for (auto topo_tri : topo_tris_with_se_vertex)
                        {
                            if (current_triangle_set.contains(topo_tri))
                            {
                                duplicate = true;
                                break;
                            }
                        }
                    }

                    if (duplicate)
                    {
                        continue;
                    }

                    //  This self intersection is unique - record it.

                    self_intersections.emplace_back(edge.source_triangle_id(), edge.edge_index(),
                                                    triangle_intersected.source_triangle_id());

                    si_triangle_sets.emplace_back(topo_tris_with_se_vertex);
                }
            }
        }

        return self_intersections;
    }
}  // namespace Cork::Intersection