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

#include "intersection/edge_cache.hpp"
#include "intersection/self_intersections.hpp"

namespace Cork::Intersection
{
    using TriangleByIndicesIndex = Primitives::TriangleByIndicesIndex;
    using VertexIndex = Primitives::VertexIndex;

    using TopoEdge = Meshes::TopoEdge;
    using TopoTri = Meshes::TopoTri;
    using TopoEdgePointerVector = Meshes::TopoEdgePointerVector;

    class SelfIntersectionFinderImpl : public SelfIntersectionFinder
    {
       public:
        SelfIntersectionFinderImpl(Primitives::TriangleByIndicesVector& triangles, Primitives::Vertex3DVector& vertices,
                                   uint32_t num_edges, const Math::Quantizer& quantizer);

        virtual ~SelfIntersectionFinderImpl() { reset(); }

        const std::vector<IntersectionInfo> CheckSelfIntersection() final;

       private:
        SEFUtility::CachingFactory<IntersectionWorkspace>::UniquePtr m_intersection_workspace;

        Math::Quantizer quantizer_;

        Meshes::TopoCacheBase<Primitives::TriangleByIndicesVector> topo_cache_;

        std::unique_ptr<AABVH::AxisAlignedBoundingVolumeHierarchy> m_edgeBVH;

        void reset();

        void CreateBoundingVolumeHierarchy()
        {
            std::unique_ptr<AABVH::GeomBlobVector> edge_geoms(new AABVH::GeomBlobVector());

            edge_geoms->reserve(topo_cache_.edges().size());

            for (auto& e : topo_cache_.edges())
            {
                edge_geoms->emplace_back(e);
            }

            m_edgeBVH.reset(new AABVH::AxisAlignedBoundingVolumeHierarchy(
                edge_geoms, *m_intersection_workspace, Cork::CorkService::get_default_control_block()));
        }
    };

    SelfIntersectionFinderImpl::SelfIntersectionFinderImpl(Primitives::TriangleByIndicesVector& triangles,
                                                           Primitives::Vertex3DVector& vertices, uint32_t num_edges,
                                                           const Math::Quantizer& quantizer)
        : m_intersection_workspace(std::move(SEFUtility::CachingFactory<IntersectionWorkspace>::GetInstance())),
          quantizer_(quantizer),
          topo_cache_(triangles, vertices, num_edges, quantizer)
    {
        CreateBoundingVolumeHierarchy();
    }

    void SelfIntersectionFinderImpl::reset() { m_intersection_workspace.get()->reset(); }

    const std::vector<IntersectionInfo> SelfIntersectionFinderImpl::CheckSelfIntersection()
    {
        Empty3d::ExactArithmeticContext localArithmeticContext;
        std::vector<IntersectionInfo> self_intersecting_edges;

        for (TopoTri& t : topo_cache_.triangles())
        {
            Meshes::TopoEdgeReferenceVector edges(
                std::move(m_edgeBVH->EdgesIntersectingTriangle(t, AABVH::IntersectionType::SELF_INTERSECTION)));

            for (const TopoEdge& edge : edges)
            {
                if (t.intersectsEdge(edge, quantizer_, localArithmeticContext))
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

                        for (auto touching_edge : triangle_with_se_vertex->edges())
                        {
                            for (auto triangle_touching : touching_edge->triangles())
                            {
                                neighboring_triangles[0].insert(triangle_touching->source_triangle_id());

                                for (auto next_edge_out : triangle_touching->edges())
                                {
                                    for (auto next_triangle_out : next_edge_out->triangles())
                                    {
                                        neighboring_triangles[1].insert(next_triangle_out->source_triangle_id());
                                    }
                                }
                            }
                        }
                    }

                    for (auto triangle_to_remove : triangles_with_se_vertex)
                    {
                        neighboring_triangles[0].erase(triangle_to_remove);
                        neighboring_triangles[1].erase(triangle_to_remove);
                    }

                    for (auto triangle_to_remove : neighboring_triangles[0])
                    {
                        neighboring_triangles[1].erase(triangle_to_remove);
                    }

                    self_intersecting_edges.emplace_back(Intersection::IntersectionInfo(
                        edge.source_triangle_id(), edge.edge_index(), t.source_triangle_id(),
                        std::move(triangles_with_se_vertex), std::move(neighboring_triangles)));
                }
            }
        }

        return self_intersecting_edges;
    }

    std::unique_ptr<SelfIntersectionFinder> SelfIntersectionFinder::GetFinder(
        Primitives::TriangleByIndicesVector& triangles, Primitives::Vertex3DVector& vertices, uint32_t num_edges,
        const Math::Quantizer& quantizer)
    {
        //        IntersectionWorkspaceFactory::UniquePtr workspace(IntersectionWorkspaceFactory::GetInstance());

        std::unique_ptr<SelfIntersectionFinder> finder(
            new SelfIntersectionFinderImpl(triangles, vertices, num_edges, quantizer));

        return finder;
    }
}  // namespace Cork::Intersection