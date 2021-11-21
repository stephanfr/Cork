// +-------------------------------------------------------------------------
// | self_intersection_finder.hpp
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

#include "edge_cache.hpp"
#include "mesh/topo_cache.hpp"

namespace Cork::Intersection
{
    class SelfIntersectionFinder
    {
       public:
        SelfIntersectionFinder(const Meshes::TriangleByIndicesVectorTopoCache&   topo_cache);

        virtual ~SelfIntersectionFinder() { reset(); }

        const std::vector<Statistics::SelfIntersectingEdge> CheckSelfIntersection();

//        std::set<TriangleByIndicesIndex> find_enclosing_triangles(
//            const std::set<TriangleByIndicesIndex>& triangles_patch);

       private:
        SEFUtility::CachingFactory<IntersectionWorkspace>::UniquePtr m_intersection_workspace;

        const Meshes::TriangleByIndicesVectorTopoCache& topo_cache_;

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
}