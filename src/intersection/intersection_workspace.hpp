// +-------------------------------------------------------------------------
// | intersection_workspace_base.hpp
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

#include "util/caching_factory.hpp"
#include "glue_and_generic_types.hpp"
#include "accel/aabvh.hpp"
#include "mesh/topo_cache.hpp"

namespace Cork::Intersection
{
    class IntersectionWorkspace 
    {
       public:
        IntersectionWorkspace()
        {
            m_gluePointMarkerPool.reserve(100000);
            m_genericVertexTypePool.reserve(200000);
            m_genericEdgeTypePool.reserve(300000);
            m_genericTriTypePool.reserve(100000);
            m_isctVertexPointerPool.reserve(200000);
            m_isctEdgePointerPool.reserve(100000);
            m_genericTriPointerPool.reserve(100000);
        }

        virtual ~IntersectionWorkspace() noexcept {};

        void reset()
        {
            m_gluePointMarkerPool.clear();
            m_genericVertexTypePool.clear();
            m_genericEdgeTypePool.clear();
            m_genericTriTypePool.clear();
            m_isctVertexPointerPool.clear();
            m_isctEdgePointerPool.clear();
            m_genericTriPointerPool.clear();

            m_AABVHWorkspace.reset();
        }

        GluePointMarkerList::PoolType& getGluePointMarkerListPool() { return m_gluePointMarkerPool; }

        GenericVertTypeList::PoolType& getGenericVertexListPool() { return m_genericVertexTypePool; }

        GenericEdgeTypeList::PoolType& getGenericEdgeListPool() { return m_genericEdgeTypePool; }

        GenericTriTypeList::PoolType& getGenericTriTypeListPool() { return m_genericTriTypePool; }

        IntersectionVertexPointerList::PoolType& getIsctVertexPointerListPool() { return m_isctVertexPointerPool; }

        IntersectionEdgePointerList::PoolType& getIsctEdgePointerListPool() { return m_isctEdgePointerPool; }

        GenericTriPointerList::PoolType& getGenericTriPointerListPool() { return m_genericTriPointerPool; }

        AABVH::Workspace& getAABVHWorkspace() { return m_AABVHWorkspace; }

       private:
        GluePointMarkerList::PoolType m_gluePointMarkerPool;
        GenericVertTypeList::PoolType m_genericVertexTypePool;  // Covers both the intersection and original vertex types
        GenericEdgeTypeList::PoolType m_genericEdgeTypePool;    // Covers intersection, original and split edges
        GenericTriTypeList::PoolType m_genericTriTypePool;

        IntersectionVertexPointerList::PoolType m_isctVertexPointerPool;
        IntersectionEdgePointerList::PoolType m_isctEdgePointerPool;
        GenericTriPointerList::PoolType m_genericTriPointerPool;

        AABVH::Workspace m_AABVHWorkspace;
    };

    typedef SEFUtility::CachingFactory<IntersectionWorkspace> IntersectionWorkspaceFactory;
}

