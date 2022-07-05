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
#include "intersection/glue_and_generic_types.hpp"
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
            m_isctVertexTypePool.reserve(200000);
            m_isctEdgeTypePool.reserve(300000);
            m_genericTriTypePool.reserve(100000);
            m_isctVertexPointerPool.reserve(200000);
            m_isctEdgePointerPool.reserve(100000);
            m_genericTriPointerPool.reserve(100000);
        }

        virtual ~IntersectionWorkspace() noexcept {};

        void reset()
        {
            m_gluePointMarkerPool.clear();
            m_isctVertexTypePool.clear();
            m_isctEdgeTypePool.clear();
            m_genericTriTypePool.clear();
            m_isctVertexPointerPool.clear();
            m_isctEdgePointerPool.clear();
            m_genericTriPointerPool.clear();

            m_AABVHWorkspace.reset();
        }

        operator GluePointMarkerList::PoolType &() { return (m_gluePointMarkerPool); }

        operator IsctVertTypeList::PoolType &() { return (m_isctVertexTypePool); }

        operator IsctEdgeTypeList::PoolType &() { return (m_isctEdgeTypePool); }

        operator GenericTriTypeList::PoolType &() { return (m_genericTriTypePool); }

        operator IntersectionVertexPointerList::PoolType &() { return (m_isctVertexPointerPool); }

        operator IntersectionEdgePointerList::PoolType &() { return (m_isctEdgePointerPool); }

        operator GenericTriPointerList::PoolType &() { return (m_genericTriPointerPool); }

        operator AABVH::Workspace &() { return (m_AABVHWorkspace); }

       private:
        GluePointMarkerList::PoolType m_gluePointMarkerPool;
        IsctVertTypeList::PoolType m_isctVertexTypePool;  //	Covers both the intersection and original vertex
                                                          // types
        IsctEdgeTypeList::PoolType m_isctEdgeTypePool;
        GenericTriTypeList::PoolType m_genericTriTypePool;

        IntersectionVertexPointerList::PoolType m_isctVertexPointerPool;
        IntersectionEdgePointerList::PoolType m_isctEdgePointerPool;
        GenericTriPointerList::PoolType m_genericTriPointerPool;

        AABVH::Workspace m_AABVHWorkspace;
    };

    typedef SEFUtility::CachingFactory<IntersectionWorkspace> IntersectionWorkspaceFactory;
}

