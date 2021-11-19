// +-------------------------------------------------------------------------
// | EGraphCache.h
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

#include <boost/container/static_vector.hpp>

#include "util/caching_factory.hpp"
#include "util/construct_once_resizeable_vector.hpp"
#include "util/sparse_vector.hpp"

#include "primitives/primitives.hpp"

namespace Cork::Meshes
{
    //	The TIDs vector length never seems to go beyond 4 but I will double that just to be sure
    //		as static vectors will return junk if they pass beyond their limit.

    static constexpr int EGRAPH_ENTRY_TIDS_VEC_LENGTH = 8;

    using EGraphEntryTIDVector = boost::container::static_vector<IndexType, EGRAPH_ENTRY_TIDS_VEC_LENGTH>;

    class EGraphEntry : public SEFUtility::SparseVectorEntry
    {
       public:
        EGraphEntry(IndexType index) : SEFUtility::SparseVectorEntry(index), vertex_id_(index) {}

        IndexType vid() const { return (vertex_id_); }

        const EGraphEntryTIDVector& tids() const { return (triangle_ids_); }

        EGraphEntryTIDVector& tids() { return (triangle_ids_); }

        bool intersects() const { return (intersects_); }

        void set_intersects(bool newValue) { intersects_ = newValue; }

       private:
        IndexType vertex_id_;
        EGraphEntryTIDVector triangle_ids_;
        bool intersects_;
    };

    class EGraphCache
    {
       public:
        using EGraphSkeletonColumn = SEFUtility::SparseVector<EGraphEntry, 10>;
        using SkeletonColumnVector = SEFUtility::ConstructOnceResizeableVector<EGraphSkeletonColumn>;

        EGraphCache() : skeleton_column_vector_(SEFUtility::CachingFactory<SkeletonColumnVector>::GetInstance()), skeleton_(*skeleton_column_vector_) {}

        ~EGraphCache() {}

        void resize(size_t newSize) { skeleton_.resize(newSize); }

        const SkeletonColumnVector& columns() const { return (skeleton_); }

        SkeletonColumnVector& columns() { return (skeleton_); }

        EGraphSkeletonColumn& operator[](IndexType index) { return (skeleton_[index]); }

        const EGraphSkeletonColumn& operator[](IndexType index) const { return (skeleton_[index]); }

       private:

        //  The skeleton_ reference is here to make access to the re-used skeleton_column_vector_ cleaner.

        SEFUtility::CachingFactory<SkeletonColumnVector>::UniquePtr skeleton_column_vector_;
        SkeletonColumnVector& skeleton_;
    };

}  // namespace Cork::Meshes
