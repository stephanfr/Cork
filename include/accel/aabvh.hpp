// +-------------------------------------------------------------------------
// | aabvh.h
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
#include <boost/ptr_container/ptr_vector.hpp>
#include <cstddef>
#include <cstdlib>
#include <deque>
#include <vector>

#include "cork_defs.hpp"
#include "Xoshiro256Plus.h"
#include "mesh/TopoCache.h"
#include "oneapi/tbb/spin_mutex.h"
#include "oneapi/tbb/task_group.h"
#include "primitives/primitives.hpp"
#include "util/FastStack.h"
#include "util/ManagedIntrusiveList.h"

//
//	AABVH
//
//	Axis Aligned Bounding Box Volume Hierarchy
//
//	An AABVH is a balanced tree which subdivides the model at each branch point to provide quick searching
//		for intersections between edges and triangles.  The partitioning function 'quickselect' is the key to
//		how the goemetries are arranged in the tree.
//

namespace Cork::AABVH
{
    using Xoshiro256Plus = SEFUtility::RNG::Xoshiro256Plus<g_SIMD_Level>;

    //	Maximum leaf size - needs to be a power of 2 (i.e. 2, 4, 8, 16, 32, 64)

    const unsigned int LEAF_SIZE = 32;

    class alignas(SIMD_MEMORY_ALIGNMENT) GeomBlob
    {
       public:
        explicit GeomBlob(const Meshes::TopoEdge& idx) : m_id(idx)
        {
            const Primitives::Vector3D& p0 = (idx.verts()[0])->quantizedValue();
            const Primitives::Vector3D& p1 = (idx.verts()[1])->quantizedValue();

            m_bbox = Primitives::BBox3D(p0.min(p1), p0.max(p1));
        }

        const Meshes::TopoEdge& index() const { return (m_id); }

        const Primitives::BBox3D& boundingBox() const { return (m_bbox); }

       private:
        Primitives::BBox3D m_bbox;

        const Meshes::TopoEdge& m_id;
    };

    using GeomBlobVector = std::vector<GeomBlob>;

    //	A BlobIDList will never be larger than LEAF_SIZE so a static vector is OK

    using BlobIDList = boost::container::static_vector<Primitives::IndexType, LEAF_SIZE>;

    class alignas(SIMD_MEMORY_ALIGNMENT) AABVHNode  // : public IntrusiveListHook
    {
       public:
        AABVHNode() : m_left(nullptr), m_right(nullptr) {}

        AABVHNode(AABVHNode* left, AABVHNode* right) : m_left(left), m_right(right) {}

        inline bool isLeaf() const { return (m_left == nullptr); }

        AABVHNode* left() const { return (m_left); }

        AABVHNode* right() const { return (m_right); }

        const Primitives::BBox3D& boundingBox() const { return (m_bbox); }

        Primitives::BBox3D& boundingBox() { return (m_bbox); }

        const std::array<BlobIDList, 2>& blobIDLists() const { return (m_blobids); }

        void AddBlobID(Primitives::IndexType listIndex, Primitives::IndexType blobID)
        {
            m_blobids[listIndex].emplace_back(blobID);
        }

       private:
        AABVHNode* m_left;
        AABVHNode* m_right;

        Primitives::BBox3D m_bbox;
        std::array<BlobIDList, 2> m_blobids;
    };

    typedef std::vector<AABVHNode> AABVHNodeList;

    class AABVHNodeListCollection
    {
       public:
        AABVHNodeListCollection() : m_numCollectionsCheckedOut(0) { m_nodeCollections.reserve(16); }

        void reset()
        {
            for (auto& nodeVector : m_nodeCollections)
            {
                nodeVector.clear();
            }

            m_numCollectionsCheckedOut = 0;
        }

        AABVHNodeList& getPrimaryNodeList()
        {
            if (m_nodeCollections.size() == 0)
            {
                return (getNodeList(1024));
            }

            return (m_nodeCollections[0]);
        }

        AABVHNodeList& getNodeList(size_t reservation)
        {
            m_collectionsMutex.lock();

            if (m_nodeCollections.size() <= m_numCollectionsCheckedOut)
            {
                m_nodeCollections.push_back(new AABVHNodeList());
            }

            m_nodeCollections[m_numCollectionsCheckedOut].reserve(reservation);

            AABVHNodeList& listToReturn = m_nodeCollections[m_numCollectionsCheckedOut++];

            m_collectionsMutex.unlock();

            return (listToReturn);
        }

       private:
        boost::ptr_vector<AABVHNodeList> m_nodeCollections;
        oneapi::tbb::spin_mutex m_collectionsMutex;

        size_t m_numCollectionsCheckedOut;
    };

    class Workspace
    {
       public:
        Workspace() {}

        ~Workspace() {}

        void reset() { m_nodeListCollection.reset(); }

        AABVHNodeListCollection& getAABVHNodeListCollection() { return (m_nodeListCollection); }

       private:
        AABVHNodeListCollection m_nodeListCollection;
    };

    enum class IntersectionType
    {
        SELF_INTERSECTION = 0,
        BOOLEAN_INTERSECTION
    };

    class AxisAlignedBoundingVolumeHierarchy : public boost::noncopyable
    {
       public:
        AxisAlignedBoundingVolumeHierarchy(std::unique_ptr<GeomBlobVector>& geoms, Workspace& workspace,
                                           const SolverControlBlock& solver_control_block)
            : root_(nullptr),
              blobs_(std::move(geoms)),
              tmpids_(blobs_->size()),
              node_collections_(workspace.getAABVHNodeListCollection()),
              random_number_generator_(RANDOM_SEED),
              solver_control_block_(solver_control_block)
        {
            assert(blobs_->size() > 0);

            node_collections_.reset();

            for (uint k = 0; k < tmpids_.size(); k++)
            {
                tmpids_[k] = k;
            }

            representative_points_[0].reserve(blobs_->size());
            representative_points_[1].reserve(blobs_->size());
            representative_points_[2].reserve(blobs_->size());

            for (auto& blob : *blobs_)
            {
                Primitives::Vector3D repPoint(
                    blob.boundingBox().minima() +
                    ((blob.boundingBox().maxima() - blob.boundingBox().minima()) / (NUMERIC_PRECISION)2.0));

                representative_points_[0].emplace_back(repPoint.x());
                representative_points_[1].emplace_back(repPoint.y());
                representative_points_[2].emplace_back(repPoint.z());
            }

            root_ = ConstructTree(0, tmpids_.size(), 2);
        }

        ~AxisAlignedBoundingVolumeHierarchy() { node_collections_.reset(); }

        Meshes::TopoEdgeReferenceVector EdgesIntersectingTriangle(const Meshes::TopoTri& triangle,
                                                                IntersectionType intersectionType)
        {
            Meshes::TopoEdgeReferenceVector edges;

            //	Set the boolAlgData index for intersections between two bodies or for self-intersections.

            unsigned int blobIDListSelector =
                (intersectionType == IntersectionType::BOOLEAN_INTERSECTION ? triangle.boolAlgData() ^ 1
                                                                            : triangle.boolAlgData());

            //	Use a recursive search and save edges that intersect the triangle

            FastStack<AABVHNode*, 256> nodeStack;

            nodeStack.reset();
            nodeStack.push(root_);

            AABVHNode* node;

            while (nodeStack.pop(node))
            {
                //	Move on to the next node if there is no intersection between with node and the bounding box

                if (node->boundingBox().doesNotIntersect(triangle.boundingBox()))
                {
                    continue;
                }

                //	We have an intersection with the bounding box.
                //		If the node is a leaf node, then check each blob attached to the node
                //		and execute the action if the blob intersections the bounding box.  Otherwise,
                //		just push back the right and left nodes and loop again.
                //
                //	Blobs have been pre-sorted into two lists by boolAlgID.  Using the list for the same
                //		boolAlgID as the triangle searches for self intersections.  Using the list for the
                //		other boolAlgID than the triangle serches for intersections between the triangle
                //		and the other mesh.

                if (node->isLeaf())
                {
                    const BlobIDList& blobIds = node->blobIDLists()[blobIDListSelector];

                    for (Primitives::IndexType bid : blobIds)
                    {
                        auto& currentBlob = (*blobs_)[bid];

                        if (currentBlob.boundingBox().intersects(triangle.boundingBox()))
                        {
                            edges.push_back(currentBlob.index());
                        }
                    }
                }
                else
                {
                    nodeStack.push2(node->left(), node->right());
                }
            }

            return edges;
        };

       private:
        //	Data members

        AABVHNode* root_;

        AABVHNodeListCollection& node_collections_;

        std::unique_ptr<GeomBlobVector> blobs_;

        std::vector<NUMERIC_PRECISION> representative_points_[3];

        std::vector<Primitives::IndexType> tmpids_;

        Xoshiro256Plus random_number_generator_;

        const SolverControlBlock& solver_control_block_;

        // process range of tmpids including begin, excluding end last_dim provides a hint by saying which dimension a
        // split was last made along

        void QuickSelect(size_t select, size_t begin, size_t end, size_t dim);

        AABVHNode* ConstructTree(size_t begin, size_t end, size_t lastDim);

        AABVHNode* ConstructTreeRecursive(AABVHNodeList& nodeStorage, size_t begin, size_t end, size_t lastDim);
    };
}  // namespace Cork::AABVH
