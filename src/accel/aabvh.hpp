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

#include "oneapi/tbb/spin_mutex.h"
#include "oneapi/tbb/task_group.h"

#include "cork.hpp"
#include "mesh/topo_cache.hpp"
#include "util/fast_stack.hpp"
#include "util/managed_intrusive_list.hpp"
#include "Xoshiro256Plus.h"

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

    const unsigned int MAXIMUM_LEAF_SIZE = 32;

    //  Default initial sizes for various collections and lists

    const unsigned int DEFAULT_NODE_LIST_COLLECTION_SIZE = 16;
    const unsigned int DEFAULT_PRIMARY_NODE_LIST_SIZE = 1024;
    const unsigned int DEFAULT_NODE_STACK_SIZE = 256;
    

    class alignas(SIMD_MEMORY_ALIGNMENT) GeomBlob
    {
       public:
        explicit GeomBlob(const Meshes::TopoEdge& topo_edge) : topo_edge_(topo_edge)
        {
            const Vector3D& p0 = (topo_edge_.verts()[0])->quantized_value();
            const Vector3D& p1 = (topo_edge_.verts()[1])->quantized_value();

            bbox_ = BBox3D(p0.min(p1), p0.max(p1));
        }

        [[nodiscard]] const Meshes::TopoEdge& index() const { return (topo_edge_); }

        [[nodiscard]] const BBox3D& boundingBox() const { return (bbox_); }

       private:
        BBox3D bbox_;

        const Meshes::TopoEdge& topo_edge_;
    };

    using GeomBlobVector = std::vector<GeomBlob>;

    //	A BlobIDList will never be larger than MAXIMUM_LEAF_SIZE so a static vector is OK

    using BlobIDList = boost::container::static_vector<IndexType, MAXIMUM_LEAF_SIZE>;

    class alignas(SIMD_MEMORY_ALIGNMENT) AABVHNode      //  Padding here for SIMD alignment
    {
       public:
        AABVHNode() = default;

        AABVHNode(AABVHNode* left, AABVHNode* right) : left_(left), right_(right) {}

        [[nodiscard]] inline bool isLeaf() const { return (left_ == nullptr); }

        [[nodiscard]] AABVHNode* left() const { return (left_); }

        [[nodiscard]] AABVHNode* right() const { return (right_); }

        [[nodiscard]] const BBox3D& boundingBox() const { return (bbox_); }

        [[nodiscard]] BBox3D& boundingBox() { return (bbox_); }

        [[nodiscard]] const std::array<BlobIDList, 2>& blobIDLists() const { return (blobids_); }

        void AddBlobID(IndexType listIndex, IndexType blobID)
        {
            blobids_[listIndex].emplace_back(blobID);
        }

       private:
        BBox3D bbox_;

        AABVHNode* left_{nullptr};
        AABVHNode* right_{nullptr};

        std::array<BlobIDList, 2> blobids_;
    };

    using AABVHNodeList = std::vector<AABVHNode>;

    class AABVHNodeListCollection
    {
       public:
        AABVHNodeListCollection() { node_collections_.reserve(DEFAULT_NODE_LIST_COLLECTION_SIZE); }

        void reset()
        {
            for (auto& nodeVector : node_collections_)
            {
                nodeVector.clear();
            }

            num_collections_checked_out_ = 0;
        }

        [[nodiscard]] AABVHNodeList& getPrimaryNodeList()
        {
            if (node_collections_.empty())
            {
                return (getNodeList(DEFAULT_PRIMARY_NODE_LIST_SIZE));
            }

            return (node_collections_[0]);
        }

        [[nodiscard]] AABVHNodeList& getNodeList(size_t reservation)
        {
            collections_mutex_.lock();

            if (node_collections_.size() <= num_collections_checked_out_)
            {
                node_collections_.push_back(new AABVHNodeList());
            }

            node_collections_[num_collections_checked_out_].reserve(reservation);

            AABVHNodeList& listToReturn = node_collections_[num_collections_checked_out_++];

            collections_mutex_.unlock();

            return (listToReturn);
        }

       private:
        boost::ptr_vector<AABVHNodeList> node_collections_;
        oneapi::tbb::spin_mutex collections_mutex_;

        size_t num_collections_checked_out_{0};
    };

    class Workspace
    {
       public:
        Workspace() = default;
        Workspace( const Workspace& ) = delete;
        Workspace( Workspace&& ) = delete;

        ~Workspace() = default;

        const Workspace& operator=( const Workspace& ) = delete;
        const Workspace& operator=( Workspace&& ) = delete;

        void reset() { nodeList_collection_.reset(); }

        [[nodiscard]] AABVHNodeListCollection& getAABVHNodeListCollection() { return (nodeList_collection_); }

       private:
        AABVHNodeListCollection nodeList_collection_;
    };

    enum class IntersectionType
    {
        SELF_INTERSECTION = 0,
        BOOLEAN_INTERSECTION
    };

    class AxisAlignedBoundingVolumeHierarchy : public boost::noncopyable
    {
       public:
        AxisAlignedBoundingVolumeHierarchy() = delete;
        AxisAlignedBoundingVolumeHierarchy( const AxisAlignedBoundingVolumeHierarchy& ) = delete;
        AxisAlignedBoundingVolumeHierarchy( AxisAlignedBoundingVolumeHierarchy&& ) = delete;

        AxisAlignedBoundingVolumeHierarchy(std::unique_ptr<GeomBlobVector>& geoms, Workspace& workspace,
                                           const SolverControlBlock& solver_control_block)
            : root_(nullptr),
              blobs_(std::move(geoms)),
              tmpids_(blobs_->size()),
              node_collections_(workspace.getAABVHNodeListCollection()),
              random_number_generator_(RANDOM_SEED),
              solver_control_block_(solver_control_block)
        {
            assert( !blobs_->empty() );

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
                Vector3D repPoint(
                    blob.boundingBox().minima() +
                    ((blob.boundingBox().maxima() - blob.boundingBox().minima()) / (NUMERIC_PRECISION)2.0));    //  NOLINT(cppcoreguidelines-avoid-magic-numbers, readability-magic-numbers)

                representative_points_[0].emplace_back(repPoint.x());
                representative_points_[1].emplace_back(repPoint.y());
                representative_points_[2].emplace_back(repPoint.z());
            }

            root_ = ConstructTree(0, tmpids_.size(), 2);
        }

        ~AxisAlignedBoundingVolumeHierarchy() { node_collections_.reset(); }

        const AxisAlignedBoundingVolumeHierarchy& operator=( const AxisAlignedBoundingVolumeHierarchy& ) = delete;
        const AxisAlignedBoundingVolumeHierarchy& operator=( AxisAlignedBoundingVolumeHierarchy&& ) = delete;

        [[nodiscard]] Meshes::TopoEdgeReferenceVector EdgesIntersectingTriangle(const Meshes::TopoTri& triangle,
                                                                  IntersectionType intersection_type) const
        {
            Meshes::TopoEdgeReferenceVector edges;

            //	Set the boolAlgData index for intersections between two bodies or for self-intersections.

            unsigned int blob_id_list_selector =
                (intersection_type == IntersectionType::BOOLEAN_INTERSECTION ? triangle.bool_alg_data() ^ (uint32_t)1
                                                                            : triangle.bool_alg_data());

            //	Use a recursive search and save edges that intersect the triangle

            FastStack<AABVHNode*, DEFAULT_NODE_STACK_SIZE> node_stack;

            node_stack.reset();
            node_stack.push(root_);

            AABVHNode* node;

            while (node_stack.pop(node))
            {
                //	Move on to the next node if there is no intersection between with node and the bounding box

                if (node->boundingBox().doesNotIntersect(triangle.bounding_box()))
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
                    const BlobIDList& blobIds = node->blobIDLists()[blob_id_list_selector];

                    for (IndexType bid : blobIds)
                    {
                        auto& currentBlob = (*blobs_)[bid];

                        if (currentBlob.boundingBox().intersects(triangle.bounding_box()))
                        {
                            edges.push_back(currentBlob.index());
                        }
                    }
                }
                else
                {
                    node_stack.push2(node->left(), node->right());
                }
            }

            return edges;
        };

       private:
        //	Data members

        AABVHNode* root_;

        AABVHNodeListCollection& node_collections_;

        std::unique_ptr<GeomBlobVector> blobs_;

        std::array<std::vector<NUMERIC_PRECISION>,3> representative_points_;

        std::vector<IndexType> tmpids_;

        Xoshiro256Plus random_number_generator_;

        const SolverControlBlock& solver_control_block_;

        // process range of tmpids including begin, excluding end last_dim provides a hint by saying which dimension a
        // split was last made along

        void QuickSelect(size_t select, size_t begin, size_t end, size_t dim);

        [[nodiscard]] AABVHNode* ConstructTree(size_t begin, size_t end, size_t last_dim);

        [[nodiscard]] AABVHNode* ConstructTreeRecursive(AABVHNodeList& node_storage, size_t begin, size_t end, size_t last_dim);
    };
}  // namespace Cork::AABVH
