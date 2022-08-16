// +-------------------------------------------------------------------------
// | aabvh.cpp
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

#include "aabvh.hpp"

#include "math/gmpext4.hpp"
#include "tbb/task_group.h"

namespace Cork::AABVH
{
    // precondition: begin <= select < end

    inline void AxisAlignedBoundingVolumeHierarchy::QuickSelect(size_t select, size_t begin, size_t end, size_t dim)
    {
        // NOTE: values equal to the pivot may appear on either side of the split

        if (end - 1 == select)
        {
            return;
        }

        const NUMERIC_PRECISION* representative_points{ representative_points_[dim].data() };

        size_t pivot_index{(random_number_generator_.next() % (end - begin)) + begin};

        NUMERIC_PRECISION pivot_value{ representative_points[tmpids_[pivot_index]] };

        //	I don't usually care for pointer arithmetic but it makes a substantive difference here.
        //
        //	When multi-threaded, the changes to the indices front and back are OK as the tasks are working on
        //		separate parts of the tree so there is no risk of hitting the same tmpids at the same time.

        IndexType* front{ &tmpids_[begin] };
        IndexType* back{ &tmpids_[end - 1] };

        while (front < back)
        {
            if (representative_points[*front] < pivot_value)
            {
                front++;
            }
            else if (representative_points[*back] > pivot_value)
            {
                back--;
            }
            else
            {
                std::swap(*front, *back);
                front++;
                back--;
            }
        }

        if ((front == back) && (representative_points[*front] <= pivot_value))
        {
            front++;
        }

        if (select < uint(front - &tmpids_[0]))
        {
            QuickSelect(select, begin, front - &tmpids_[0], dim);
        }
        else
        {
            QuickSelect(select, front - &tmpids_[0], end, dim);
        }
    };


    // process range of tmpids including begin, excluding end
    // last_dim provides a hint by saying which dimension a
    // split was last made along

    AABVHNode* AxisAlignedBoundingVolumeHierarchy::ConstructTree(size_t begin, size_t end, size_t last_dim)
    {
        constexpr int INITIAL_NODE_LIST_SIZE = 8;

        assert(end - begin > 0);

        // base case

        if (end - begin <= MAXIMUM_LEAF_SIZE)
        {
            AABVHNodeList& node_list = node_collections_.getNodeList(INITIAL_NODE_LIST_SIZE);

            node_list.emplace_back();
            AABVHNode* node{ &node_list.back() };

            for (uint k = 0; k < end - begin; k++)
            {
                IndexType blobid {tmpids_[begin + k]};

                node->AddBlobID((*blobs_)[blobid].index().boolean_algorithm_data(), blobid);

                node->boundingBox().convex((*blobs_)[blobid].boundingBox());
            }

            return node;
        }

        // otherwise, let's try to split this geometry up

        size_t dim = (last_dim + 1) % 3;
        size_t mid = (begin + end) / 2;

        QuickSelect(mid, begin, end, dim);

        AABVHNode* node1 = nullptr;
        AABVHNode* node2 = nullptr;

        if (solver_control_block_.use_multiple_threads())
        {
            tbb::task_group task_group;

            //	Recurse - but by splitting into a pair of tasks

            task_group.run([&] {
                node1 = ConstructTreeRecursive(node_collections_.getNodeList((end - begin) / (MAXIMUM_LEAF_SIZE / 2)), begin,
                                               mid, dim);
            });
            node2 =
                ConstructTreeRecursive(node_collections_.getNodeList((end - begin) / (MAXIMUM_LEAF_SIZE / 2)), mid, end, dim);

            //	Wait for the two tasks to complete

            task_group.wait();
        }
        else
        {
            //	Recurse directly

            node1 =
                ConstructTreeRecursive(node_collections_.getNodeList((end - begin) / (MAXIMUM_LEAF_SIZE / 2)), begin, mid, dim);
            node2 =
                ConstructTreeRecursive(node_collections_.getNodeList((end - begin) / (MAXIMUM_LEAF_SIZE / 2)), mid, end, dim);
        }

        //	Create the final node and set the bounding box

        AABVHNodeList& primary_node_list = node_collections_.getPrimaryNodeList();

        primary_node_list.emplace_back(node1, node2);
        AABVHNode* node = &primary_node_list.back();

        node1->boundingBox().convex(node2->boundingBox(), node->boundingBox());

        //	Return the node

        return node;
    };

    AABVHNode* AxisAlignedBoundingVolumeHierarchy::ConstructTreeRecursive(AABVHNodeList& node_storage, size_t begin,
                                                                          size_t end, size_t last_dim)
    {
        assert(end - begin > 0);

        // base case

        if (end - begin <= MAXIMUM_LEAF_SIZE)
        {
            node_storage.emplace_back();
            AABVHNode* node = &node_storage.back();

            for (uint k = 0; k < end - begin; k++)
            {
                IndexType blobid = tmpids_[begin + k];

                node->AddBlobID((*blobs_)[blobid].index().boolean_algorithm_data(), blobid);

                node->boundingBox().convex((*blobs_)[blobid].boundingBox());
            }

            return node;
        }

        // otherwise, let's try to split this geometry up

        size_t dim = (last_dim + 1) % 3;
        size_t mid = (begin + end) / 2;

        QuickSelect(mid, begin, end, dim);

        // now recurse

        AABVHNode* left = ConstructTreeRecursive(node_storage, begin, mid, dim);
        AABVHNode* right = ConstructTreeRecursive(node_storage, mid, end, dim);

        node_storage.emplace_back(left, right);
        AABVHNode* node = &node_storage.back();

        left->boundingBox().convex(right->boundingBox(), node->boundingBox());

        return node;
    };
}  // namespace Cork::AABVH

