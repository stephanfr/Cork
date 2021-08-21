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



#include "./intersection/gmpext4.h"

#include "accel/aabvh.h"

#include "tbb/task_group.h"




namespace Cork
{
	namespace AABVH
	{

		// precondition: begin <= select < end

		inline
		void		AxisAlignedBoundingVolumeHierarchy::QuickSelect( size_t			select,
																     size_t			begin,
																	 size_t			end,
																	 size_t			dim )  
		{
			// NOTE: values equal to the pivot may appear on either side of the split

			if (end - 1 == select)
			{
				return;
			}

			const NUMERIC_PRECISION*		representativePoints = m_representativePoints[dim].data();
			
			size_t							pivotIndex = ( random_number_generator_.next() % (end - begin)) + begin;
			NUMERIC_PRECISION				pivotValue = representativePoints[m_tmpids[pivotIndex]];
			
			//	I don't usually care for pointer arithmetic but it makes a substantive difference here.
			//
			//	When multi-threaded, the changes to the indices front and back are OK as the tasks are working on
			//		separate parts of the tree so there is no risk of hitting the same tmpids at the same time.

			IndexType*		front = &m_tmpids[begin];
			IndexType*		back = &m_tmpids[end-1];

			while ( front < back)
			{
				if( representativePoints[*front] < pivotValue )
				{
					front++;
				}
				else if( representativePoints[*back] > pivotValue )
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

			if (( front == back ) && ( representativePoints[*front] <= pivotValue ))
			{
				front++;
			}

			if (select < uint( front - &m_tmpids[0] ))
			{
				QuickSelect(select, begin, front - &m_tmpids[0], dim);
			}
			else
			{
				QuickSelect(select, front - &m_tmpids[0], end, dim);
			}
		};


		// process range of tmpids including begin, excluding end
		// last_dim provides a hint by saying which dimension a
		// split was last made along

		AABVHNode*		AxisAlignedBoundingVolumeHierarchy::ConstructTree( size_t				begin,
																		   size_t				end,
																		   size_t				lastDim )
		{
			assert(end - begin > 0);

			// base case

			if (end - begin <= LEAF_SIZE)
			{
				AABVHNodeList& nodeList = m_nodeCollections.getNodeList( 8 );

				nodeList.emplace_back();
				AABVHNode*	node = &nodeList.back();

				for (uint k = 0; k < end - begin; k++)
				{
					IndexType	blobid = m_tmpids[begin + k];

					node->AddBlobID( std::to_integer<size_t>((*m_blobs)[blobid].index().boolAlgData()), blobid );

					node->boundingBox().convex( (*m_blobs)[blobid].boundingBox() );
				}

				return(node);
			}

			// otherwise, let's try to split this geometry up

			size_t	dim = (lastDim + 1) % 3;
			size_t	mid = (begin + end) / 2;

			QuickSelect(mid, begin, end, dim);


			AABVHNode*		node1;
			AABVHNode*		node2;

			if( m_solverControlBlock.useMultipleThreads() )
			{
				tbb::task_group		taskGroup;
				
				//	Recurse - but by splitting into a pair of tasks

				taskGroup.run([&]{ node1 = ConstructTreeRecursive( m_nodeCollections.getNodeList( ( end - begin ) / ( LEAF_SIZE / 2 )), begin, mid, dim ); } );
				node2 = ConstructTreeRecursive( m_nodeCollections.getNodeList( ( end - begin ) / ( LEAF_SIZE / 2 )), mid, end, dim );

				//	Wait for the two tasks to complete

				taskGroup.wait();
			}
			else
			{
				//	Recurse directly

				node1 = ConstructTreeRecursive( m_nodeCollections.getNodeList( ( end - begin ) / ( LEAF_SIZE / 2 )), begin, mid, dim );
				node2 = ConstructTreeRecursive( m_nodeCollections.getNodeList( ( end - begin ) / ( LEAF_SIZE / 2 )), mid, end, dim );
			}
			
			//	Create the final node and set the bounding box
			
			AABVHNodeList&		primaryNodeList = m_nodeCollections.getPrimaryNodeList();

			primaryNodeList.emplace_back( node1, node2 );
			AABVHNode*	node = &primaryNodeList.back();

			node1->boundingBox().convex( node2->boundingBox(), node->boundingBox() );

			//	Return the node

			return( node );
		};



		AABVHNode*		AxisAlignedBoundingVolumeHierarchy::ConstructTreeRecursive( AABVHNodeList&		nodeStorage,
																					size_t				begin,
																				    size_t				end,
																				    size_t				lastDim )
		{
			assert(end - begin > 0);

			// base case

			if (end - begin <= LEAF_SIZE)
			{
				nodeStorage.emplace_back();
				AABVHNode*	node = &nodeStorage.back();

				for (uint k = 0; k < end - begin; k++)
				{
					IndexType	blobid = m_tmpids[begin + k];

					node->AddBlobID( std::to_integer<size_t>((*m_blobs)[blobid].index().boolAlgData()), blobid );

					node->boundingBox().convex( (*m_blobs)[blobid].boundingBox() );
				}

				return(node);
			}

			// otherwise, let's try to split this geometry up

			size_t	dim = (lastDim + 1) % 3;
			size_t	mid = (begin + end) / 2;

			QuickSelect(mid, begin, end, dim);

			// now recurse

			AABVHNode*	left = ConstructTreeRecursive( nodeStorage, begin, mid, dim);
			AABVHNode*	right =  ConstructTreeRecursive( nodeStorage, mid, end, dim);

			nodeStorage.emplace_back( left, right );
			AABVHNode*	node = &nodeStorage.back();

			left->boundingBox().convex( right->boundingBox(), node->boundingBox() );

			return( node );
		};

	}	//	namespace AABVH
}		//	namespace Cork




