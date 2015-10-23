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



#include <vector>
#include <deque>

#include <boost\container\static_vector.hpp>
#include <boost\align\aligned_allocator.hpp>
#include <boost\ptr_container\ptr_vector.hpp>

#include "..\Util\FastStack.h"
#include "..\Util\ManagedIntrusiveList.h"

#include "..\math\Primitives.h"

#include "..\Mesh\TopoCache.h"

#include "tbb\mutex.h"
#include "tbb\task_group.h"



//
//	AABVH
//
//	Axis Aligned Bounding Box Volume Hierarchy
//
//	An AABVH is a balanced tree which subdivides the model at each branch point to provide quick searching
//		for intersections between edges and triangles.  The partitioning function 'quickselect' is the key to 
//		how the goemetries are arranged in the tree.
//




namespace Cork
{
	namespace AABVH
	{
		//	Maximum leaf size - needs to be a power of 2 (i.e. 2, 4, 8, 16, 32, 64)

		const unsigned int	LEAF_SIZE = 32;

		class GeomBlob
		{
		public :

			explicit GeomBlob(const TopoEdge&		idx )
				: m_id( idx )
			{
				const Cork::Math::Vector3D& p0 = *((idx.verts()[0])->quantizedValue());
				const Cork::Math::Vector3D& p1 = *((idx.verts()[1])->quantizedValue());

				m_bbox = Cork::Math::BBox3D( min( p0, p1 ), max( p0, p1 ));
			}




			const TopoEdge&					index() const
			{
				return( m_id );
			}

			const Cork::Math::BBox3D&		boundingBox() const
			{
				return( m_bbox );
			}


		private :

			Cork::Math::BBox3D			m_bbox;
			
			const TopoEdge&				m_id;
		};

		
		typedef std::vector<GeomBlob,boost::alignment::aligned_allocator<GeomBlob>>		GeomBlobVector;

		//	A BlobIDList will never be larger than LEAF_SIZE so a static vector is OK

		typedef boost::container::static_vector<IndexType, LEAF_SIZE>						BlobIDList;



		class AABVHNode // : public IntrusiveListHook
		{
		public:

			AABVHNode()
				: m_left( nullptr ),
				  m_right( nullptr )
			{}

			AABVHNode( AABVHNode*		left,
					   AABVHNode*		right )
				: m_left( left ),
				  m_right( right )
			{}

			inline bool isLeaf() const
			{
				return( m_left == nullptr );
			}



			AABVHNode*							left() const
			{
				return( m_left );
			}

			AABVHNode*							right() const
			{
				return( m_right );
			}


			const Cork::Math::BBox3D&			boundingBox() const
			{
				return( m_bbox );
			}

			Cork::Math::BBox3D&					boundingBox() 
			{
				return( m_bbox );
			}

			const std::array<BlobIDList,2>&		blobIDLists() const
			{
				return( m_blobids );
			}

			void								AddBlobID( IndexType		listIndex,
														   IndexType		blobID )
			{
				m_blobids[listIndex].emplace_back( blobID );
			}

		private :

			AABVHNode*						m_left;
			AABVHNode*						m_right;

			Cork::Math::BBox3D				m_bbox;
			std::array<BlobIDList,2>		m_blobids;
		};

		typedef std::vector<AABVHNode,boost::alignment::aligned_allocator<AABVHNode>>		AABVHNodeList;



		class AABVHNodeListCollection
		{
		public :

			AABVHNodeListCollection()
				: m_numCollectionsCheckedOut( 0 )
			{
				m_nodeCollections.reserve( 16 );			
			}


			void		reset()
			{
				for( auto& nodeVector : m_nodeCollections )
				{
					nodeVector.clear();
				}

				m_numCollectionsCheckedOut = 0;
			}

			AABVHNodeList&				getPrimaryNodeList()
			{
				if( m_nodeCollections.size() == 0 )
				{
					return( getNodeList( 1024 ) );
				}

				return( m_nodeCollections[0] );
			}

			AABVHNodeList&				getNodeList( size_t		reservation )
			{
				m_collectionsMutex.lock();

				if( m_nodeCollections.size() <= m_numCollectionsCheckedOut )
				{
					m_nodeCollections.push_back( new AABVHNodeList() );
				}

				m_nodeCollections[m_numCollectionsCheckedOut].reserve( reservation );

				AABVHNodeList&		listToReturn = m_nodeCollections[m_numCollectionsCheckedOut++];

				m_collectionsMutex.unlock();

				return( listToReturn );
			}


		private :

			boost::ptr_vector<AABVHNodeList>			m_nodeCollections;
			tbb::mutex									m_collectionsMutex;							

			size_t										m_numCollectionsCheckedOut;
		};



		class Workspace
		{
		public:

			Workspace()
			{}


			void		reset()
			{
				m_nodeListCollection.reset();
			}

			AABVHNodeListCollection&			getAABVHNodeListCollection()
			{
				return( m_nodeListCollection );
			}


			tbb::task_group&				getTaskGroup()
			{
				return( m_taskGroup );
			}

		private :

			tbb::task_group					m_taskGroup;

			AABVHNodeListCollection			m_nodeListCollection;
		};


		
		typedef enum class IntersectionType { SELF_INTERSECTION = 0, BOOLEAN_INTERSECTION };


		class AxisAlignedBoundingVolumeHierarchy : public boost::noncopyable
		{
		public:


			AxisAlignedBoundingVolumeHierarchy( std::unique_ptr<GeomBlobVector>&			geoms,
												Workspace&									workspace,
												const SolverControlBlock&					solverControlBlock )
				: m_root(nullptr),
				  m_blobs( std::move( geoms) ),
				  m_tmpids(m_blobs->size()),
				  m_nodeCollections( workspace.getAABVHNodeListCollection() ),
				  m_taskGroup( workspace.getTaskGroup() ),
				  m_solverControlBlock( solverControlBlock )
			{
				ENSURE(m_blobs->size() > 0);

				m_nodeCollections.reset();

				for (uint k = 0; k < m_tmpids.size(); k++)
				{
					m_tmpids[k] = k;
				}

				m_representativePoints[0].reserve( m_blobs->size() );
				m_representativePoints[1].reserve( m_blobs->size() );
				m_representativePoints[2].reserve( m_blobs->size() );

				for( auto& blob : *m_blobs )
				{
					Cork::Math::Vector3D	repPoint( blob.boundingBox().minima() + (( blob.boundingBox().maxima() - blob.boundingBox().minima()) / (NUMERIC_PRECISION)2.0) );

					m_representativePoints[0].emplace_back( repPoint.x() );
					m_representativePoints[1].emplace_back( repPoint.y() );
					m_representativePoints[2].emplace_back( repPoint.z() );
				}

				m_root = ConstructTree( 0, m_tmpids.size(), 2);
			}

			~AxisAlignedBoundingVolumeHierarchy()
			{}



			void AxisAlignedBoundingVolumeHierarchy::EdgesIntersectingTriangle( TopoTri&					triangle,
																				IntersectionType			intersectionType,
																				TopoEdgePointerVector&		edges )
			{ 
				//	Set the boolAlgData index for intersections between two bodies or for self-intersections.

				unsigned int		blobIDListSelector = intersectionType == IntersectionType::BOOLEAN_INTERSECTION ? triangle.boolAlgData() ^ 1 : triangle.boolAlgData();
				
				//	Use a recursive search and save edges that intersect the triangle

				FastStack<AABVHNode*,256>					nodeStack;

				nodeStack.reset();
				nodeStack.push( m_root );

				AABVHNode*		node;

				while ( nodeStack.pop( node ))
				{
					//	Move on to the next node if there is no intersection between with node and the bounding box

					if( node->boundingBox().doesNotIntersect(  triangle.boundingBox() ))
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
						const BlobIDList&		blobIds = node->blobIDLists()[blobIDListSelector];

						for( IndexType	bid : blobIds )
						{
							auto&		currentBlob = (*m_blobs)[bid];

							if( currentBlob.boundingBox().intersects( triangle.boundingBox() ) )
							{
								edges.push_back( &currentBlob.index() );
							}
						}
					}
					else
					{
						nodeStack.push2( node->left(), node->right() );
					}
				}
			};



		private:

			//	Data members

			AABVHNode*									m_root;

			AABVHNodeListCollection&					m_nodeCollections;

			std::unique_ptr<GeomBlobVector>				m_blobs;

			std::vector<NUMERIC_PRECISION>				m_representativePoints[3];

			std::vector<IndexType>						m_tmpids;

			tbb::task_group&							m_taskGroup;

			const SolverControlBlock&					m_solverControlBlock;


			// process range of tmpids including begin, excluding end last_dim provides a hint by saying which dimension a
			// split was last made along

			void					QuickSelect( size_t				select,
												 size_t				begin,
												 size_t				end,
												 size_t				dim );

			AABVHNode*				ConstructTree( size_t			begin,
												   size_t			end,
												   size_t			lastDim );

			AABVHNode*				ConstructTreeRecursive( AABVHNodeList&	nodeStorage,
															size_t			begin,
														    size_t			end,
														    size_t			lastDim );

		};

	}	//	namespace AABVH
}		//	namespace Cork






















