// +-------------------------------------------------------------------------
// | Primitives.h
// | 
// | Author: Stephan Friedl
// +-------------------------------------------------------------------------
// | COPYRIGHT:
// |    Copyright Stephan Friedl 2015
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



#include <cmath>
#include <cstdlib>
#include <iostream>
#include <array>
#include <vector>
#include <map>

#include <boost\align\aligned_allocator.hpp>

#include "..\CorkDefs.h"

#include "..\util\prelude.h"




//
//	Setup the numeric precision and the vector implementation for the build.
//


#include "Vector2DTemplate.h"

namespace Cork
{
	namespace Math
	{
		typedef Vector2DTemplate<NUMERIC_PRECISION>			Vector2D;
	}
}



#include "Vector3DTemplate.h"

namespace Cork
{
	namespace Math
	{
#ifdef __CORK_AVX__
		__declspec( align( 32 ) )
#endif
			typedef Vector3DTemplate<NUMERIC_PRECISION>			Vector3D;
	}
}




//	The Ray and Bounding Box clesses depend on the Vector3D and Vertex3D classes

#include "Ray3D.h"

#include "BoundingBox.h"





namespace Cork
{
	typedef size_t		IndexType;

	namespace Math
	{


		//	Vertices and vectors share the same implementation of a numeric 3-tuple

		typedef Vector3D	Vertex3D;



		struct Vertex3DMapCompare
		{
			bool operator ()( const Vertex3D&		vertex1,
							  const Vertex3D&		vertex2 ) const
			{
				//	Equality is by x, then y and finally z

				if ( vertex1.x() < vertex2.x() )
				{
					return(true);
				}

				if (  vertex1.x() > vertex2.x() )
				{
					return(false);
				}

				//	X values are equal

				if (  vertex1.y() < vertex2.y() )
				{
					return(true);
				}

				if (  vertex1.y() > vertex2.y() )
				{
					return(false);
				}

				//	X and Y values are equal

				if (  vertex1.z() < vertex2.z() )
				{
					return(true);
				}

				if (  vertex1.z() > vertex2.z() )
				{
					return(false);
				}

				//	The only way we should end up down here is if the two vertices are equal

				return(false);
			}
		};



		typedef std::vector<Vector3D, boost::alignment::aligned_allocator<Vector3D>>								Vector3DVector;
		typedef std::vector<Vertex3D, boost::alignment::aligned_allocator<Vertex3D>>								Vertex3DVector;

		typedef std::map<Vertex3D, IndexType, Vertex3DMapCompare, boost::alignment::aligned_allocator<Vertex3D>>	Vertex3DMap;


		//
		//	Finally, define some base classes for key mesh classes: Triangle by vertex indices, Edges and Triangle by vertex points
		//


		typedef size_t	VertexIndex;


		class TriangleByIndicesBase
		{
		public :

			TriangleByIndicesBase()
				: m_a( -1 ),
				  m_b( -1 ),
				  m_c( -1 )
			{}

			TriangleByIndicesBase( VertexIndex		a,
								   VertexIndex		b,
								   VertexIndex		c )
				: m_a( a ),
				  m_b( b ),
				  m_c( c )
			{}

	
			virtual ~TriangleByIndicesBase()
			{}


			const VertexIndex				operator[]( size_t		index ) const
			{
				return( m_indices[index] );
			}

			VertexIndex&					operator[]( size_t		index )
			{
				return( m_indices[index] );
			}


			const VertexIndex				a() const
			{
				return( m_a );
			}

			VertexIndex&					a()
			{
				return( m_a );
			}

			const VertexIndex				b() const
			{
				return( m_b );
			}

			VertexIndex&					b()
			{
				return( m_b );
			}

			const VertexIndex				c() const
			{
				return( m_c );
			}

			VertexIndex&					c()
			{
				return( m_c );
			}


		protected :

			union
			{
				struct
				{
					VertexIndex			m_a;
					VertexIndex			m_b;
					VertexIndex			m_c;
				};

				std::array<VertexIndex,3>		m_indices;
			};
		};





		class EdgeBase
		{
		public :

			EdgeBase( VertexIndex		a,
					  VertexIndex		b )
				: m_vertexA( std::min( a, b ) ),
				  m_vertexB( std::max( a, b ) )
			{}

			virtual ~EdgeBase()
			{}


			VertexIndex			vertexA() const
			{
				return( m_vertexA );
			}

			VertexIndex			vertexB() const
			{
				return( m_vertexB );
			}

			bool		operator==( const EdgeBase&		edgeToCompare ) const
			{
				return(( vertexA() == edgeToCompare.vertexA() ) && ( vertexB() == edgeToCompare.vertexB() ));
			}


		private :

			VertexIndex			m_vertexA;
			VertexIndex			m_vertexB;
		};






		class TriangleByVerticesBase
		{
		public :

			TriangleByVerticesBase( const Vertex3D&			firstVertex,
									const Vertex3D&			secondVertex,
									const Vertex3D&			thirdVertex )
				: m_vertices( { firstVertex, secondVertex, thirdVertex } )
			{}


			const Vertex3D&		operator[]( unsigned int	index ) const
			{
				return(m_vertices[index]);
			}


			const Vertex3D&				vertexA() const
			{
				return( m_vertices[0] );
			}

			const Vertex3D&				vertexB() const
			{
				return( m_vertices[1] );
			}

			const Vertex3D&				vertexC() const
			{
				return( m_vertices[2] );
			}


			Vector3D					edgeAB() const
			{
				return( m_vertices[0] - m_vertices[1] );
			}

			Vector3D					edgeAC() const
			{
				return( m_vertices[0] - m_vertices[2] );
			}

			Vector3D					edgeBC() const
			{
				return( m_vertices[1] - m_vertices[2] );
			}


		private:

			std::array<Vertex3D, 3>			m_vertices;

		};


	}	//	Namespace Math
}		//	Namespace Cork

