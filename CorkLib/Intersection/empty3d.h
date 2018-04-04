// +-------------------------------------------------------------------------
// | empty3d.h
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


#include <array>

#include "ext4.h"
#include "absext4.h"
#include "fixext4.h"
#include "gmpext4.h"

#include "..\Math\Primitives.h"



namespace Cork
{

	namespace Empty3d
	{

		class ExactArithmeticContext
		{
		public:


			int degeneracy_count = 0;
			int exact_count = 0;
			int callcount = 0;
		};


		struct TriIn
		{
			TriIn( const Cork::Math::Vector3D&		tri0,
				   const Cork::Math::Vector3D&		tri1,
				   const Cork::Math::Vector3D&		tri2 )
			  : p0( tri0 ),
				p1( tri1 ),
				p2( tri2 )
			{}

			const Cork::Math::Vector3D&		p0;
			const Cork::Math::Vector3D&		p1;
			const Cork::Math::Vector3D&		p2;
		};

		struct EdgeIn
		{
			EdgeIn( const Cork::Math::Vector3D&		edge0,
					const Cork::Math::Vector3D&		edge1 )
				: p0( edge0 ),
				  p1( edge1 )
			{}

			const Cork::Math::Vector3D&		p0;
			const Cork::Math::Vector3D&		p1;
		};



		class TriEdgeIn
		{
		public :

			TriEdgeIn( const TriIn&&	triangle,
					   const EdgeIn&&	_edge )
			   : tri( triangle ),
				 edge( _edge )
			{}

			TriIn   tri;
			EdgeIn  edge;

			bool isEmpty( ExactArithmeticContext&			context ) const;
			bool emptyExact( ExactArithmeticContext&		context ) const;

			Cork::Math::Vector3D coords() const;
			Cork::Math::Vector3D coordsExact() const;


		private :

			int			emptyFilter() const;

			bool		exactFallback( ExactArithmeticContext&		context ) const;
		};


		Cork::Math::Vector3D coordsExact( const GMPExt4::GmpExt4_2&		edge,
										  const GMPExt4::GmpExt4_3&		triangle );



		class TriTriTriIn
		{
		public :

			TriTriTriIn( const TriIn&&	triangle0,
						 const TriIn&&	triangle1,
						 const TriIn&&	triangle2 )
				: m_tri( {{ triangle0, triangle1, triangle2 }} )
			{}
				   

			const std::array<TriIn,3>&			triangle() const
			{
				return( m_tri );
			}

			bool isEmpty( ExactArithmeticContext&			context ) const;
			bool emptyExact( ExactArithmeticContext&		context ) const;
		
			Cork::Math::Vector3D		coords() const;
			Cork::Math::Vector3D		coordsExact() const;


		private:
			
			std::array<TriIn,3>			m_tri;

			int							emptyFilter() const;

			bool						exactFallback( ExactArithmeticContext&		context ) const;
		};



		void toGmpExt( GMPExt4::GmpExt4_1 &out, const Cork::Math::Vector3D &in );
	//	{
	//		out.e0 = Quantization::quantize2int( in.x() );
	//		out.e1 = Quantization::quantize2int( in.y() );
	//		out.e2 = Quantization::quantize2int( in.z() );
	//		out.e3 = 1;
	//	}



		Cork::Math::Vector3D coordsExact( const GMPExt4::GmpExt4_3&		triangle0,
										  const GMPExt4::GmpExt4_3&		triangle1,
										  const GMPExt4::GmpExt4_3&		triangle2 );


	} //	namespace Empty3d

}	//	namespace Cork
