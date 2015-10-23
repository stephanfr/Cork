// +-------------------------------------------------------------------------
// | Ray3D.h
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



namespace Cork
{
	namespace Math
	{


		struct Ray3D final
		{
    
			Ray3D()
			{}
    
			Ray3D( const Vector3D&		point,
				   const Vector3D&		dir )
				: p( point ),
				  r( dir )
			{}

			Ray3D(const Ray3D&		cp)
				: p( cp.p ),
				  r( cp.r )
			{}


			const Vector3D&			origin() const
			{
				return( p );
			}

			const Vector3D&			direction() const
			{
				return( r );
			}



		private :

			Vector3D	p; // point of origin
			Vector3D	r; // ray direction
		};



		inline std::ostream& operator<<(std::ostream &out, const Ray3D&	ray)
		{
			return out << '[' << ray.origin() << ';' << ray.direction() << ']';
		}

	}	//	namespace Math
}		//	namespace Cork



