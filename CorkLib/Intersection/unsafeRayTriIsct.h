// +-------------------------------------------------------------------------
// | unsafeRayTriIsct.h
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


#include "../Math/Ray3D.h"




inline
bool CheckForRayTriangleIntersection( Cork::Math::Ray3D&			ray,
									  Cork::Math::Vector3D&			va,
									  Cork::Math::Vector3D&			vb,
									  Cork::Math::Vector3D&			vc )
{
	// re-center the problem at the base point of the ray
	va -= ray.origin();
	vb -= ray.origin();
	vc -= ray.origin();

	// Then compute volumes of tetrahedra spanning
	//  * the base point / ray direction line segment
	//  * an edge of the triangle
	// Keeping orientations in mind...

	double volAB = determinant( va, vb, ray.direction() );
	double volBC = determinant( vb, vc, ray.direction() );
	double volCA = -determinant( va, vc, ray.direction() );

	// then also compute the volume of tet with the entire triangle as a face...

	double volABC = determinant( va, vb, vc );

	// if any of the signs of the edge tests
	// disagree with the sign of the whole triangle, then
	// the ray does not pass through the triangle

	if( ( volAB * volABC < 0 ) ||
		( volBC * volABC < 0 ) ||
		( volCA * volABC < 0 ) )
	{
		return( false );
	}

	// otherwise, compute the t - value for the ray to intersect
	// if this is negative, then the client can detect that the
	// ray would have to travel backwards to hit the triangle in question.

	double edgeSum = volAB + volBC + volCA;

	if( edgeSum == 0 )
	{
		return( false );
	}

	return( (NUMERIC_PRECISION)( volABC / edgeSum ) > 0 );
}




inline
bool CheckForRayTriangleIntersectionWithPoint( Cork::Math::Ray3D&				ray,
											   Cork::Math::Vector3D&			va,
											   Cork::Math::Vector3D&			vb,
											   Cork::Math::Vector3D&			vc,
											   NUMERIC_PRECISION*				t,
											   Cork::Math::Vector3D*			bary )
{
    ENSURE(t);
    ENSURE(bary);
    
    // re-center the problem at the base point of the ray
    va -= ray.origin();
    vb -= ray.origin();
    vc -= ray.origin();
    
    // Then compute volumes of tetrahedra spanning
    //  * the base point / ray direction line segment
    //  * an edge of the triangle
    // Keeping orientations in mind...

    double volAB  =   determinant(va, vb, ray.direction());
    double volBC  =   determinant(vb, vc, ray.direction());
    double volCA  = - determinant(va, vc, ray.direction());

    // then also compute the volume of tet with the entire triangle as a face...

    double volABC =   determinant(va, vb, vc);

    // if any of the signs of the edge tests
    // disagree with the sign of the whole triangle, then
    // the ray does not pass through the triangle

    if(( volAB * volABC < 0 ) ||
       ( volBC * volABC < 0 )||
       ( volCA * volABC < 0 ))
	{
		return( false );
	}
    
    // otherwise, compute the t - value for the ray to intersect
    // if this is negative, then the client can detect that the
    // ray would have to travel backwards to hit the triangle in question.
    
	double edgeSum = volAB + volBC + volCA;
    
	if(edgeSum == 0)
	{
		return( false );
	}

    *t = (NUMERIC_PRECISION)( volABC / (volAB + volBC + volCA));
    
	if(*t <= 0)
	{
		return( false );
	}

    *bary = Cork::Math::Vector3D( (NUMERIC_PRECISION)(volBC/edgeSum), (NUMERIC_PRECISION)(volCA/edgeSum), (NUMERIC_PRECISION)(volAB/edgeSum) );
    
	return( true );
}



