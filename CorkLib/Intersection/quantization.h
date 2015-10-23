// +-------------------------------------------------------------------------
// | quantization.h
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

#include<cmath>

#include "..\Math\Primitives.h"




namespace Quantization
{
	// NOTE: none of these values should be modified by the clients
	
	const int BITS = 30;


	extern double MAGNIFY;
	extern double RESHRINK;

	extern __m128	MAGNIFY_SSE;
	extern __m128	RESHRINK_SSE;


	inline
	int quantize2int(double number)
	{
		return( int(number * MAGNIFY));
	}

	inline
	double quantizedInt2double(int number)
	{
		return( RESHRINK * double(number));
	}

	inline
	double quantize(double number)
	{
		return( RESHRINK * double(int(number * MAGNIFY)));
	}

	inline
	double quantize(float number)
	{
		return( (float)RESHRINK * float(int(number * (float)MAGNIFY)));
	}

	inline
	Cork::Math::Vector3D	quantize( const Cork::Math::Vector3D&		vectorToQuantize )
	{
#ifdef CORK_SSE

		__m128		magnifiedVector = _mm_mul_ps( vectorToQuantize, MAGNIFY_SSE );
		__m128i		integerMagVec = _mm_cvttps_epi32( magnifiedVector );
		__m128		floatMagVec = _mm_cvtepi32_ps( integerMagVec );
		__m128		reshrunkVector = _mm_mul_ps( floatMagVec, RESHRINK_SSE );
		
		return( Cork::Math::Vector3D( reshrunkVector ));

#else

		return( Cork::Math::Vector3D( quantize( vectorToQuantize.x() ), quantize( vectorToQuantize.y() ), quantize( vectorToQuantize.z() ) ) );

#endif

	}


	//	Given the specified number of bits, and bound on the coordinate values of points,
	//		fit as fine-grained a grid as possible over the space.

	inline
	void calibrate(double maximumMagnitude)
	{
		int max_exponent;
		std::frexp(maximumMagnitude, &max_exponent);
		max_exponent++; // ensure that 2^max_exponent > maximumMagnitude
    
		// set constants
		MAGNIFY = std::pow(2.0, BITS - max_exponent);
		
		// we are guaranteed that maximumMagnitude * MAGNIFY < 2.0^BITS
		RESHRINK = std::pow(2.0, max_exponent - BITS);

		{
			float	magnify = (float)MAGNIFY;
			float	reshrink = (float)RESHRINK;

			MAGNIFY_SSE = _mm_load_ps1( &magnify );
			RESHRINK_SSE = _mm_load_ps1( &reshrink );
		}
	}

} // end namespace Quantization


