/////////////////////////////////////////////////////////////////////////////
// The Software is provided "AS IS" and possibly with faults. 
// Intel disclaims any and all warranties and guarantees, express, implied or
// otherwise, arising, with respect to the software delivered hereunder,
// including but not limited to the warranty of merchantability, the warranty
// of fitness for a particular purpose, and any warranty of non-infringement
// of the intellectual property rights of any third party.
// Intel neither assumes nor authorizes any person to assume for it any other
// liability. Customer will use the software at its own risk. Intel will not
// be liable to customer for any direct or indirect damages incurred in using
// the software. In no event will Intel be liable for loss of profits, loss of
// use, loss of data, business interruption, nor for punitive, incidental,
// consequential, or special damages of any kind, even if advised of
// the possibility of such damages.
//
// Copyright (c) 2003 Intel Corporation


//	SEF - lifted from https://software.intel.com/en-us/articles/fast-random-number-generator-on-the-intel-pentiumr-4-processor


//
// Third-party brands and names are the property of their respective owners
//

///////////////////////////////////////////////////////////////////////////
// Random Number Generation for SSE / SSE2
// Source File
// Version 0.1
// Author Kipp Owens, Rajiv Parikh
////////////////////////////////////////////////////////////////////////


#pragma once

#include <array>
#include <cmath>
#include <cstdlib>
#include <stdalign.h>

#include "emmintrin.h"



//define this if you wish to return values similar to the standard rand();

#define COMPATABILITY


void srand_sse( unsigned int seed );

void rand_sse( unsigned int* );

void drand_sse( std::array<float,4>& result, float min, float max );


alignas(16) static __m128i cur_seed;



inline
void srand_sse( unsigned int seed )
{
	cur_seed = _mm_set_epi32( seed, seed+1, seed, seed+1 );
}



inline
void rand_sse( unsigned int* result )
{

	alignas(16) __m128i cur_seed_split;
	alignas(16) __m128i multiplier;
	alignas(16) __m128i adder;
	alignas(16) __m128i mod_mask;
	alignas(16) __m128i sra_mask;
	alignas(16) __m128i sseresult;
	
	alignas(16) static const unsigned int mult[4] = { 214013, 17405, 214013, 69069 };
	alignas(16) static const unsigned int gadd[4] = { 2531011, 10395331, 13737667, 1 };
	alignas(16) static const unsigned int mask[4] = { 0xFFFFFFFF, 0, 0xFFFFFFFF, 0 };
	alignas(16) static const unsigned int masklo[4] = { 0x00007FFF, 0x00007FFF, 0x00007FFF, 0x00007FFF };



	adder = _mm_load_si128( (__m128i*) gadd);
	multiplier = _mm_load_si128( (__m128i*) mult);
	mod_mask = _mm_load_si128( (__m128i*) mask);
	sra_mask = _mm_load_si128( (__m128i*) masklo);
	cur_seed_split = _mm_shuffle_epi32( cur_seed, _MM_SHUFFLE( 2, 3, 0, 1 ) );


	cur_seed = _mm_mul_epu32( cur_seed, multiplier );
	multiplier = _mm_shuffle_epi32( multiplier, _MM_SHUFFLE( 2, 3, 0, 1 ) );
	cur_seed_split = _mm_mul_epu32( cur_seed_split, multiplier );


	cur_seed = _mm_and_si128( cur_seed, mod_mask);
	cur_seed_split = _mm_and_si128( cur_seed_split, mod_mask );
	cur_seed_split = _mm_shuffle_epi32( cur_seed_split, _MM_SHUFFLE( 2, 3, 0, 1 ) );

	cur_seed = _mm_or_si128( cur_seed, cur_seed_split );
	cur_seed = _mm_add_epi32( cur_seed, adder);



	// Add the lines below if you wish to reduce your results to 16-bit vals...

	#ifdef COMPATABILITY

	sseresult = _mm_srai_epi32( cur_seed, 16);
	sseresult = _mm_and_si128( sseresult, sra_mask );
	_mm_storeu_si128( (__m128i*) result, sseresult );

	return;

	#endif



	_mm_storeu_si128( (__m128i*) result, cur_seed);
}




const float		g_invMAX = 1.0f / float( RAND_MAX ); 

alignas(16) const __m128	g_invMAXSSE = _mm_load_ps1( &g_invMAX );


inline
void drand_sse( std::array<float,4>& result, float min, float max )
{
    float		range = max - min;

	alignas(16) __m128		rangeSSE = _mm_load_ps1( &range );
	alignas(16) __m128		minSSE = _mm_load_ps1( &min );

	alignas(16) __m128i		fourRandoms;

	rand_sse( (unsigned int*)&fourRandoms );

	alignas(16) __m128		temp;

	temp = _mm_cvtepi32_ps( fourRandoms );

	_mm_store_ps( result.data(), _mm_add_ps( _mm_mul_ps( _mm_mul_ps( temp, g_invMAXSSE ), rangeSSE ), minSSE ));
}

