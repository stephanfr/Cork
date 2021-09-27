// +-------------------------------------------------------------------------
// | CorkDefs.h
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

#include <cfloat>
#include <cstdint>

#include <gsl/gsl_util>

//  Alignment for platform - 32 bytes is optimal for AVX

constexpr int SIMD_MEMORY_ALIGNMENT = 32;

constexpr int MAX_TRIANGLES_IN_DISJOINT_UNION = 5000000;

//
//  Set flags to control use of AVX2 instruction set
//

#include "SIMDInstructionSet.h"

#ifdef __HAVE_AVX_EXTENSIONS__
#ifdef __HAVE_AVX2_EXTENSIONS__
constexpr SIMDInstructionSet g_SIMD_Level = SIMDInstructionSet::AVX2;
#define __AVX_AVAILABLE__
#define __AVX2_AVAILABLE__
#else
constexpr SIMDInstructionSet g_SIMD_Level = SIMDInstructionSet::AVX;
#define __AVX_AVAILABLE__
#endif
#else
constexpr SIMDInstructionSet g_SIMD_Level = SIMDInstructionSet::NONE;
#endif

//
//  Setup the random number generator
//

constexpr uint64_t RANDOM_SEED = 1ULL;

//
//	Default to double precision, unless CORK_FLOAT_PRECISION is defined
//

#ifdef CORK_FLOAT_PRECISION

#define NUMERIC_PRECISION float

constexpr float NUMERIC_PRECISION_MAX = FLT_MAX;
constexpr float NUMERIC_PRECISION_MIN = FLT_MIN;
constexpr float NUMERIC_PRECISION_EPSILON = FLT_EPSILON;
constexpr float NUMERIC_PRECISION_MIN_EPSILON = (10 * FLT_EPSILON);

constexpr int PURTURBATION_ORDERS_OF_MAG_LESS 4;

constexpr float PURTURBATION_UNDERFLOW_EDGE_LENGTH = (10 * NUMERIC_PRECISION_MIN_EPSILON);

// NOTE: none of these values should be modified by the clients

constexpr int QUANTIZATION_BITS = 30;                       //	number of bits to set resolution of quantizing grid snapped to model
constexpr int PERTURBATION_BUFFER_BITS = 8;                 //	minimum number of bits of resolution that we will reserve when perturbing
                                                            //  the model 8 bits means that we will not perturb the mesh more than
                                                            //  1/128th of the length of the smallest edge in the model
constexpr int PERTURBATION_RANGE_BITS = 5;                  //	number of bits we will perturb over
constexpr int MINIMUM_PERTURBATION_RANGE_BITS = 2;          //	initially perturbation range will be 2^2 or 4

constexpr int FIXED_INTEGER_BITS = QUANTIZATION_BITS + 1;   // +1 for sign bit

#else

//  NOLINTNEXTLINE
#define NUMERIC_PRECISION double

constexpr double NUMERIC_PRECISION_MAX = DBL_MAX;
constexpr double NUMERIC_PRECISION_MIN = DBL_MIN;
constexpr double NUMERIC_PRECISION_EPSILON = DBL_EPSILON;
constexpr double NUMERIC_PRECISION_MIN_EPSILON = (100 * DBL_EPSILON);

constexpr int PURTURBATION_ORDERS_OF_MAG_LESS = 5;

constexpr double PURTURBATION_UNDERFLOW_EDGE_LENGTH = (100 * NUMERIC_PRECISION_MIN_EPSILON);

// NOTE: none of these values should be modified by the clients

constexpr int QUANTIZATION_BITS = 30;                       //	number of bits to set resolution of quantizing grid snapped to model
constexpr int PERTURBATION_BUFFER_BITS = 8;                 //	minimum number of bits of resolution that we will reserve when perturbing
                                                            //  the model 8 bits means that we will not perturb the mesh more than
                                                            //  1/128th of the length of the smallest edge in the model
constexpr int PERTURBATION_RANGE_BITS = 5;                  //	number of bits we will perturb over
constexpr int MINIMUM_PERTURBATION_RANGE_BITS = 2;          //	initially perturbation range will be 2^2 or 4

constexpr int FIXED_INTEGER_BITS = QUANTIZATION_BITS + 1;   // +1 for sign bit

#endif

//
//  Macro to check for memory alignment
//
#ifdef __AVX_AVAILABLE__
//  NOLINTNEXTLINE
#define __CHECK_ALIGNMENT__(x) assert(alignof(x) == SIMD_MEMORY_ALIGNMENT);
#else
//  NOLINTNEXTLINE
#define __CHECK_ALIGNMENT__(x)
#endif
