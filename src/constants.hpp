#pragma once

#include <cstddef>
#include <stdint.h>

//
//  Threading control
//

constexpr uint32_t MIN_TRIANGLES_FOR_MULTITHREADING = 50000;


//
//  Object pool initial sizes
//

constexpr size_t GLUE_POINT_MARKER_POOL_INITIAL_SIZE = 100000;
constexpr size_t GENERIC_VERTEX_POOL_INITIAL_SIZE = 200000;
constexpr size_t GENERIC_EDGE_POOL_INITIAL_SIZE = 500000;
constexpr size_t GENERIC_TRIANGLE_POOL_INITIAL_SIZE = 100000;
constexpr size_t INTERSECTION_VERTEX_POINTER_POOL_INITIAL_SIZE = 200000;
constexpr size_t INTERSECTION_EDGE_POINTER_POOL_INITIAL_SIZE = 100000;
constexpr size_t GENERIC_TRIANGLE_POINTER_POOL_INITIAL_SIZE = 100000;

//
//  Controls the number of permutations attempted for the PerturbationRandomizationMatrix
//

constexpr size_t MIN_NUM_PERMUTATIONS = 4;
constexpr size_t MAX_NUM_PERMUTATIONS = 32;

//
//  Triangulation constants
//

constexpr size_t MAX_TRIANGULATION_POINTS = 2048;

//
//  Edge Graph Cache Sizes
//

constexpr size_t EGRAPH_CACHE_SKELETON_COLUMN_INITIAL_SIZE = 10;
constexpr size_t EGRAPH_ENTRY_TIDS_VEC_LENGTH = 8;

//
//  Edge and Incidence Counter Sizes
//

constexpr size_t EDGE_AND_INCIDENCE_COUNT_NUM_TRIANGLEs_INITIAL_SIZE = 6;

//
//  Topo Cache sizes
//

constexpr size_t TOPO_CACHE_INITIAL_TRIANGLE_POINTER_VECTOR_SIZE = 6;
constexpr size_t TOPO_CACHE_INITIAL_EDGE_POINTER_VECTOR_SIZE = 12;
