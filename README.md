Cork - A Computational Geometry Toolkit

This project was forked from Gilbert Berstein's original Cork Computational Geometry source code.  The Cork library provides high performance boolean operations on meshes: Union, Difference, Intersection and Symmetric Difference.

Overall, this library features improved performance for the boolean operations and a growing set of additional functionality, such as automated hole closing and automatic repair of self intersections in meshes.  The automated self intersection repair is far from perfect but it works reasonably well for simple self intersections.  The underlying algorithms and heuristics will be improved over time.

The intent is to produce a high performace library which has solid functionality and is easy to use.

The code builds with the CMake system and is tested extensively on Linux.  It is vanilla C++ 20 so it should be very portable to other OSes.  By default, the code will attempt to use AXV extensions for SIMD operations but this can be turned off for platforms that do not have AVX.  In general, AVX improves performance marginally - it is not a huge boost.  The code is also multithreaded but given the memory layout of the data structures, there is minimal performance improvement when multiple threads are used to parallelize sections of the code.  In general with current generation processors, the memory layout of the internal data structures appears to result in multiple threads colliding over the same cache lines resulting in an increase in waits and an increase in accesses to main memory.  I have worked mainly with AMD CPUs, it is possible Current generation Intel CPUs may perform better but I suspect not materially.

Building the code



