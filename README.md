# Cork - A Computational Geometry Toolkit

This project was forked from Gilbert Berstein's original Cork Computational Geometry source code.  The Cork library provides high performance boolean operations on meshes: Union, Difference, Intersection and Symmetric Difference.

Overall, this library features improved performance for the boolean operations and a growing set of additional functionality, such as automated hole closing and automatic repair of self intersections in meshes.  The automated self intersection repair is far from perfect but it works reasonably well for simple self intersections.  The underlying algorithms and heuristics will be improved over time.

The intent is to produce a high performace library which has solid functionality and is easy to use.

The code builds with the CMake system and is tested extensively on Linux.  It is vanilla C++ 20 so it should be very portable to other OSes.  By default, the code will attempt to use AXV extensions for SIMD operations but this can be turned off for platforms that do not have AVX.  In general, AVX improves performance marginally - it is not a huge boost.  The code is also multithreaded but given the memory layout of the data structures, there is minimal performance improvement when multiple threads are used to parallelize sections of the code.  In general with current generation processors, the memory layout of the internal data structures appears to result in multiple threads colliding over the same cache lines resulting in an increase in waits and an increase in accesses to main memory.  I have worked mainly with AMD CPUs, it is possible Current generation Intel CPUs may demonstrate better performance.


## Building the code

Cork uses the C++20 standard and requires the std::span<> template which was introduced in gcc version 10, therefore to compile GCC V10 or better is required. 

After cloning the repository, run the setup_thirdparty.sh script in the root of the repository.  This script will download and install a number of dependencies, such as the Boost libraries and Intel's Threading Building Blocks.

After that, CMake configure and make should be all that is needed.

The project is setup for the VS Code IDE, and .clang-tidy as well as .clang-format config files can also be found in the root directory.  VS Code will automatically pickup these files for Code Analysis and Formatting.


## CLI and Tests

The RegressionTest folder contains an executable, CorkCLI, which is useful for regression testing the library.  CLI options appear below:

```
build/RegressionTest/CorkCLI --help
Allowed options:
  --help                 produce help message
  --repair               Repair Input Meshes
  --union                Compute Boolean Union
  --intersection         Compute Boolean Intersection
  --difference           Compute Boolean Difference
  --xor                  Compute Boolean Exclusive Or
  --input-directory arg  Directory containing input meshes
  --output-directory arg Directory to receive output files
  --write-results        Write result meshes
  --write-statistics     Write Statistics
```

If the *--input-directory* option is used, then all of the solid model files in the specified directory will be loaded and all of the boolean operations specified will be run on all combinations of the solid objects.  For example, if three .OFF files are present in the input directory, then a total of six permutations will be computed for each boolean operation specified.  If *--write-results* is specified, then the result of each boolean operation will be written as a .OFF file in the results directory.  The output filename will be of the form: *firstmeshname_secondmeshname_booleanoperation*.OFF.

If *--write statistics* is specified, then a 'stats' subdirectory must be created under the results directory.  A set of timing and geotopo metrics will be written into the 'stats' directory.  Since the code uses perturbation with random values, the mesh produced from a pair of input meshes will not necessarily be the same every time.  Depending on the geometry, it is possible that a slightly different number of triangles will be created as a result of the boolean ops.

The *--repair* option attempts to fix meshes with holes or self intersections.  In general, self intersections will cause the CSG operations in general to fail and this includes Cork.  The repair operation should be considered experimental at present but for simple to moderately complex self intersections, it appears to work reasonably well.  Cork will usually be able to compute a result mesh from input meshes with self intersections - if the self intersections are not in the regions where the two models intersect.  In general, the code will determine if an operation has failed as a result of a self intersecting input mesh and provide an accurate error message.

## Test Solid Models

The https://github.com/stephanfr/SolidModelRepository.git repository contains a collection of solid models that can be used with Cork.  Some of these models contain self intersections and will fail to be unioned for example.  The repair option will successfully repair some of the models, but not all.

An example CLI command which will compute the union of all pairwise combinations of meshes in the 'TestSet' directory and which will write the timing and geotopo statistics but not the result meshes themselves appears below:

```
build/RegressionTest/CorkCLI --union --input-directory RegressionTest/TestSet/ --output-directory RegressionTest/Results/ --write-statistics
```

## Unit Tests and Benchmarks

The UnitTest directory contains a collection of unit tests and benchmarks which rely on the Catch2 framework.

## Valgrind

```
valgrind --tool=callgrind --verbose --log-file=callgrind-out.txt /home/steve/dev/Cork/build/RegressionTest/CorkCLI --union --input-directory /home/steve/dev/Cork/RegressionTest/PerfTestSet --output-directory /home/steve/dev/Cork/RegressionTest/PerfResults/

valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes --verbose --log-file=valgrind-out.txt /home/steve/dev/Cork/build/RegressionTest/CorkCLI --union --input-directory /home/steve/dev/Cork/RegressionTest/PerfTestSet --output-directory /home/steve/dev/Cork/RegressionTest/PerfResults/ --repair
```

## Math Worksheets

In support of some of the mathematical operations in Cork and to prove the correctness of the code, the Documents directory contains a collection of Maple worksheets for a selection of Cork internal operations.  These operations are checked by unit tests and the Maple results are used to confirm the correct values are being computed by the code.  For those interested in the math behind computing intersections in Cork - the worksheets provide that insight as well as examples of exterior calculus used in Cork.

