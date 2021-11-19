
FetchContent_Declare(
  Xoshiro256PlusSIMD
  GIT_REPOSITORY https://github.com/stephanfr/Xoshiro256PlusSIMD.git
  GIT_TAG        main)

FetchContent_MakeAvailable(Xoshiro256PlusSIMD)

FetchContent_Declare(
  fmtlib
  GIT_REPOSITORY https://github.com/fmtlib/fmt.git
  GIT_TAG        8.0.1)

FetchContent_MakeAvailable(fmtlib)

include_directories(
    .
    ../include
    ../thirdparty/tbb/${TBB_VERSION}/include
    ../thirdparty/mpir/${MPIR_VERSION}
    ${xoshiro256plussimd_SOURCE_DIR}/include
    ${fmtlib_SOURCE_DIR}/include
)

add_library(Corklib SHARED 
    Accel/aabvh.cpp
#    FileFormats/ifs.cpp
    FileFormats/off.cpp
    Intersection/empty3d.cpp
    primitives/streaming_operators.cpp
    Intersection/intersection_problem_base.cpp
    Intersection/perturbation_epsilon.cpp
    Intersection/triangle.c
    Intersection/triangulator.cpp
    Intersection/intersection.cpp
    Intersection/self_intersection_finder.cpp
    Mesh/mesh.cpp
    Mesh/topo_cache.cpp
    Mesh/triangle_mesh.cpp
    statistics/statistics_engines.cpp
    util/thread_pool.cpp
    cork.cpp
)

target_link_libraries(Corklib 
  PRIVATE tbb
  PRIVATE mpir )