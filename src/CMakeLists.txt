
FetchContent_Declare(
  EigenTemplateLibrary
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG        3.4.0)

FetchContent_MakeAvailable(EigenTemplateLibrary)

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
    ${eigentemplatelibrary_SOURCE_DIR}
    ${redsvdtemplatelibrary_SOURCE_DIR}/include
)

add_library(Corklib SHARED 
    primitives/boundary_edge.cpp
    primitives/streaming_operators.cpp
    2d_geometry/polygon.cpp
    accel/aabvh.cpp
#    file_formats/ifs.cpp
    file_formats/off.cpp
    file_formats/polyline3d.cpp
    intersection/empty3d.cpp
    intersection/intersection_problem_base.cpp
    intersection/perturbation_epsilon.cpp
    intersection/triangle.c
    intersection/triangulator.cpp
    intersection/intersection.cpp
    intersection/self_intersection_finder.cpp
    math/plane.cpp
    mesh/mesh.cpp
    mesh/surface_mesh.cpp
    mesh/topo_cache.cpp
    mesh/triangle_mesh_impl.cpp
    mesh/triangle_mesh_wrapper.cpp
    mesh/mesh_base.cpp
    mesh/mesh_builder.cpp
    mesh/boundary_edge_builder.cpp
    mesh/triangle_remapper.cpp
    mesh/self_intersecting_regions.cpp
    statistics/statistics_engines.cpp
    util/thread_pool.cpp
    cork.cpp
)

target_link_libraries(Corklib 
  PRIVATE tbb
  PRIVATE mpir )