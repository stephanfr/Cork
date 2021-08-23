
FetchContent_Declare(
  Xoshiro256PlusSIMD
  GIT_REPOSITORY https://github.com/stephanfr/Xoshiro256PlusSIMD.git
  GIT_TAG        main)

FetchContent_MakeAvailable(Xoshiro256PlusSIMD)

include_directories(
    .
    ../include
    ../thirdparty/tbb/${TBB_VERSION}/include
    ../thirdparty/mpir/${MPIR_VERSION}
    ${xoshiro256plussimd_SOURCE_DIR}/include
)

add_library(Corklib SHARED 
    Accel/aabvh.cpp
    FileFormats/ifs.cpp
    FileFormats/off.cpp
    Intersection/empty3d.cpp
    Intersection/triangle.c
    Mesh/Intersection.cpp
    Mesh/Mesh.cpp
    Mesh/TopoCache.cpp
    Mesh/TriangleMesh.cpp
    util/ThreadPool.cpp
)

target_link_libraries(Corklib 
  PRIVATE tbb
  PRIVATE mpir )