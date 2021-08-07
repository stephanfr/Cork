
include_directories(
  ../thirdparty/boost/boost_1_65_1
  ../thirdparty/tbb/tbb2017_20170807oss/include
  ../thirdparty/mpir/mpir-2.7.2
  )

add_library(Corklib STATIC 
    TriangleMesh.cpp
    aabvh.cpp
    FileFormats/ifs.cpp
    FileFormats/off.cpp
    Intersection/empty3d.cpp
    Intersection/triangle.c
    Mesh/Intersection.cpp
    Mesh/Mesh.cpp
    Mesh/TopoCache.cpp
    Util/log.cpp
    Util/ThreadPool.cpp )
