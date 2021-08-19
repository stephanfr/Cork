
include_directories(
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
