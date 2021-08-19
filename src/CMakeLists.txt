
include_directories(
    .
    ../include
  )
  
  
add_library(Corklib STATIC 
    Accel/aabvh.cpp
    FileFormats/ifs.cpp
    FileFormats/off.cpp
    Intersection/empty3d.cpp
    Intersection/triangle.c
    Mesh/Intersection.cpp
    Mesh/Mesh.cpp
    Mesh/TopoCache.cpp
    Mesh/TriangleMesh.cpp
    util/log.cpp
    util/ThreadPool.cpp
)