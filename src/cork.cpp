#include "util/ThreadPool.h"

#include "cork.hpp"
#include "mesh/mesh.h"

namespace Cork
{
    CorkService::~CorkService() { ThreadPool::getPool().Shutdown(); }

    std::unique_ptr<CorkMesh> CorkService::from_triangle_mesh(const TriangleMesh& triangleMesh)
    {
        return (std::unique_ptr<CorkMesh>(new Meshes::Mesh(triangleMesh,CorkService::get_default_control_block())));
    }

    const SolverControlBlock& CorkService::get_default_control_block()
    {
        static SolverControlBlock defaultBlock(true, (long)50000, true);

        return (defaultBlock);
    }

}  // namespace Cork
