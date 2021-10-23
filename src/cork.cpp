#include "util/ThreadPool.h"

#include "cork.h"
#include "mesh/mesh.h"

namespace Cork
{
    CorkService::~CorkService() { ThreadPool::getPool().Shutdown(); }

    std::unique_ptr<CorkMesh> CorkService::FromTriangleMesh(const TriangleMesh& triangleMesh)
    {
        return (std::unique_ptr<CorkMesh>(new Mesh(triangleMesh,CorkService::GetDefaultControlBlock())));
    }

    const SolverControlBlock& CorkService::GetDefaultControlBlock()
    {
        static SolverControlBlock defaultBlock(true, (long)50000, true);

        return (defaultBlock);
    }

}  // namespace Cork
