#include "cork.hpp"

#include "constants.hpp"
#include "mesh/mesh.hpp"


namespace Cork
{

    CorkService::~CorkService() {}

    std::unique_ptr<SolidObjectMesh> CorkService::from_triangle_mesh(const TriangleMesh& triangleMesh)
    {
        return (std::unique_ptr<SolidObjectMesh>(
            new Meshes::Mesh(triangleMesh, SolverControlBlock::get_default_control_block())));
    }

    const SolverControlBlock& SolverControlBlock::get_default_control_block()
    {
        static SolverControlBlock defaultBlock(true, MIN_TRIANGLES_FOR_MULTITHREADING, true);

        return (defaultBlock);
    }

}  // namespace Cork
