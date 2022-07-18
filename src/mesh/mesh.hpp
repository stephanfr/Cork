// +-------------------------------------------------------------------------
// | mesh.hpp
// |
// | Author: Gilbert Bernstein
// +-------------------------------------------------------------------------
// | COPYRIGHT:
// |    Copyright Gilbert Bernstein 2013
// |    See the included COPYRIGHT file for further details.
// |
// |    This file is part of the Cork library.
// |
// |    Cork is free software: you can redistribute it and/or modify
// |    it under the terms of the GNU Lesser General Public License as
// |    published by the Free Software Foundation, either version 3 of
// |    the License, or (at your option) any later version.
// |
// |    Cork is distributed in the hope that it will be useful,
// |    but WITHOUT ANY WARRANTY; without even the implied warranty of
// |    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// |    GNU Lesser General Public License for more details.
// |
// |    You should have received a copy
// |    of the GNU Lesser General Public License
// |    along with Cork.  If not, see <http://www.gnu.org/licenses/>.
// +-------------------------------------------------------------------------
#pragma once

#include "edge_graph_cache.hpp"
#include "mesh_base.hpp"
#include "solver_perf_stats.hpp"
#include "tbb/concurrent_vector.h"

namespace Cork::Meshes
{
    //
    //	The Mesh class brings together the functionality needed for the boolean operations
    //

    class Mesh : public MeshBase, public SolidObjectMesh
    {
       public:
        Mesh() = delete;

        Mesh(Mesh&& src)
            : MeshBase(std::move(src)), control_block_(SolverControlBlock::get_default_control_block()){};

        Mesh(MeshBase&& src, const SolverControlBlock& control_block)
            : MeshBase(std::move(src)), control_block_(control_block){};

        explicit Mesh(const TriangleMesh& inputMesh, const SolverControlBlock& controlBlock);

        virtual ~Mesh();

        Mesh& operator=(Mesh&& src) noexcept;

        //	Validity check:
        //		- all numbers are well-defined and finite
        //		- all triangle vertex indices are in the right range

        bool valid() const;

        // form the disjoint union of two meshes

        void DisjointUnion(const Mesh& meshToMerge);

        //	Boolean operations

        // NOLINTBEGIN(google-default-arguments)

        BooleanOperationResult Union(const SolidObjectMesh& rhs,
                                     const SolverControlBlock& solverControlBlock) const final;
        BooleanOperationResult Difference(const SolidObjectMesh& rhs,
                                          const SolverControlBlock& solverControlBlock) const final;
        BooleanOperationResult Intersection(const SolidObjectMesh& rhs,
                                            const SolverControlBlock& solverControlBlock) const final;
        BooleanOperationResult SymmetricDifference(const SolidObjectMesh& rhs,
                                                   const SolverControlBlock& solverControlBlock) const final;

        // NOLINTEND(google-default-arguments)

        std::unique_ptr<TriangleMesh> ToTriangleMesh() const;

        const SolverControlBlock& solver_control_block() const { return control_block_; }
        const SolverPerformanceStatistics& GetPerformanceStats() const final { return performance_stats_; }

        size_t CountComponents() const final;

       private:
        SolverControlBlock control_block_;
        SolverPerfStats performance_stats_;

        enum class TriCode
        {
            KEEP_TRI,
            DELETE_TRI,
            FLIP_TRI
        };

        using SetupBooleanProblemResult = SEFUtility::Result<SetupBooleanProblemResultCodes>;

        using BuildEGraphCacheResult = SEFUtility::ResultWithReturnUniquePtr<BuildEGraphCacheResultCodes, EGraphCache>;

        using ComponentType = tbb::concurrent_vector<TriangleByIndicesIndex>;
        using ComponentList = tbb::concurrent_vector<ComponentType>;

        SetupBooleanProblemResult SetupBooleanProblem(const Mesh& rhs);

        BuildEGraphCacheResult BuildEdgeGraphCache() const;

        std::unique_ptr<ComponentList> FindComponents(EGraphCache& ecache) const;

        void ProcessComponent(const EGraphCache& ecache, const ComponentType& trisInComponent);

        TriangleByIndicesIndex FindTriForInsideTest(const ComponentType& trisInComponent);

        void doDeleteAndFlip(const std::function<TriCode(uint32_t bool_alg_data)>& classify);

        void for_ecache(EGraphCache& ecache, std::function<void(const EGraphEntryTIDVector& tids)> action,
                        int numThreads = 1) const;

        bool isInside(TriangleByIndicesIndex tid, uint32_t operand);

        void RayTriangleIntersection(const TriangleByIndices& tri, Primitives::Ray3D& ray, long& winding);
    };
}  // namespace Cork::Meshes
