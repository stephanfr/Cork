// +-------------------------------------------------------------------------
// | IntersectionProblem.h
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

#include "mesh/mesh_base.hpp"

namespace Cork::Intersection
{
    enum class IntersectionProblemResultCodes
    {
        SUCCESS = 0,
        OUT_OF_MEMORY,
        SUBDIVIDE_FAILED,
        EXHAUSTED_PURTURBATION_RETRIES,
        SELF_INTERSECTING_MESH,
        CONSOLIDATE_FAILED
    };

    using IntersectionProblemResult = SEFUtility::Result<IntersectionProblemResultCodes>;

    class IntersectionSolver
    {
       public:
        static std::unique_ptr<IntersectionSolver> GetSolver(Meshes::MeshBase& owner, const Math::Quantizer& quantizer, const SolverControlBlock& solver_control_block);

        virtual ~IntersectionSolver() {}

        virtual IntersectionProblemResult FindIntersections() = 0;
        virtual IntersectionProblemResult ResolveAllIntersections() = 0;

        virtual void commit() = 0;
    };

}  // namespace Cork::Intersection
