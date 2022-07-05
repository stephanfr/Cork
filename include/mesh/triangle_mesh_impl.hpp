#pragma once
// +-------------------------------------------------------------------------
// | triangle_mesh_impl.hpp
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

#include "cork.hpp"
#include "mesh/mesh_base.hpp"
#include "mesh/self_intersecting_regions.hpp"

namespace Cork::Meshes
{
    //
    //	TriangleMeshImpl is a straightforward implementation of a triangularized solid expressed as a set
    //		of vertices and triangles defined as 3-tuples of indices into the vertex set.
    //

    class TriangleMeshImpl : public MeshBase
    {
       public:
        TriangleMeshImpl(MeshBase&& mesh_base) : MeshBase(std::move(mesh_base)) {}

        void add_triangle(const TriangleByIndices& triangle_to_add) { tris_->emplace_back(triangle_to_add); }

        void remove_triangle(TriangleByIndicesIndex triangle_index)
        {
            tris_->erase(tris_->begin() + TriangleByIndicesIndex::integer_type(triangle_index));
        }

        [[nodiscard]] Statistics::GeometricStatistics ComputeGeometricStatistics(
            Statistics::GeometricProperties props_to_compute) const;

        [[nodiscard]] Statistics::TopologicalStatisticsResult ComputeTopologicalStatistics(
            Statistics::TopologicalProperties props_to_compute) const;

        void remove_non_manifold_edges(const Statistics::TopologicalStatistics& topo_stats);

        SelfIntersectionResolutionResults remove_self_intersections(
            const Statistics::TopologicalStatistics& topo_stats);

        SelfIntersectingRegions find_self_intersecting_regions(
            const Statistics::TopologicalStatistics& topo_stats);

        bool resolves_self_intersection(const TriangleByIndicesIndexSet& tris_to_remove);

        HoleClosingResult close_holes(const Statistics::TopologicalStatistics& topo_stats);

        TriangleByIndicesVector as_triangles(const TriangleByIndicesIndexSet& indices)
        {
            TriangleByIndicesVector result;

            for (TriangleByIndicesIndex current_index : indices)
            {
                result.emplace_back((*tris_)[current_index]);
            }

            return result;
        }
    };

}  // namespace Cork::Meshes