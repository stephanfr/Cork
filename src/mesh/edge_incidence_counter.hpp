// +-------------------------------------------------------------------------
// | edge_incidence_counter.hpp
// |
// | Author: Stephan Friedl
// +-------------------------------------------------------------------------
// | COPYRIGHT:
// |    Copyright Stephan Friedl 2021
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

#include "../constants.hpp"

#include "primitives/edge_and_incidence_count.hpp"

#include "mesh_base.hpp"


namespace Cork::Meshes
{
    class EdgeIncidenceCounter
    {
       public:
        EdgeIncidenceCounter() = delete;
        EdgeIncidenceCounter( const EdgeIncidenceCounter& ) = delete;
        EdgeIncidenceCounter( EdgeIncidenceCounter&& ) = delete;

        explicit EdgeIncidenceCounter(const Cork::Meshes::MeshBase& triangle_mesh)
        {
            edges_and_incidences_.reserve((triangle_mesh.num_triangles() * 3) + 10);  //  Pad just a little bit

            for (TriangleByIndicesIndex i = 0U; i < triangle_mesh.triangles().size(); i++)
            {
                add_incidence( i, triangle_mesh.triangles()[i] );
            }
        }

        EdgeIncidenceCounter(const MeshBase& mesh, const TriangleByIndicesIndexVector& tris_in_region)
        {
            edges_and_incidences_.reserve((tris_in_region.size() * 3) + 10);  //  Pad just a little bit

            for ( auto tri_index : tris_in_region )
            {
                add_incidence( tri_index, mesh.triangles()[tri_index] );
            }
        }

        EdgeIncidenceCounter(const MeshBase& mesh, const TriangleByIndicesIndexSet& tris_in_region)
        {
            edges_and_incidences_.reserve((tris_in_region.size() * 3) + 10);  //  Pad just a little bit

            for ( auto tri_index : tris_in_region )
            {
                add_incidence( tri_index, mesh.triangles()[tri_index] );
            }
        }

        ~EdgeIncidenceCounter() = default;

        EdgeIncidenceCounter& operator=( const EdgeIncidenceCounter& ) = delete;
        EdgeIncidenceCounter& operator=( EdgeIncidenceCounter&& ) = delete;

        [[nodiscard]] const EdgeIncidenceSet& edges_and_incidences() const { return edges_and_incidences_; }

       private:
        EdgeIncidenceSet edges_and_incidences_;

        void add_incidence( TriangleByIndicesIndex i, const TriangleByIndices& tri )
        {
            auto itr_edge_ab = edges_and_incidences_.emplace(tri.a(), tri.b()).first;
            auto itr_edge_bc = edges_and_incidences_.emplace(tri.b(), tri.c()).first;
            auto itr_edge_ca = edges_and_incidences_.emplace(tri.c(), tri.a()).first;

            //  Const casts are used here to deal with the fact that iterators into sets are const by definition.
            //      This is safe only because the key for the set is the edge and we are conly changing the
            //      triangle list and num_incidences in add_incidence() - so we know we are not changing the key.

            const_cast<EdgeAndIncidenceCount&>(*itr_edge_ab).add_incidence(i, TriangleEdgeId::AB);
            const_cast<EdgeAndIncidenceCount&>(*itr_edge_bc).add_incidence(i, TriangleEdgeId::BC);
            const_cast<EdgeAndIncidenceCount&>(*itr_edge_ca).add_incidence(i, TriangleEdgeId::CA);
        }
    };
}  // namespace Cork::Meshes
