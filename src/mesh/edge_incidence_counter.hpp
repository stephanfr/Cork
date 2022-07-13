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

#include <boost/container/small_vector.hpp>
#include <unordered_set>

#include "mesh_base.hpp"

namespace Cork::Meshes
{
    class EdgeAndIncidenceCount : public Primitives::EdgeByIndices      //  TODO make this standalone and move to separate hpp file
    {
       public:
        EdgeAndIncidenceCount(const Primitives::VertexIndex a, const Primitives::VertexIndex b)
            : Primitives::EdgeByIndices(a, b), num_incidences_(0)
        {
        }

        virtual ~EdgeAndIncidenceCount() {}

        int AddIncidence(TriangleByIndicesIndex tri_index, TriangleEdgeId edge_id)
        {
            triangles_.emplace_back(std::make_pair(tri_index, edge_id));
            return ++num_incidences_;
        }

        int numIncidences() const { return (num_incidences_); }

        const boost::container::small_vector<std::pair<TriangleByIndicesIndex, TriangleEdgeId>, 6> triangles() const
        {
            return triangles_;
        }

       private:
        int num_incidences_;
        boost::container::small_vector<std::pair<TriangleByIndicesIndex, TriangleEdgeId>, 6> triangles_;
    };

    class EdgeIncidenceSet : public std::unordered_set<EdgeAndIncidenceCount, EdgeAndIncidenceCount::HashFunction> {};

    class EdgeIncidenceCounter
    {
       public:
        EdgeIncidenceCounter(const Cork::Meshes::MeshBase& triangle_mesh)
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

        const EdgeIncidenceSet& edges_and_incidences() const { return edges_and_incidences_; }

       private:
        EdgeIncidenceSet edges_and_incidences_;

        void add_incidence( TriangleByIndicesIndex i, const TriangleByIndices& tri )
        {
            EdgeIncidenceSet::iterator itrEdgeAB = edges_and_incidences_.emplace(tri.a(), tri.b()).first;
            EdgeIncidenceSet::iterator itrEdgeBC = edges_and_incidences_.emplace(tri.b(), tri.c()).first;
            EdgeIncidenceSet::iterator itrEdgeCA = edges_and_incidences_.emplace(tri.c(), tri.a()).first;

            const_cast<EdgeAndIncidenceCount&>(*itrEdgeAB).AddIncidence(i, TriangleEdgeId::AB);
            const_cast<EdgeAndIncidenceCount&>(*itrEdgeBC).AddIncidence(i, TriangleEdgeId::BC);
            const_cast<EdgeAndIncidenceCount&>(*itrEdgeCA).AddIncidence(i, TriangleEdgeId::CA);
        }
    };
}  // namespace Cork::Meshes
