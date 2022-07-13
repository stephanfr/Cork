#pragma once
// +-------------------------------------------------------------------------
// | triangle_mesh_wrapper.hpp
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

#include "triangle_mesh_impl.hpp"

namespace Cork::Meshes
{
    class TriangleMeshWrapper : public TriangleMesh
    {
       public:
        using GeometricStatistics = Statistics::GeometricStatistics;
        using GeometricProperties = Statistics::GeometricProperties;
        using TopologicalStatistics = Statistics::TopologicalStatistics;
        using TopologicalProperties = Statistics::TopologicalProperties;

        //  Constructor/Destructor

        TriangleMeshWrapper(MeshBase&& mesh_base) : mesh_(new TriangleMeshImpl(std::move(mesh_base))) {}

        ~TriangleMeshWrapper(){};

        TriangleMeshImpl& implementation() { return *mesh_; }

        //	Methods follow

        size_t num_triangles() const { return mesh_->num_triangles(); }
        size_t num_vertices() const { return mesh_->num_vertices(); }

        const Vertex3DVector& vertices() const { return mesh_->vertices(); }
        const TriangleByIndicesVector& triangles() const { return mesh_->triangles(); }

        TriangleByVertices triangle_by_vertices(const TriangleByIndices& triangle_by_indices) const
        {
            return mesh_->triangle_by_vertices(triangle_by_indices);
        }

        virtual std::unique_ptr<TriangleMesh> extract_surface(TriangleByIndicesIndex center_triangle,
                                                              uint32_t num_rings)
        {
            Cork::Primitives::TriangleByIndicesIndexSet single_triangle;

            single_triangle.emplace(center_triangle);

            TriangleRemapper        remapper( *mesh_ );

            auto result = mesh_->find_enclosing_triangles(single_triangle, num_rings);

            if( !result.succeeded() )       //  TODO return proper success/failure
            {
                return std::unique_ptr<TriangleMesh>();
            }

            return std::make_unique<TriangleMeshWrapper>( std::move( *(mesh_->extract_surface( remapper,
                result.return_ptr()->merge(single_triangle)))));
        }

        const BBox3D& bounding_box() const { return mesh_->bounding_box(); }
        MinAndMaxEdgeLengths min_and_max_edge_lengths() const { return mesh_->min_and_max_edge_lengths(); }
        double max_vertex_magnitude() const { return mesh_->max_vertex_magnitude(); }

        GeometricStatistics ComputeGeometricStatistics(GeometricProperties props_to_compute) const
        {
            return mesh_->ComputeGeometricStatistics(props_to_compute);
        }

        TopologicalStatisticsResult ComputeTopologicalStatistics(TopologicalProperties props_to_compute) const
        {
            return mesh_->ComputeTopologicalStatistics(props_to_compute);
        }

        HoleClosingResult close_holes(const TopologicalStatistics& topo_stats)
        {
            return mesh_->close_holes(topo_stats);
        }

        SelfIntersectionResolutionResults remove_self_intersections(const TopologicalStatistics& topo_stats)
        {
            return mesh_->remove_self_intersections(topo_stats);
        }

        void remove_non_manifold_edges(const Statistics::TopologicalStatistics& topo_stats)
        {
            return mesh_->remove_non_manifold_edges(topo_stats);
        }

       private:
        std::unique_ptr<TriangleMeshImpl> mesh_;
    };
}  // namespace Cork::Meshes
