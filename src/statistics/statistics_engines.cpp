// +-------------------------------------------------------------------------
// | statistics_engines.cpp
// |
// | Author: Stephan Friedl
// +-------------------------------------------------------------------------
// | COPYRIGHT:
// |    Copyright Stephan Friedl 2015
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

#include "statistics_engines.hpp"

#include "intersection/self_intersection_finder.hpp"
#include "mesh/boundary_edge_builder.hpp"

namespace Cork::Statistics
{
    using MeshBase = Meshes::MeshBase;

    using BoundaryEdgeBuilder = Meshes::BoundaryEdgeBuilder;

    GeometricStatisticsEngine::GeometricStatisticsEngine(const Meshes::MeshBase& triangle_mesh,
                                                         GeometricProperties properties_to_compute)
        : num_triangles_(triangle_mesh.num_triangles()),
          num_vertices_(triangle_mesh.num_vertices()),
          area_(0.0),
          volume_(0.0),
          min_edge_length_(DBL_MAX),
          max_edge_length_(DBL_MIN),
          bounding_box_(triangle_mesh.bounding_box())
    {
        for (const auto& currentTriangle : triangle_mesh.triangles())
        {
            auto tri_by_verts = triangle_mesh.triangle_by_vertices(currentTriangle);

            //	Get the edges

            if ((properties_to_compute &
                 (GeometricProperties::GEOM_AREA_AND_VOLUME | GeometricProperties::GEOM_EDGE_LENGTHS)) != 0)
            {
                Vector3D edgeAB = tri_by_verts.edgeAB_from_origin();
                Vector3D edgeAC = tri_by_verts.edgeAC_from_origin();
                Vector3D edgeBC = tri_by_verts.edgeBC_from_origin();

                if ((properties_to_compute & GeometricProperties::GEOM_AREA_AND_VOLUME) != 0)
                {
                    //	Add the incremental area of this triangle

                    Vector3D ABcrossAC = edgeAB.cross(edgeAC);

                    area_ += ABcrossAC.len() / 2;

                    //	Do the same for volume

                    double temp = tri_by_verts.vertexA().dot(ABcrossAC);

                    volume_ += temp / 6;
                }

                //	Update the min/max edge lengths

                if ((properties_to_compute & GeometricProperties::GEOM_EDGE_LENGTHS) != 0)
                {
                    min_edge_length_ = std::min(min_edge_length_,
                                                (double)std::min(edgeAB.len(), std::min(edgeAC.len(), edgeBC.len())));
                    max_edge_length_ = std::max(max_edge_length_,
                                                (double)std::max(edgeAB.len(), std::max(edgeAC.len(), edgeBC.len())));
                }
            }
        }
    }

    TopologicalStatisticsEngine::TopologicalStatisticsEngine(const MeshBase& triangle_mesh)
        : triangle_mesh_(triangle_mesh), edges_(triangle_mesh)
    {
    }

    TopologicalStatisticsEngineAnalyzeResult TopologicalStatisticsEngine::Analyze(
        TopologicalProperties props_to_compute) const
    {
        //  First, look for non 2 manifold edges

        int num_non_2_manifold = 0;
        int num_edges = 0;

        std::vector<NonManifoldEdge> non_manifold_edges;
        std::vector<BoundaryEdge> holes;
        std::vector<SelfIntersectingEdge> si_stats;

        if (props_to_compute & TopologicalProperties::TOPO_BASE)
        {
            for (const auto& edge : edges_.edges_and_incidences())
            {
                if (edge.numIncidences() != 2)
                {
                    non_manifold_edges.emplace_back(
                        NonManifoldEdge(edge.triangles().front().first, edge.triangles().front().second));
                }
            }

            num_edges = edges_.edges_and_incidences().size();
        }

        if (props_to_compute & TopologicalProperties::TOPO_HOLES)
        {
            std::vector<Primitives::EdgeByIndices> hole_edges;

            for (const auto& edge : edges_.edges_and_incidences())
            {
                if (edge.numIncidences() != 2)
                {
                    if (edge.numIncidences() <= 1)
                    {
                        hole_edges.push_back(dynamic_cast<const Primitives::EdgeByIndices&>(edge));
                    }
                }
            }

            auto extract_boundaries_result =
                BoundaryEdgeBuilder(triangle_mesh_)
                    .extract_boundaries(hole_edges);  //  TODO fix this - maybe success/failure

            if (extract_boundaries_result.succeeded())
            {
                holes = *(extract_boundaries_result.return_ptr());
            }
        }

        if (props_to_compute & TopologicalProperties::TOPO_SELF_INTERSECTIONS)
        {
            Intersection::SelfIntersectionFinder se_finder(triangle_mesh_.topo_cache());

            si_stats = se_finder.CheckSelfIntersection();
        }

        return TopologicalStatistics(num_edges, 0, non_manifold_edges, holes, si_stats);
    }

};  // namespace Cork::Statistics
