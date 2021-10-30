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

#include "statistics/statistics_engines.h"

#include <boost/container/small_vector.hpp>
#include <unordered_set>

#include "CorkDefs.h"
#include "cork.h"
#include "intersection/intersection_problem.hpp"
#include "math/quantization.hpp"
#include "mesh/mesh.h"

namespace Cork::Statistics
{
    using Vector3D = Primitives::Vector3D;

    using TriangleByVertices = Primitives::TriangleByVertices;

    GeometricStatisticsEngine::GeometricStatisticsEngine(const TriangleMesh& triangle_mesh,
                                                         GeometricProperties properties_to_compute)
        : num_triangles_(triangle_mesh.numTriangles()),
          num_vertices_(triangle_mesh.numVertices()),
          area_(0.0),
          volume_(0.0),
          min_edge_length_(DBL_MAX),
          max_edge_length_(DBL_MIN),
          bounding_box_(triangle_mesh.boundingBox())
    {
        for (const auto& currentTriangle : triangle_mesh.triangles())
        {
            auto tri_by_verts = triangle_mesh.triangleByVertices(currentTriangle);

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

    TopologicalStatisticsEngine::TopologicalStatisticsEngine(const TriangleMesh& triangle_mesh)
        : triangle_mesh_(triangle_mesh)
    {
        edges_.reserve((triangle_mesh.numTriangles() * 6) + 10);  //  Pad just a little bit

        for (const auto& current_triangle : triangle_mesh_.triangles())
        {
            EdgeSet::iterator itrEdgeAB = edges_.emplace(current_triangle.a(), current_triangle.b()).first;
            EdgeSet::iterator itrEdgeAC = edges_.emplace(current_triangle.a(), current_triangle.c()).first;
            EdgeSet::iterator itrEdgeBC = edges_.emplace(current_triangle.b(), current_triangle.c()).first;

            const_cast<EdgeAndIncidence&>(*itrEdgeAB).AddIncidence();
            const_cast<EdgeAndIncidence&>(*itrEdgeAC).AddIncidence();
            const_cast<EdgeAndIncidence&>(*itrEdgeBC).AddIncidence();
        }
    }

    TopologicalStatisticsEngineAnalyzeResult TopologicalStatisticsEngine::Analyze(
        TopologicalProperties props_to_compute)
    {
        //  First, look for non 2 manifold edges

        int num_non_2_manifold = 0;
        int num_edges = 0;
        std::vector<Hole> holes;
        std::vector<IntersectionInfo> si_stats;

        if (props_to_compute & TopologicalProperties::TOPO_BASE)
        {
            for (auto& edge : edges_)
            {
                if (edge.numIncidences() != 2)
                {
                    num_non_2_manifold++;
                }
            }

            num_edges = edges_.size();
        }

        if (props_to_compute & TopologicalProperties::TOPO_HOLES)
        {
            std::vector<Primitives::EdgeByIndices> hole_edges;

            for (auto& edge : edges_)
            {
                if (edge.numIncidences() != 2)
                {
                    if (edge.numIncidences() <= 1)
                    {
                        hole_edges.push_back(dynamic_cast<const Primitives::EdgeByIndices&>(edge));
                    }
                }
            }
            holes = HoleBuilder::extract_holes(hole_edges);
        }

        if (props_to_compute & TopologicalProperties::TOPO_SELF_INTERSECTIONS)
        {
            std::unique_ptr<Meshes::Mesh> single_mesh(
                new Meshes::Mesh(triangle_mesh_, CorkService::get_default_control_block()));

            Math::Quantizer::GetQuantizerResult get_quantizer_result = single_mesh->getQuantizer();

            if (!get_quantizer_result.succeeded())
            {
                return TopologicalStatisticsEngineAnalyzeResult::failure(
                    TopologicalStatisticsEngineAnalyzeResultCodes::UNABLE_TO_ACQUIRE_QUANTIZER,
                    "Unable to Acquire Quntizer");
            }

            Math::Quantizer quantizer(get_quantizer_result.return_value());

            std::unique_ptr<Intersection::IntersectionProblemIfx> iproblem(
                Intersection::IntersectionProblemIfx::GetProblem(*single_mesh, quantizer, single_mesh->boundingBox()));

            Intersection::IntersectionProblemResult findResult = iproblem->FindIntersections();

            si_stats = iproblem->CheckSelfIntersection();
        }

        return TopologicalStatistics(num_edges, 0, num_non_2_manifold, holes, si_stats);
    }

};  // namespace Cork::Statistics
