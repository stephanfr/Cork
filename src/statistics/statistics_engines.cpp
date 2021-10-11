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
#include "intersection/quantization.h"
#include "mesh/IntersectionProblem.h"
#include "mesh/mesh.h"

namespace Cork::Statistics
{
    GeometricStatisticsEngine::GeometricStatisticsEngine(const TriangleMesh& triangle_mesh,
                                                         PropertiesToCompute propertiesToCompute)
        : triangle_mesh_(triangle_mesh),
          m_propertiesToCompute(propertiesToCompute),
          m_area(0.0),
          m_volume(0.0),
          m_minEdgeLength(DBL_MAX),
          m_maxEdgeLength(DBL_MIN)
    {
        for (const auto& currentTriangle : triangle_mesh_.triangles())
        {
            AddTriangle(triangle_mesh_.triangleByVertices(currentTriangle));
        }
    }

    void GeometricStatisticsEngine::AddTriangle(const Math::TriangleByVerticesBase& nextTriangle)
    {
        //	Get the edges

        if ((m_propertiesToCompute & (PropertiesToCompute::AREA_AND_VOLUME | PropertiesToCompute::EDGE_LENGTHS)) != 0)
        {
            Math::Vector3D edgeAB = nextTriangle.edgeAB_from_origin();
            Math::Vector3D edgeAC = nextTriangle.edgeAC_from_origin();
            Math::Vector3D edgeBC = nextTriangle.edgeBC_from_origin();

            if ((m_propertiesToCompute & PropertiesToCompute::AREA_AND_VOLUME) != 0)
            {
                //	Add the incremental area of this triangle

                Math::Vector3D ABcrossAC = edgeAB.cross(edgeAC);

                m_area += ABcrossAC.len() / 2;

                //	Do the same for volume

                double temp = nextTriangle.vertexA().dot(ABcrossAC);

                m_volume += temp / 6;
            }

            //	Update the min/max edge lengths

            if ((m_propertiesToCompute & PropertiesToCompute::EDGE_LENGTHS) != 0)
            {
                m_minEdgeLength =
                    std::min(m_minEdgeLength, (double)std::min(edgeAB.len(), std::min(edgeAC.len(), edgeBC.len())));
                m_maxEdgeLength =
                    std::max(m_maxEdgeLength, (double)std::max(edgeAB.len(), std::max(edgeAC.len(), edgeBC.len())));
            }
        }
    };

    TopologicalStatisticsEngine::TopologicalStatisticsEngine(const TriangleMesh& triangle_mesh)
        : triangle_mesh_(triangle_mesh), num_bodys_(0), m_minEdgeLength(DBL_MAX), m_maxEdgeLength(DBL_MIN)
    {
        edges_.reserve(triangle_mesh_.numTriangles() * 6);
        vertex_associations_.reserve(triangle_mesh_.numTriangles() * 12);

        for (const auto& currentTriangle : triangle_mesh_.triangles())
        {
            AddTriangle(currentTriangle);
        }
    }

    inline void TopologicalStatisticsEngine::AddTriangle(const Math::TriangleByIndicesBase& nextTriangle)
    {
        EdgeSet::iterator itrEdgeAB = edges_.emplace(nextTriangle.a(), nextTriangle.b()).first;
        EdgeSet::iterator itrEdgeAC = edges_.emplace(nextTriangle.a(), nextTriangle.c()).first;
        EdgeSet::iterator itrEdgeBC = edges_.emplace(nextTriangle.b(), nextTriangle.c()).first;

        int abIncidences = const_cast<EdgeAndIncidence&>(*itrEdgeAB).AddIncidence();
        int acIncidences = const_cast<EdgeAndIncidence&>(*itrEdgeAC).AddIncidence();
        int bcIncidences = const_cast<EdgeAndIncidence&>(*itrEdgeBC).AddIncidence();

        vertex_associations_[nextTriangle.a()].push_back(nextTriangle.b());
        vertex_associations_[nextTriangle.a()].push_back(nextTriangle.c());
        vertex_associations_[nextTriangle.b()].push_back(nextTriangle.a());
        vertex_associations_[nextTriangle.b()].push_back(nextTriangle.c());
        vertex_associations_[nextTriangle.c()].push_back(nextTriangle.a());
        vertex_associations_[nextTriangle.c()].push_back(nextTriangle.b());
    }

    TopologicalStatistics TopologicalStatisticsEngine::Analyze()
    {
        num_non_2_manifold_ = 0;

        for (auto& edge : edges_)
        {
            if (edge.numIncidences() != 2)
            {
                num_non_2_manifold_++;

                if (edge.numIncidences() <= 1)
                {
                    hole_edges_.push_back(dynamic_cast<const Math::EdgeByIndicesBase&>(edge));
                    std::cout << "Found Hole" << std::endl;
                }
                else
                {
                    self_intersecting_edges_.push_back(dynamic_cast<const Math::EdgeByIndicesBase&>(edge));
                    std::cout << "Found Self Intersection" << std::endl;
                }
            }
        }

        std::unique_ptr<Mesh> single_mesh(new Mesh(triangle_mesh_));

        std::cout << "Get Quantizer: " << triangle_mesh_.max_vertex_magnitude() << "    "
                  << triangle_mesh_.min_and_max_edge_lengths().min() << std::endl;

        Quantization::Quantizer::GetQuantizerResult get_quantizer_result = Quantization::Quantizer::get_quantizer(
            triangle_mesh_.max_vertex_magnitude(), triangle_mesh_.min_and_max_edge_lengths().min());

        if (!get_quantizer_result.succeeded())
        {
            std::cout << "Failed to get Quantizer: " << get_quantizer_result.message() << std::endl;

            return (TopologicalStatistics(edges_.size(), 0, num_non_2_manifold_, hole_edges_, std::vector<IntersectionInfo>() ));
        }

        Quantization::Quantizer quantizer(get_quantizer_result.return_value());

        std::unique_ptr<Intersection::IntersectionProblemIfx> iproblem(
            Intersection::IntersectionProblemIfx::GetProblem(*single_mesh, quantizer, single_mesh->boundingBox()));

        Intersection::IntersectionProblemIfx::IntersectionProblemResult findResult = iproblem->FindIntersections();

        const std::vector<IntersectionInfo>   si_stats = iproblem->CheckSelfIntersection();

        std::cout << "Found: " << si_stats.size() << " self intersections." << std::endl;

        return (TopologicalStatistics(edges_.size(), 0, num_non_2_manifold_, hole_edges_, si_stats ));
    }

};  // namespace Cork::Statistics
