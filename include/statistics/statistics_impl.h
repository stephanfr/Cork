// +-------------------------------------------------------------------------
// | StatsImpl.h
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

#include <boost/container/small_vector.hpp>
#include <unordered_set>

#include "CorkDefs.h"

//#include "math/Primitives.h"
//#include "mesh/EGraphCache.h"
#include "statistics/statistics.h"
//#include "util/unionFind.h"

namespace Cork::Statistics
{
    static const Cork::Math::BBox3D gEmptyBoundingBox(Cork::Math::Vector3D(0.0, 0.0, 0.0),
                                                      Cork::Math::Vector3D(0.0, 0.0, 0.0));

    class GeometricStatisticsEngine
    {
       public:
        enum PropertiesToCompute
        {
            ALL = 0xFFFF,
            AREA_AND_VOLUME = 2,
            EDGE_LENGTHS = 4,
            BOUNDING_BOX = 8
        };

        explicit GeometricStatisticsEngine(PropertiesToCompute propertiesToCompute = ALL)
            : m_propertiesToCompute(propertiesToCompute),
              m_numTriangles(0),
              m_area(0.0),
              m_volume(0.0),
              m_minEdgeLength(DBL_MAX),
              m_maxEdgeLength(DBL_MIN)
        {
        }

        void AddTriangle(const Cork::Math::TriangleByVerticesBase& nextTriangle)
        {
            //	Get the edges

            if ((m_propertiesToCompute & (PropertiesToCompute::AREA_AND_VOLUME | PropertiesToCompute::EDGE_LENGTHS)) !=
                0)
            {
                Cork::Math::Vector3D edgeAB = nextTriangle.edgeAB();
                Cork::Math::Vector3D edgeAC = nextTriangle.edgeAC();
                Cork::Math::Vector3D edgeBC = nextTriangle.edgeBC();

                if ((m_propertiesToCompute & PropertiesToCompute::AREA_AND_VOLUME) != 0)
                {
                    //	Add the incremental area of this triangle

                    Cork::Math::Vector3D ABcrossAC = edgeAB.cross(edgeAC);

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

            if ((m_propertiesToCompute & PropertiesToCompute::BOUNDING_BOX) != 0)
            {
                m_boundingBox.convex(Cork::Math::BBox3D(nextTriangle[0].min(nextTriangle[1], nextTriangle[2]),
                                                        nextTriangle[0].max(nextTriangle[1], nextTriangle[2])));
            }

            //	Increment the triangle count

            m_numTriangles++;
        };

        size_t numTriangles() const { return (m_numTriangles); }

        const Cork::Math::BBox3D& boundingBox() const
        {
            //	Special case when there are no triangles

            if (m_numTriangles == 0)
            {
                return (gEmptyBoundingBox);
            }

            return (m_boundingBox);
        }

        double area() const { return (m_area); }

        double volume() const { return (m_volume); }

        double minEdgeLength() const { return (m_minEdgeLength); }

        double maxEdgeLength() const { return (m_maxEdgeLength); }

       private:
        PropertiesToCompute m_propertiesToCompute;

        size_t m_numTriangles;

        Cork::Math::BBox3D m_boundingBox;

        double m_area;
        double m_volume;

        double m_minEdgeLength;
        double m_maxEdgeLength;
    };

    class TopologicalStatisticsEngine
    {
       public:
        TopologicalStatisticsEngine(size_t numTriangles) : num_bodys_(0)
        {
            edges_.reserve(numTriangles * 6);
            vertex_associations_.reserve(numTriangles * 12);
        }

        ~TopologicalStatisticsEngine()
        {
            edges_.clear();
            vertex_associations_.clear();
        }

        void AddTriangle(const Cork::Math::TriangleByIndicesBase& nextTriangle)
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

        TopologicalStatistics Analyze()
        {
            num_non_2_manifold_ = 0;

            for (auto& edge : edges_)
            {
                if (edge.numIncidences() != 2)
                {
                    num_non_2_manifold_++;

                    if (edge.numIncidences() <= 1)
                    {
                        hole_edges_.push_back(dynamic_cast<const Cork::Math::EdgeBase&>(edge));
                    }
                    else
                    {
                        self_intersecting_edges_.push_back(dynamic_cast<const Cork::Math::EdgeBase&>(edge));
                    }
                }
            }

            return (TopologicalStatistics(edges_.size(), 0, num_non_2_manifold_ ));
        }

       private:
        class EdgeAndIncidence : public Cork::Math::EdgeBase
        {
           public:
            EdgeAndIncidence(const Cork::Math::IndexType a, const Cork::Math::IndexType b)
                : Cork::Math::EdgeBase(a, b), m_numIncidences(0)
            {
            }

            virtual ~EdgeAndIncidence() {}

            int AddIncidence() { return (++m_numIncidences); }

            int numIncidences() const { return (m_numIncidences); }

            struct HashFunction
            {
                std::size_t operator()(const Cork::Math::EdgeBase& k) const
                {
                    return (k.vertexA() * 10000019 ^ k.vertexB());
                }
            };

           private:
            int m_numIncidences;
        };

        using AssociatedVertexVector = boost::container::small_vector<Cork::Math::IndexType, 100>;

        using EdgeSet = std::unordered_set<EdgeAndIncidence, EdgeAndIncidence::HashFunction>;
        using VertexAssociations = std::unordered_map<Cork::Math::IndexType, AssociatedVertexVector>;

        int num_bodys_;
        int num_non_2_manifold_;

        EdgeSet edges_;

        std::vector<Cork::Math::EdgeBase> hole_edges_;
        std::vector<Cork::Math::EdgeBase> self_intersecting_edges_;

        VertexAssociations vertex_associations_;
    };

}  // namespace Cork::Statistics
