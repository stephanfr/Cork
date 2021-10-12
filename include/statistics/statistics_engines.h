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
#include "cork.h"
#include "statistics/statistics.h"

namespace Cork::Statistics
{
    using VertexIndex = Math::VertexIndex;

    class GeometricStatisticsEngine
    {
       public:
        enum PropertiesToCompute
        {
            ALL = 0xFFFF,
            AREA_AND_VOLUME = 2,
            EDGE_LENGTHS = 4
        };

        explicit GeometricStatisticsEngine(const Cork::TriangleMesh& triangle_mesh,
                                           PropertiesToCompute propertiesToCompute = ALL);

        size_t numTriangles() const { return triangle_mesh_.numTriangles(); }

        const Math::BBox3D& boundingBox() const { return triangle_mesh_.boundingBox(); }

        double area() const { return (m_area); }

        double volume() const { return (m_volume); }

        double minEdgeLength() const { return (m_minEdgeLength); }

        double maxEdgeLength() const { return (m_maxEdgeLength); }

       private:
        PropertiesToCompute m_propertiesToCompute;

        const Cork::TriangleMesh& triangle_mesh_;

        double m_area;
        double m_volume;

        double m_minEdgeLength;
        double m_maxEdgeLength;

        void AddTriangle(const Math::TriangleByVertices& nextTriangle);
    };

    class TopologicalStatisticsEngine
    {
       public:
        TopologicalStatisticsEngine(const Cork::TriangleMesh& triangle_mesh);

        ~TopologicalStatisticsEngine()
        {
            edges_.clear();
            vertex_associations_.clear();
        }

        TopologicalStatistics Analyze();

       private:
        class EdgeAndIncidence : public Math::EdgeByIndices
        {
           public:
            EdgeAndIncidence(const VertexIndex a, const VertexIndex b)
                : Math::EdgeByIndices(a, b), m_numIncidences(0)
            {
            }

            virtual ~EdgeAndIncidence() {}

            int AddIncidence() { return (++m_numIncidences); }

            int numIncidences() const { return (m_numIncidences); }

            struct HashFunction
            {
                std::size_t operator()(const Math::EdgeByIndices& k) const { return (VertexIndex::integer_type(k.first()) * 10000019 ^ VertexIndex::integer_type(k.second())); }
            };

           private:
            int m_numIncidences;
        };

        using AssociatedVertexVector = boost::container::small_vector<Math::VertexIndex, 100>;

        using EdgeSet = std::unordered_set<EdgeAndIncidence, EdgeAndIncidence::HashFunction>;
        using VertexAssociations = std::unordered_map<Math::VertexIndex, AssociatedVertexVector>;

        const Cork::TriangleMesh& triangle_mesh_;

        double m_minEdgeLength;
        double m_maxEdgeLength;

        int num_bodys_;
        int num_non_2_manifold_;

        EdgeSet edges_;

        std::vector<Math::EdgeByIndices> hole_edges_;
        std::vector<Math::EdgeByIndices> self_intersecting_edges_;

        VertexAssociations vertex_associations_;

        void AddTriangle(const Math::TriangleByIndices& nextTriangle);
    };

}  // namespace Cork::Statistics
