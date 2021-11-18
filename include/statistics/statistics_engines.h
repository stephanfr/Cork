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

#include "cork_defs.hpp"
#include "mesh/triangle_mesh_with_topo_cache.hpp"
#include "statistics.hpp"

namespace Cork::Statistics
{
    class GeometricStatisticsEngine
    {
       public:
        explicit GeometricStatisticsEngine(const Cork::TriangleMesh& triangle_mesh,
                                           GeometricProperties propertiesToCompute = GeometricProperties::GEOM_ALL);

        GeometricStatistics statistics() const
        {
            return GeometricStatistics(num_triangles_, num_vertices_, area_, volume_, min_edge_length_,
                                       max_edge_length_, bounding_box_);
        }

       private:
        uint32_t num_triangles_;
        uint32_t num_vertices_;

        double area_;
        double volume_;

        double min_edge_length_;
        double max_edge_length_;

        Primitives::BBox3D bounding_box_;

        void AddTriangle(const Primitives::TriangleByVertices& nextTriangle);
    };

    enum class TopologicalStatisticsEngineAnalyzeResultCodes
    {
        SUCCESS = 0,

        UNABLE_TO_ACQUIRE_QUANTIZER
    };

    using TopologicalStatisticsEngineAnalyzeResult =
        SEFUtility::ResultWithReturnValue<TopologicalStatisticsEngineAnalyzeResultCodes, TopologicalStatistics>;

    class TopologicalStatisticsEngine
    {
       public:
        TopologicalStatisticsEngine(const Cork::Meshes::TriangleMeshWithTopoCache& triangle_mesh);

        ~TopologicalStatisticsEngine() = default;

        TopologicalStatisticsEngineAnalyzeResult Analyze(TopologicalProperties props_to_compute);

       private:
        const Cork::Meshes::TriangleMeshWithTopoCache& triangle_mesh_;

        class EdgeAndIncidence : public Primitives::EdgeByIndices
        {
           public:
            EdgeAndIncidence(const Primitives::VertexIndex a, const Primitives::VertexIndex b)
                : Primitives::EdgeByIndices(a, b), m_numIncidences(0)
            {
            }

            virtual ~EdgeAndIncidence() {}

            int AddIncidence() { return (++m_numIncidences); }

            int numIncidences() const { return (m_numIncidences); }

            struct HashFunction
            {
                std::size_t operator()(const Primitives::EdgeByIndices& k) const
                {
                    return (Primitives::VertexIndex::integer_type(k.first()) * 10000019 ^
                            Primitives::VertexIndex::integer_type(k.second()));
                }
            };

           private:
            int m_numIncidences;
        };

        using EdgeSet = std::unordered_set<EdgeAndIncidence, EdgeAndIncidence::HashFunction>;

        EdgeSet edges_;
    };

}  // namespace Cork::Statistics
