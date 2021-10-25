// +-------------------------------------------------------------------------
// | Statistics.h
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

#pragma once

#include <memory>

#include "primitives/primitives.hpp"
#include "intersection/self_intersections.hpp"
#include "primitives/hole.hpp"

namespace Cork::Statistics
{
    using IntersectionInfo = Intersection::IntersectionInfo;

    class GeometricStatistics
    {
       public:
        GeometricStatistics(size_t numVertices, size_t numTriangles, double area, double volume, double minEdgeLength,
                            double maxEdgeLength, const Primitives::BBox3D& boundingBox)
            : m_numVertices(numVertices),
              m_numTriangles(numTriangles),
              m_area(area),
              m_volume(volume),
              m_minEdgeLength(minEdgeLength),
              m_maxEdgeLength(maxEdgeLength),
              m_boundingBox(std::make_unique<Primitives::BBox3D>(boundingBox))
        {
        }

        size_t numVertices() const { return (m_numVertices); }

        size_t numTriangles() const { return (m_numTriangles); }

        Primitives::BBox3D boundingBox() const { return (*m_boundingBox); }

        double area() const { return (m_area); }

        double volume() const { return (m_volume); }

        double minEdgeLength() const { return (m_minEdgeLength); }

        double maxEdgeLength() const { return (m_maxEdgeLength); }

       private:
        size_t m_numVertices;
        size_t m_numTriangles;
        double m_area;
        double m_volume;
        double m_minEdgeLength;
        double m_maxEdgeLength;
        std::unique_ptr<Primitives::BBox3D> m_boundingBox;
    };

    class TopologicalStatistics
    {
       public:

        TopologicalStatistics() = delete;

        TopologicalStatistics(size_t num_edges, size_t num_bodies, size_t non_2_manifold_edges,
                              const std::vector<Hole> holes, const std::vector<IntersectionInfo>& self_intersections)
            : num_edges_(num_edges),
              num_bodies_(num_bodies),
              non_2_manifold_edges_(non_2_manifold_edges),
              holes_(holes),
              self_intersections_(self_intersections)
        {
        }

        TopologicalStatistics(const TopologicalStatistics& stats_to_copy)
            : num_edges_(stats_to_copy.num_edges_),
              num_bodies_(stats_to_copy.num_bodies_),
              non_2_manifold_edges_(stats_to_copy.non_2_manifold_edges_),
              holes_(stats_to_copy.holes_),
              self_intersections_(stats_to_copy.self_intersections_)
        {
        }

        TopologicalStatistics(TopologicalStatistics&&) = delete;

        ~TopologicalStatistics() = default;

        TopologicalStatistics& operator=(const TopologicalStatistics&) = delete;
        TopologicalStatistics& operator=(TopologicalStatistics&&) = delete;

        size_t num_edges() const { return (num_edges_); }

        size_t num_bodies() const { return (num_bodies_); }

        bool is_two_manifold() const { return (non_2_manifold_edges_ == 0); }

        const std::vector<Hole>& holes() { return holes_; }
        const std::vector<IntersectionInfo>& self_intersections() { return self_intersections_; }

       private:
        size_t num_edges_;
        size_t num_bodies_;
        size_t non_2_manifold_edges_;

        const std::vector<Hole> holes_;
        const std::vector<IntersectionInfo> self_intersections_;
    };

}  // namespace Cork::Statistics
