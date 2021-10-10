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

#include "math/Primitives.h"
#include "mesh/self_intersections.hpp"

namespace Cork::Statistics
{
    class GeometricStatistics
    {
       public:
        GeometricStatistics(size_t numVertices, size_t numTriangles, double area, double volume, double minEdgeLength,
                            double maxEdgeLength, const Math::BBox3D& boundingBox)
            : m_numVertices(numVertices),
              m_numTriangles(numTriangles),
              m_area(area),
              m_volume(volume),
              m_minEdgeLength(minEdgeLength),
              m_maxEdgeLength(maxEdgeLength),
              m_boundingBox(std::make_unique<Math::BBox3D>(boundingBox))
        {
        }

        size_t numVertices() const { return (m_numVertices); }

        size_t numTriangles() const { return (m_numTriangles); }

        Math::BBox3D boundingBox() const { return (*m_boundingBox); }

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
        std::unique_ptr<Math::BBox3D> m_boundingBox;
    };

    class TopologicalStatistics
    {
       public:
        using EdgeByIndicesVector = Math::EdgeByIndicesVector;

        TopologicalStatistics() = delete;

        TopologicalStatistics(size_t num_edges, size_t num_bodies, size_t non_2_manifold_edges, const EdgeByIndicesVector& hole_edges,
                              const Cork::Intersection::SelfIntersectionStats& self_intersections)
            : num_edges_(num_edges),
              num_bodies_(num_bodies),
              non_2_manifold_edges_(non_2_manifold_edges),
              hole_edges_(hole_edges),
              self_intersections_(self_intersections)
        {
        }

        TopologicalStatistics(const TopologicalStatistics& stats_to_copy)
            : num_edges_(stats_to_copy.num_edges_),
              num_bodies_(stats_to_copy.num_bodies_),
              non_2_manifold_edges_(stats_to_copy.non_2_manifold_edges_),
              hole_edges_(stats_to_copy.hole_edges_),
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

        const EdgeByIndicesVector& hole_edges() { return hole_edges_; }
        const Intersection::SelfIntersectionStats& self_intersections() { return self_intersections_; }

       private:
        size_t num_edges_;
        size_t num_bodies_;
        size_t non_2_manifold_edges_;

        const EdgeByIndicesVector hole_edges_;
        const Intersection::SelfIntersectionStats self_intersections_;
    };

}  // namespace Cork::Statistics
