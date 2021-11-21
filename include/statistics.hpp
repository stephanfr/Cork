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

#include <set>

#include "primitives/hole.hpp"

namespace Cork::Statistics
{
    enum GeometricProperties : uint16_t
    {
        GEOM_ALL = 0xFFFF,
        GEOM_AREA_AND_VOLUME = 2,
        GEOM_EDGE_LENGTHS = 4
    };

    enum TopologicalProperties : uint16_t
    {
        TOPO_ALL = 0xFFFF,
        TOPO_BASE = 1,
        TOPO_HOLES = 2,
        TOPO_SELF_INTERSECTIONS = 4
    };

    class GeometricStatistics
    {
       public:
        GeometricStatistics(uint32_t num_triangles, uint32_t num_vertices, double area, double volume,
                            double min_edge_length, double max_edge_length, const BBox3D& bounding_box)
            : num_triangles_(num_triangles),
              num_vertices_(num_vertices),
              area_(area),
              volume_(volume),
              min_edge_length_(min_edge_length),
              max_edge_length_(max_edge_length),
              bounding_box_(bounding_box)
        {
        }

        size_t num_vertices() const { return (num_vertices_); }

        size_t num_triangles() const { return (num_triangles_); }

        const BBox3D& bounding_box() const { return bounding_box_; }

        double area() const { return (area_); }

        double volume() const { return (volume_); }

        double min_edge_length() const { return (min_edge_length_); }

        double max_edge_length() const { return (max_edge_length_); }

       private:
        size_t num_vertices_;
        size_t num_triangles_;
        double area_;
        double volume_;
        double min_edge_length_;
        double max_edge_length_;
        const BBox3D bounding_box_;
    };

    class SelfIntersectingEdge
    {
       public:
        SelfIntersectingEdge() = delete;

        SelfIntersectingEdge(const SelfIntersectingEdge& ) = default;
        SelfIntersectingEdge(SelfIntersectingEdge&& ) = default;

        SelfIntersectingEdge(TriangleByIndicesIndex edge_triangle_id, TriangleEdgeId edge_index,
                             TriangleByIndicesIndex triangle_instersected_id)
            : edge_triangle_id_(edge_triangle_id),
              edge_index_(edge_index),
              triangle_instersected_id_(triangle_instersected_id)
        {
        }

        SelfIntersectingEdge&   operator=(const SelfIntersectingEdge&) = default;
        SelfIntersectingEdge&   operator=(SelfIntersectingEdge&&) = default;

        TriangleByIndicesIndex edge_triangle_id() const { return edge_triangle_id_; }
        TriangleEdgeId edge_index() const { return edge_index_; }
        TriangleByIndicesIndex triangle_instersected_id() const { return triangle_instersected_id_; }

       private:
        TriangleByIndicesIndex edge_triangle_id_;
        TriangleEdgeId edge_index_;
        TriangleByIndicesIndex triangle_instersected_id_;
    };


    class TopologicalStatistics
    {
       public:
        TopologicalStatistics() = delete;

        TopologicalStatistics(size_t num_edges, size_t num_bodies, size_t non_2_manifold_edges,
                              const std::vector<Hole>& holes, const std::vector<SelfIntersectingEdge>& self_intersecting_edges)
            : num_edges_(num_edges),
              num_bodies_(num_bodies),
              non_2_manifold_edges_(non_2_manifold_edges),
              holes_(holes),
              self_intersecting_edges_(self_intersecting_edges)
        {
        }

        TopologicalStatistics(const TopologicalStatistics& stats_to_copy)
            : num_edges_(stats_to_copy.num_edges_),
              num_bodies_(stats_to_copy.num_bodies_),
              non_2_manifold_edges_(stats_to_copy.non_2_manifold_edges_),
              holes_(stats_to_copy.holes_),
              self_intersecting_edges_(stats_to_copy.self_intersecting_edges_)
        {
        }

        TopologicalStatistics(TopologicalStatistics&&) = delete;

        ~TopologicalStatistics() = default;

        TopologicalStatistics& operator=(const TopologicalStatistics&) = delete;
        TopologicalStatistics& operator=(TopologicalStatistics&&) = delete;

        size_t num_edges() const { return (num_edges_); }

        size_t num_bodies() const { return (num_bodies_); }

        bool is_two_manifold() const { return (non_2_manifold_edges_ == 0); }

        const std::vector<Hole>& holes() const { return holes_; }
        const std::vector<SelfIntersectingEdge>& self_intersecting_edges() const { return self_intersecting_edges_; }

       private:
        size_t num_edges_;
        size_t num_bodies_;
        size_t non_2_manifold_edges_;

        const std::vector<Hole> holes_;
        const std::vector<SelfIntersectingEdge> self_intersecting_edges_;
    };

}  // namespace Cork::Statistics
