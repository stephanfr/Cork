// Copyright (c) 2021 Stephan Friedl
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files (the "Software"), to deal in
// the Software without restriction, including without limitation the rights to
// use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
// the Software, and to permit persons to whom the Software is furnished to do so,
// subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
// IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
// CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "mesh/self_intersecting_regions.hpp"

#include "intersection/self_intersection_finder.hpp"
#include "mesh/si_region_premutations.inc"

namespace Cork::Meshes
{
    void SelfIntersectingRegions::find_regions()
    {
        //  Start by identifying the self intersections in the mesh

        Intersection::SelfIntersectionFinder si_finder(mesh_.topo_cache());

        auto si_stats = si_finder.CheckSelfIntersection();

        //  For each self intersection, extract the two triangles sharing the edge and then extract
        //      a region around it that extends out until it contains all triangles on the self
        //      intersecting edge out to the triangle that is intersected.  Then expand the region by
        //      an extra two rings - this will help insure that the entire problematic region is contained
        //      within the sub surface.
        //
        //  If we cannot grow the region to include the self intersected triangle, then extract the triangle
        //      and a ring of two deep around it.

        for (const SelfIntersectingEdge& intersecting_edge : si_stats)
        {
            TriangleByIndicesIndexSet tris_sharing_si_edge(std::move(mesh_.topo_cache().triangles_sharing_edge(
                intersecting_edge.edge_triangle_id(), intersecting_edge.edge_index())));

            TriangleByIndicesIndexSet si_region;

            for (uint32_t ring_size = 1; ring_size < 8; ring_size++)
            {
                auto find_tris_result = mesh_.find_enclosing_triangles(tris_sharing_si_edge, ring_size);

                if (!find_tris_result.succeeded())
                {
                    continue;
                }

                si_region = find_tris_result.return_ptr()->merge(tris_sharing_si_edge);

                if (si_region.contains(intersecting_edge.triangle_instersected_id()))
                {
                    auto find_tris_result2 = mesh_.find_enclosing_triangles(tris_sharing_si_edge, ring_size + 4);

                    if (!find_tris_result2.succeeded())
                    {
                        continue;
                    }

                    si_region = find_tris_result2.return_ptr()->merge(tris_sharing_si_edge);

                    break;
                }
            }

            if (!si_region.contains(intersecting_edge.triangle_instersected_id()))
            {
                TriangleByIndicesIndexSet intersected_tri;

                intersected_tri.insert(intersecting_edge.triangle_instersected_id());

                auto find_tris_result3 = mesh_.find_enclosing_triangles(intersected_tri, 4);

                if (!find_tris_result3.succeeded())
                {
                    continue;
                }

                regions_.emplace_back(std::move(find_tris_result3.return_ptr()->merge(intersected_tri)));
            }

            regions_.emplace_back(std::move(si_region));
        }

        //  We have a bunch of regions now - but in really messed up meshes there are typically a lot of
        //      self intersections in small areas so we may have overlapping regions.  We only want each
        //      self intersection represented once - so merge overlapping regions.

        merge_overlapping_regions();

        //  Finally, scrub the regions.  This will insure we have complete, tractable regions.

        scrub_regions();
    }

    void SelfIntersectingRegions::merge_overlapping_regions()
    {
        bool merged_regions = false;
        do
        {
            merged_regions = false;

            for (int i = 0; i < regions_.size(); i++)
            {
                for (int j = i + 1; j < regions_.size(); j++)
                {
                    if (regions_[i].intersects(regions_[j]))
                    {
                        regions_[i].merge(regions_[j]);
                        regions_.erase(regions_.begin() + j);
                        j--;
                        merged_regions = true;
                    }
                }
            }
        } while (merged_regions);
    }

    void SelfIntersectingRegions::scrub_regions()
    {
        std::vector<std::vector<Cork::TriangleByIndicesIndexSet>::iterator> unbound_regions;

        for (auto itr = regions_.begin(); itr != regions_.end(); itr++)
        {
            //  Get the boundary for the current region.  If that fails for some unofrtunate reason,
            //      we will just kick out this region and move on to the next.
            //
            //  If there is only a single boundary, then we do not need to worry about reprocessing
            //      the region to fill in any holes formed between unioned sub meshes.

            auto get_be_result = mesh_.get_boundary_edge(*itr);

            if (get_be_result.failed())
            {
                unbound_regions.insert(unbound_regions.begin(), itr);
                continue;
            }

            if (get_be_result.return_ptr()->size() == 1)
            {
                continue;
            }

            //  Multiple regions, so sort the boundaries by length and we will start with the longest
            //      boundary based on the assumption that the longest boundary will be the enclosing boundary.

            std::map<double, uint32_t, std::greater<double>> boundaries_ordered_by_length;

            for (uint32_t i = 0; i < get_be_result.return_ptr()->size(); i++)
            {
                double current_length = (*(get_be_result.return_ptr()))[i].length(mesh_.vertices());

                boundaries_ordered_by_length.insert(std::make_pair(current_length, i));
            }

            bool region_confirmed = false;

            const std::vector<std::vector<uint32_t>>& permutations =
                g_region_permutations[boundaries_ordered_by_length.size() - 1];

            for (const std::vector<uint32_t>& current_permutation : permutations)
            {
                std::vector<TopoEdgeBoundary> boundaries;

                for (const auto& current_index : current_permutation)
                {
                    TopoEdgeBoundary topo_edge_boundary =
                        mesh_.topo_cache().topo_edge_boundary((*get_be_result.return_ptr())[current_index]);

                    boundaries.emplace_back(topo_edge_boundary);
                }

                uint32_t tri_in_region_index = TriangleByIndicesIndex::integer_type(*(itr->begin()));

                const TopoTri& topo_tri_seed = mesh_.topo_cache().triangles().getPool()[tri_in_region_index];

                std::unordered_set<const TopoTri*> topo_tris_in_boundary(
                    std::move(mesh_.topo_cache().tris_inside_boundaries(boundaries, topo_tri_seed, itr->size() + 500)));

                if (topo_tris_in_boundary.size() == itr->size())
                {
                    region_confirmed = true;
                    break;
                }

                if (topo_tris_in_boundary.size() > itr->size())
                {
                    for (auto tri_id : topo_tris_in_boundary)
                    {
                        if (!itr->contains(tri_id->ref()))
                        {
                            itr->insert(tri_id->ref());
                        }
                    }

                    region_confirmed = true;
                    break;
                }
            }
        }

        //  Remove the unbounded regions.  The iterators should have been pushed into the unbound_regions vector
        //      at the front, so erasing should happen from the end of regions_ forward.

        for (auto itr : unbound_regions)
        {
            regions_.erase(itr);
        }
    }
}  // namespace Cork::Meshes