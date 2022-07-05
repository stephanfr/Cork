// +-------------------------------------------------------------------------
// | Mesh.cpp
// |
// | Author: Gilbert Bernstein
// +-------------------------------------------------------------------------
// | COPYRIGHT:
// |    Copyright Gilbert Bernstein 2013
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

#include "mesh/surface_mesh.hpp"

#include <numeric>

#include "file_formats/files.hpp"
#include "intersection/self_intersection_finder.hpp"
#include "math/normal_projector.hpp"
#include "math/plane.hpp"
#include "mesh/edge_incidence_counter.hpp"

namespace Cork::Meshes
{
    class EdgesByVerticesMap
    {
       public:
        EdgesByVerticesMap() = delete;

        template <typename I>
        EdgesByVerticesMap(I itr_begin, I itr_end)
        {
            for (auto itr = itr_begin; itr != itr_end; itr++)
            {
                edges_by_verts_.insert(std::make_pair(&((*itr)->vert_0()), *itr));
                edges_by_verts_.insert(std::make_pair(&((*itr)->vert_1()), *itr));
            }
        }

        bool empty() const { return edges_by_verts_.empty(); }

        const TopoEdge* find(const TopoVert* vert) { return edges_by_verts_.find(vert)->second; }

        std::multimap<const TopoVert*, const TopoEdge*>::iterator begin() { return edges_by_verts_.begin(); }

        std::multimap<const TopoVert*, const TopoEdge*>::iterator end() { return edges_by_verts_.end(); }

        void erase(const TopoEdge* edge_to_remove)
        {
            auto vert0_range = edges_by_verts_.equal_range(&(edge_to_remove->vert_0()));

            for (auto itr = vert0_range.first; itr != vert0_range.second; itr++)
            {
                if (itr->second == edge_to_remove)
                {
                    edges_by_verts_.erase(itr);
                    break;
                }
            }

            auto vert1_range = edges_by_verts_.equal_range(&(edge_to_remove->vert_1()));

            for (auto itr = vert1_range.first; itr != vert1_range.second; itr++)
            {
                if (itr->second == edge_to_remove)
                {
                    edges_by_verts_.erase(itr);
                    break;
                }
            }
        }

        template <typename I>
        static TopoEdgeBoundaryVector find_all_boundaries(I itr_begin, I itr_end)
        {
            //  Build out the boundaries by starting with a single edge and growing it edge by edge until it closes.

            EdgesByVerticesMap edges_by_verts(itr_begin, itr_end);

            TopoEdgeBoundaryVector boundaries;

            //  Loop while we have edges to process

            while (!edges_by_verts.empty())
            {
                //  Grab the first edge as a starting point.  Then work through the collection of edges
                //      looking for the next edge based on the last vertex on the boundary until we run
                //      out of vertices to process.

                const TopoEdge* current_edge = edges_by_verts.begin()->second;
                TopoEdgeBoundary new_boundary(current_edge);

                edges_by_verts.erase(current_edge);

                while (!new_boundary.is_closed())
                {
                    auto current_edge = edges_by_verts.find(new_boundary.last_vert());

                    new_boundary.add_edge(current_edge);
                    edges_by_verts.erase(current_edge);
                }

                boundaries.emplace_back(std::move(new_boundary));
            }

            return boundaries;
        }

       private:
        std::multimap<const TopoVert*, const TopoEdge*> edges_by_verts_;
    };

    SurfaceMesh::SurfaceMesh(const TriangleMesh& inputMesh)
        : MeshBase(inputMesh.num_vertices(), inputMesh.num_triangles())
    {
        min_and_max_edge_lengths_ = inputMesh.min_and_max_edge_lengths();
        max_vertex_magnitude_ = inputMesh.max_vertex_magnitude();

        tris_->reserve(inputMesh.num_triangles());
        verts_->reserve(inputMesh.num_vertices());

        //	Start by copying the vertices.

        for (auto currentVertex : inputMesh.vertices())
        {
            verts_->emplace_back(currentVertex.x(), currentVertex.y(), currentVertex.z());
        }

        //	Fill the triangles

        for (TriangleByIndicesIndex i = 0u; i < inputMesh.triangles().size(); i++)
        {
            tris_->emplace_back(inputMesh.triangles()[i], 0);
        }

        bounding_box_ = inputMesh.bounding_box();
    }

    SurfaceMesh::~SurfaceMesh() {}

    void SurfaceMesh::operator=(SurfaceMesh&& src)
    {
        tris_ = std::move(src.tris_);
        verts_ = std::move(src.verts_);

        bounding_box_ = std::move(src.bounding_box_);

        min_and_max_edge_lengths_ = std::move(src.min_and_max_edge_lengths_);
        max_vertex_magnitude_ = std::move(src.max_vertex_magnitude_);

        topo_cache_ = std::move(src.topo_cache_);

        centroid_ = std::move(src.centroid_);
    }

    BestFitPlaneEquation SurfaceMesh::best_fit_plane() const
    {
       return BestFitPlaneEquation( verts_->size(), verts_->begin(), verts_->end() );
    }

    std::unique_ptr<SurfaceMesh> SurfaceMesh::project_surface(const Vector3D projection_surface_normal,
                                                              const Vertex3D normal_surface_origin)
    {
        MeshBase projected_mesh(verts_->size(), tris_->size());

        for (auto vert : *verts_)
        {
            Vertex3D projected_vertex =
                vert - ((vert - normal_surface_origin).dot(projection_surface_normal) * projection_surface_normal);

            projected_mesh.verts_->emplace_back(projected_vertex);
        }

        for (auto tri : *tris_)
        {
            projected_mesh.add_triangle_and_update_metrics(tri);
        }

        return std::make_unique<SurfaceMesh>(std::move(projected_mesh));
    }

    FindTopoEdgeBoundariesResult SurfaceMesh::find_topo_edge_boundaries()
    {
        //  Start with a seed triangle which is simply the first in the pool.

        return find_topo_edge_boundaries(&(topo_cache().triangles().getPool().front()));
    }

    FindTopoEdgeBoundariesResult SurfaceMesh::find_topo_edge_boundaries(const TopoTri* seed_triangle)
    {
        //  The difference between this method on 'get_boundary_edge()' is that 'get_boundary_edge'
        //      relies only on hooking up vertices on edges with a single incidence.  This method takes
        //      a starting triangle and grows it outward to find the 'largest' fully connected surface
        //      in the SurfaceMesh - so it should be more robust and we can rely on the triangle set
        //      obeying basic principles of 2 manifold meshes *inside* of the  boundary edge.

        //  Starting from the seed triangle, grow the surface outward until we run out of triangles to process.
        //      Along the way, keep track of the boundary edges, we use them below to get reliable boundaries.

        std::unordered_set<const TopoEdge*> all_boundary_edges_found;
        std::unordered_set<const TopoEdge*> edges_processed;
        std::unordered_set<const TopoTri*> tris_inside_boundary;
        std::vector<const TopoTri*> tris_to_process;

        tris_to_process.emplace_back(seed_triangle);

        while (!tris_to_process.empty())
        {
            const TopoTri* current_tri = tris_to_process.back();
            tris_to_process.pop_back();

            tris_inside_boundary.insert(current_tri);

            for (auto current_edge : current_tri->edges())
            {
                if (edges_processed.contains(current_edge))
                {
                    continue;
                }

                if (current_edge->triangles().size() == 1)
                {
                    all_boundary_edges_found.insert(current_edge);
                }
                else if (current_edge->triangles().size() == 2)
                {
                    if (current_edge->triangles()[0] != current_tri)
                    {
                        tris_to_process.push_back(current_edge->triangles()[0]);
                    }
                    else
                    {
                        tris_to_process.push_back(current_edge->triangles()[1]);
                    }
                }
                else
                {
                    return FindTopoEdgeBoundariesResult::failure(
                        FindTopoEdgeBoundariesResultCodes::EDGE_WITH_MORE_THAN_TWO_INCIDENCES_FOUND,
                        "Edge with 3 or more incidences found.  This surface is too irregular to find boundaries.");
                }

                edges_processed.insert(current_edge);
            }
        }

        //  Build out the boundaries by starting with a single edge and growing it edge by edge until it closes.

        TopoEdgeBoundaryVector boundaries = std::move(
            EdgesByVerticesMap::find_all_boundaries(all_boundary_edges_found.begin(), all_boundary_edges_found.end()));

        double longest_boundary = -1;
        int longest_boundary_index = -1;

        for (uint32_t i = 0; i < boundaries.size(); i++)
        {
            //  Find the boundary that is longest - we will assume this is the outside boundary.
            //      Shorter boundaries *should* be holes.
            //
            //  There are absolutely degenerate cases where this heuristic may fail, but my suspicion is that
            //      those cases will be so irregular that they will not be fixable with the heuristics
            //      implemented here.
            //
            //  A more definitive test would be to take the boundary bounding boxes and translate and rotate them
            //      so that we could test for containment of holes within the largest boundary.

            double current_length = boundaries[i].length();

            if (current_length > longest_boundary)
            {
                longest_boundary = current_length;
                longest_boundary_index = i;
            }
        }

        //  Prepare the collection of triangles for return

        TriangleByIndicesIndexSet tris_inside;

        for (auto current_topo_tri : tris_inside_boundary)
        {
            tris_inside.insert(current_topo_tri->ref());
        }

        std::vector<TopoEdgeBoundary> holes;

        for (int j = 0; j < boundaries.size(); j++)
        {
            if (j == longest_boundary_index)
            {
                continue;
            }

            holes.emplace_back(std::move(boundaries[j]));
        }

        //  Return the outside boundary, the enclosed triangles and any holes we found.

        TopoEdgeBoundaryAndHoles result(std::move(boundaries[longest_boundary_index]), std::move(tris_inside),
                                        std::move(holes));

        return FindTopoEdgeBoundariesResult::success(std::move(result));
    }

    void SurfaceMesh::remove_self_intersections()
    {
        //  Start by obtaining the initial boundaries.  We will retain all of these
        //      boundaries and not fill them.

        auto initial_boundaries = find_topo_edge_boundaries();

        if (initial_boundaries.failed())
        {
            std::cout << "Find initial boundaries failed." << std::endl;
            return;
        }

        //  Identify a triangle on the outside boundary, we will need this when we get the
        //      boundaries again later.

        TriangleUID tri_on_boundary_uid =
            initial_boundaries.return_value().outside_boundary_.triangle_on_boundary_uid();

        //  Next, find all the self intersections

        Intersection::SelfIntersectionFinder si_finder(topo_cache());

        auto si_stats = si_finder.CheckSelfIntersection();

        //  For each self intersection, extract the two triangles sharing the edge and then extract
        //      one ring around it.  Do the same for the intersected triangle.

        std::set<TriangleByIndicesIndex, std::greater<>> all_tris_to_remove;

        for (const SelfIntersectingEdge& edge : si_stats)
        {
            TriangleByIndicesIndexSet tris_sharing_si_edge(
                std::move(topo_cache().triangles_sharing_edge(edge.edge_triangle_id(), edge.edge_index())));

            TriangleByIndicesIndexSet si_region;

            auto find_tris_result = find_enclosing_triangles(tris_sharing_si_edge, 1);

            if (!find_tris_result.succeeded())
            {
                std::cout << "Find enclosing si edge tris failed" << std::endl;
                continue;
            }

            all_tris_to_remove.merge(tris_sharing_si_edge);
            all_tris_to_remove.merge(*(find_tris_result.return_ptr()));

            TriangleByIndicesIndexSet intersected_tri;

            intersected_tri.insert(edge.triangle_instersected_id());

            auto find_tris_result2 = find_enclosing_triangles(intersected_tri, 1);

            if (!find_tris_result2.succeeded())
            {
                std::cout << "Find enclosing intersected tri tris failed" << std::endl;
                continue;
            }

            all_tris_to_remove.merge(intersected_tri);
            all_tris_to_remove.merge(*(find_tris_result2.return_ptr()));
        }

        //  Remove all the triangles associated with self intersections.
        //      The set of indices should be sorted in descending order.

        for (auto tri_index : all_tris_to_remove)
        {
            remove_triangle(tri_index);
        }

        //  Look for self intersections again - we should not find any.

        Intersection::SelfIntersectionFinder si_finder_after(topo_cache());

        auto si_stats_after = si_finder_after.CheckSelfIntersection();

        std::cout << "After removing tris: " << si_stats_after.size() << " self intersections" << std::endl;

        //  Find the boundaries again and identify the originals - all the rest should be holes associated
        //      with the self intersections we removed.

        auto after_boundaries =
            find_topo_edge_boundaries(topo_cache().find_topo_tri_by_source_triangle_uid(tri_on_boundary_uid));

        if (after_boundaries.failed())
        {
            std::cout << "Find boundaries after removing sis failed" << std::endl;
            return;
        }

        if ((initial_boundaries.return_value().outside_boundary_.edges().size() !=
             after_boundaries.return_value().outside_boundary_.edges().size()) ||
            (fabs(initial_boundaries.return_value().outside_boundary_.length() -
                  after_boundaries.return_value().outside_boundary_.length()) > 1e-6))
        {
            std::cout << "Outside Boundary is not intact" << std::endl;
            return;
        }

        TopoEdgeBoundaryVector si_holes_to_fill;
        uint32_t num_original_holes_found = 0;

        for (auto hole : after_boundaries.return_value().holes_)
        {
            bool found_original_hole = false;

            for (auto original_hole : initial_boundaries.return_value().holes_)
            {
                if ((hole.edges().size() == original_hole.edges().size()) &&
                    (fabs(hole.length() - original_hole.length()) < 1e-6))
                {
                    num_original_holes_found++;
                    break;
                }
            }

            if (found_original_hole)
            {
                continue;
            }

            si_holes_to_fill.emplace_back(std::move(hole));
        }

        if (initial_boundaries.return_value().holes_.size() != num_original_holes_found)
        {
            std::cout << "Could not identify all original holes" << std::endl;
            return;
        }

        std::cout << "Num si holes to fill: " << si_holes_to_fill.size() << std::endl;

        //  Finally, remove any disconnected triangles

        if (after_boundaries.return_value().tris_inside_boundary_.size() != tris_->size())
        {
            std::cout << "Disconnected triangles found in surface: "
                      << tris_->size() - after_boundaries.return_value().tris_inside_boundary_.size() << std::endl;

            Cork::Files::writeOFF("../../UnitTest/Test Results/region_with_disconnected_tris.off", *this);

            //  Remove the disconnected triangles, they should be in the middle of holes and will get replaced
            //      when holes are filled.

            //            TriangleByIndicesIndexSet disconnected_tris;

            for (TriangleByIndicesIndex i = 0u; i < tris_->size(); i++)
            {
                if (!after_boundaries.return_value().tris_inside_boundary_.contains(i))
                {
                    //                    disconnected_tris.insert(i);
                    all_tris_to_remove.insert(i);
                }
            }

            //            for (auto itr = disconnected_tris.rbegin(); itr != disconnected_tris.rend(); itr++)
            //            {
            //                remove_triangle(*itr);
            //            }
        }

//        Cork::Files::writeOFF("../../UnitTest/Test Results/region_with_holes.off", *this);

        TriangleByIndicesVector all_tris_to_add;

        for (auto& hole_to_fill : si_holes_to_fill)
        {
            BoundaryEdge boundary = hole_to_fill.as_boundary_edge(*verts_);

            Cork::Files::write_3d_polyline("../../UnitTest/Test Results/boundary.mat", boundary);

            //  Smooth the boundary, this helps occasionally with projection

            TopoEdgeBoundary topo_boundary = topo_cache().topo_edge_boundary(boundary);

            topo_boundary.smooth();

            Cork::Files::write_3d_polyline("../../UnitTest/Test Results/boundary_smooth.mat",
                                           topo_boundary.as_boundary_edge(*verts_));

            //  If the smoothed boundary is shorter, then use it moving forward

            if (boundary.vertex_indices().size() > topo_boundary.edges().size())
            {
                boundary = topo_boundary.as_boundary_edge(*verts_);
            }

            Vertex3D boundary_centroid = boundary.centroid();
            PlaneEquation best_fit_plane = boundary.best_fit_plane();

            auto projected_boundary = boundary.project(best_fit_plane.unit_normal(), boundary_centroid);

            auto self_intersections = projected_boundary.self_intersections();

            if (self_intersections.size() > 0)
            {
                std::cout << "Boundary self intersects" << std::endl;

                //  Let's split the boundary into quarters and add the centroid to each 1/4.  This gives
                //      us four pie wedges to try individually.

                std::vector<BoundaryEdge> sections =
                    boundary.divide(std::min(std::size_t(8), boundary.vertex_indices().size() / 2));

                std::cout << "Dividing into: " << sections.size() << " sections" << std::endl;

                for (auto& current_section : sections)
                {
                    current_section.append(boundary_centroid, Primitives::UNINITIALIZED_INDEX);

                    Vertex3D new_boundary_centroid = current_section.centroid();
                    PlaneEquation new_best_fit_plane = current_section.best_fit_plane();

                    auto new_projected_boundary = current_section.project(new_best_fit_plane.unit_normal(), new_boundary_centroid);

                    auto new_self_intersections = new_projected_boundary.self_intersections();

                    if (new_self_intersections.size() > 0)
                    {
                        std::cout << "Section has self intersection" << std::endl;
                    }
                }
            }

            GetHoleClosingTrianglesResult get_hole_closing_result =
                MeshBase::close_hole(hole_to_fill.as_boundary_edge(*verts_));

            if (get_hole_closing_result.failed())
            {
                std::cout << "Could not close hole" << std::endl;
                continue;
            }

            for (auto tri : get_hole_closing_result.return_value().triangles_to_add_)
            {
                all_tris_to_add.emplace_back(tri);
            }
        }

        for (auto& tri_to_add : all_tris_to_add)
        {
            tris_->emplace_back(tri_to_add);
        }

        Cork::Files::writeOFF("../../UnitTest/Test Results/region_repaired.off", *this);
    }

    void SurfaceMesh::remove_self_intersection(const SelfIntersectingEdge& self_intersection)
    {
        //  We will start by removing the triangle containing the self intersecting edge and
        //      one rign around it.  That *should* leave the two vertices on the edge
        //      completely disconnected.

        auto& tri_including_si_edge = triangles()[self_intersection.edge_triangle_id()];

        VertexIndex vertex_1 = tri_including_si_edge.edge(self_intersection.edge_index()).first();
        VertexIndex vertex_2 = tri_including_si_edge.edge(self_intersection.edge_index()).second();

        TriangleByIndicesIndexSet tri_including_vertex_1{std::move(find_triangles_including_vertex(vertex_1))};

        TriangleByIndicesIndexSet tri_including_vertex_2{std::move(find_triangles_including_vertex(vertex_2))};

        TriangleByIndicesIndexSet tris_including_either_vertex(tri_including_vertex_1, tri_including_vertex_2);

        //  Find the triangles adjoining the self intersecting edge

        TriangleByIndicesIndexSet single_tri;

        single_tri.insert(self_intersection.edge_triangle_id());

        auto find_enclosing_triangles_result = find_enclosing_triangles(single_tri, 2);

        if (!find_enclosing_triangles_result.succeeded())
        {
            std::cout << "Could not find enclosing triangles" << std::endl;
            return;
        }

        //  Try expanding the enclosing region around the self intersecting triangle.  If the triangle is on the
        //  main
        //      surface this should result in a single connected surface and a set of leftover triangles.

        auto find_full_surface_result = find_enclosing_triangles(single_tri, 12);

        if (!find_full_surface_result.succeeded())
        {
            std::cout << "Could not find enclosing triangles" << std::endl;
            return;
        }

        TriangleByIndicesIndexSet tris_in_surface;

        TriangleByIndicesIndex tri_index = 0u;
        for (auto tri : triangles())
        {
            tris_in_surface.insert(tri_index++);
        }

        TriangleByIndicesIndexSet excess_tris;

        std::set_difference(
            tris_in_surface.begin(), tris_in_surface.end(), find_full_surface_result.return_ptr()->begin(),
            find_full_surface_result.return_ptr()->end(), std::inserter(excess_tris, excess_tris.begin()));

        std::cout << "Excess tris count: " << excess_tris.size() << std::endl;

        TriangleByIndicesIndexSet all_tris_to_remove(*(find_enclosing_triangles_result.return_ptr()), excess_tris);

        for (auto itr_element = all_tris_to_remove.rbegin(); itr_element != all_tris_to_remove.rend(); itr_element++)
        {
            tris_->erase(tris_->begin() + TriangleByIndicesIndex::integer_type(*itr_element));
        }

        Cork::Files::writeOFF("../../UnitTest/Test Results/patch1.off", *this);

        TriangleByIndicesIndexVector all_indices(tris_->size(), 0u);
        std::iota(all_indices.begin(), all_indices.end(), 0u);

        auto patch_boundary_result = get_boundary_edge(all_indices);

        if (!patch_boundary_result.succeeded())
        {
            return;
        }

        std::cout << "Num Boundaries: " << patch_boundary_result.return_ptr()->size() << std::endl;

        std::vector<double> boundary_lengths;
        double max_length = -1;

        for (auto boundary : *(patch_boundary_result.return_ptr()))
        {
            boundary_lengths.emplace_back(boundary.length());

            max_length = std::max(max_length, boundary_lengths.back());
        }

        for (int i = 0; i < patch_boundary_result.return_ptr()->size(); i++)
        {
            if (boundary_lengths[i] >= max_length)
            {
                continue;
            }

            GetHoleClosingTrianglesResult get_hole_closing_result =
                close_hole((*(patch_boundary_result.return_ptr()))[i]);

            if (get_hole_closing_result.failed())
            {
                return;
            }

            for (auto tri : get_hole_closing_result.return_value().triangles_to_add_)
            {
                tris_->emplace_back(tri);
            }
        }

        Cork::Files::writeOFF("../../UnitTest/Test Results/patch2.off", *this);
    }

}  // namespace Cork::Meshes
