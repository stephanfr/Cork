// +-------------------------------------------------------------------------
// | mesh_base.cpp
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

#include "mesh_base.hpp"

#include <numeric>

#include "intersection/triangulator.hpp"
#include "boundary_edge_builder.hpp"
#include "primitives/remappers.hpp"

namespace Cork::Meshes
{
    MeshBase::MeshBase(MeshBase&& mesh_base_to_move)
        : bounding_box_(mesh_base_to_move.bounding_box_),
          min_and_max_edge_lengths_(mesh_base_to_move.min_and_max_edge_lengths_),
          max_vertex_magnitude_(mesh_base_to_move.max_vertex_magnitude_),
          tris_(std::move(mesh_base_to_move.tris_)),
          verts_(std::move(mesh_base_to_move.verts_))
    {
        mesh_base_to_move.clear();
    }

    MeshBase::MeshBase(size_t num_vertices, size_t num_triangles)
        : tris_(new TriangleByIndicesVector()),
          verts_(new Vertex3DVector()),
          max_vertex_magnitude_(NUMERIC_PRECISION_MIN)
    {
        tris_->reserve(num_triangles);
        verts_->reserve(num_vertices);
    }

    MeshBase::MeshBase(std::shared_ptr<TriangleByIndicesVector>& triangles, std::shared_ptr<Vertex3DVector>& vertices,
                       const Primitives::BBox3D& boundingBox,
                       const Primitives::MinAndMaxEdgeLengths min_and_max_edge_lengths, double max_vertex_magnitude)
        : tris_(triangles),
          verts_(vertices),
          bounding_box_(boundingBox),
          min_and_max_edge_lengths_(min_and_max_edge_lengths),
          max_vertex_magnitude_(max_vertex_magnitude)
    {
    }

    void MeshBase::clear()
    {
        tris_.reset();
        verts_.reset();

        bounding_box_ = BBox3D();
        max_vertex_magnitude_ = NUMERIC_PRECISION_MIN;
        min_and_max_edge_lengths_ = MinAndMaxEdgeLengths();
    }

    MeshBase MeshBase::clone() const
    {
        auto copy_of_tris{std::make_shared<TriangleByIndicesVector>(*tris_)};
        auto copy_of_verts{std::make_shared<Vertex3DVector>(*verts_)};

        return MeshBase(copy_of_tris, copy_of_verts, bounding_box_, min_and_max_edge_lengths_, max_vertex_magnitude_);
    }

    std::unique_ptr<MeshBase> MeshBase::extract_surface(TriangleRemapper& remapper,
                                                        TriangleByIndicesIndex center_triangle,
                                                        uint32_t num_rings) const
    {
        Cork::Primitives::TriangleByIndicesIndexSet single_triangle;

        single_triangle.emplace(center_triangle);

        auto find_enclosing_triangles_result = find_enclosing_triangles(single_triangle, num_rings);

        if (!find_enclosing_triangles_result.succeeded())  //  TODO return proper success/failure result
        {
            return std::unique_ptr<MeshBase>();
        }

        return extract_surface(remapper, find_enclosing_triangles_result.return_ptr()->merge(single_triangle));
    }

    std::unique_ptr<MeshBase> MeshBase::extract_surface(TriangleRemapper& remapper,
                                                        const TriangleByIndicesVector& tris_to_extract) const
    {
        return remapper.extract_surface(tris_to_extract);
    }

    std::unique_ptr<MeshBase> MeshBase::extract_surface(TriangleRemapper& remapper,
                                                        const TriangleByIndicesIndexSet& tris_to_extract) const
    {
        return remapper.extract_surface(tris_to_extract);
    }

    GetHoleClosingTrianglesResult MeshBase::close_hole(const BoundaryEdge& hole)
    {
        //  Start by obtaining the best fit plane normal for the boundary point.  We will project the
        //      boundary on to this plane before triangulating.  It will *usually* be an excellent projection
        //      surface but sometimes it is possible that the plane will be off by say 90 degrees if the hole
        //      is folded.  In that case we will end up with a lot of self intersections.

        Vertex3D boundary_centroid = hole.centroid();
        BestFitPlaneEquation best_fit_plane = hole.best_fit_plane();

        auto projected_hole = hole.project(best_fit_plane.unit_normal(), boundary_centroid);

        auto self_intersections = projected_hole.self_intersections();

        if (self_intersections.size() > 0)
        {

            return GetHoleClosingTrianglesResult::failure(HoleClosingResultCodes::HOLE_BOUNDARY_SELF_INTERSECTS,
                                                          "Hole boundary hsa self intersections");
        }

        //  Get the triangulator and add the points on the hole edge and the segments joining them.
        //      This is trivial as the vertices are ordered so segments are just one after the next.
        //      Holes must be closed, thus the last segment from the last vertex to the first.

        Triangulator::Triangulator triangulator;

        triangulator.add_point(projected_hole.edges().front().v0(), true);

        for (uint32_t i = 0; i < projected_hole.edges().size() - 1; i++)
        {
            triangulator.add_point(projected_hole.edges()[i].v1(), true);
        }

        for (int i = 0; i < hole.vertex_indices().size() - 1; i++)
        {
            triangulator.add_segment(i, i + 1, true);
        }

        triangulator.add_segment(hole.vertex_indices().size() - 1, 0, true);

        //  Compute the triangulation.  We should not be here unless we have a good projection, so this
        //      step should not fail ever.

        auto result = triangulator.compute_triangulation();

        if (result.failed())
        {
            return GetHoleClosingTrianglesResult::failure(result, HoleClosingResultCodes::TRIANGULATION_FAILED,
                                                          "Triangulation failed for hole");
        }

        //  Add the new triangles which close the hole.  There will never be new vertices to add based
        //      on the settings of the triangulator - thus this operation is simple.

        TriangleByIndicesVector hole_closing_triangles;

        hole_closing_triangles.reserve(result.return_ptr()->size() + 2);

        for (auto triangle_to_add : *(result.return_ptr()))
        {
            hole_closing_triangles.emplace_back(TriangleByIndices(
                Primitives::UNINITIALIZED_INDEX, hole.vertex_indices()[triangle_to_add.v0()],
                hole.vertex_indices()[triangle_to_add.v2()], hole.vertex_indices()[triangle_to_add.v1()]));
        }

        //  Return the triangles to close the hole

        HoleClosingSolution     hole_solution( std::move(hole_closing_triangles),std::move(VertexIndexVector()) );

        return GetHoleClosingTrianglesResult::success( std::move( hole_solution ));
    }

    void MeshBase::compact()
    {
        //  Removes unreferenced vertices and remaps triangles

        //  Start by identifying the vertices that are still in use

        BooleanVector<VertexIndex> live_verts(verts_->size(), false);

        for (const auto& tri : *tris_)
        {
            live_verts[tri.a()] = true;
            live_verts[tri.b()] = true;
            live_verts[tri.c()] = true;
        }

        //  Next, remap those verts - this will end up with vertices getting smaller indices as unused
        //      vertices are skipped.

        Primitives::IndexRemapper<VertexIndex> vertex_remapper;

        vertex_remapper.reserve(verts_->size());

        VertexIndex new_index{0};

        for (VertexIndex i{0}; i < live_verts.size(); i++)
        {
            if (live_verts[i])
            {
                vertex_remapper[i] = new_index;
                (*verts_)[new_index] = (*verts_)[i];
                new_index++;
            }
        }

        verts_->resize(static_cast<size_t>(new_index));

        //  Remap the triangles

        TriangleUID tri_uid = 0U;

        for (auto& tri : *tris_)
        {
            tri = TriangleByIndices(tri_uid++, vertex_remapper[tri.a()], vertex_remapper[tri.b()],
                                    vertex_remapper[tri.c()]);
        }

        //  Invalidate the topo cache

        topo_cache_.release();

        //  Done
    }

    ExtractBoundariesResult MeshBase::get_boundary_edge(const TriangleByIndicesIndexSet& tris_to_outline) const
    {
        return BoundaryEdgeBuilder(*this).extract_boundaries(tris_to_outline);
    }

    ExtractBoundariesResult MeshBase::get_boundary_edge(const TriangleByIndicesIndexVector& tris_to_outline) const
    {
        return BoundaryEdgeBuilder(*this).extract_boundaries(tris_to_outline);
    }

    FindEnclosingTrianglesResult MeshBase::find_enclosing_triangles(const TriangleByIndicesIndexVector& triangles,
                                                                    uint32_t num_layers) const
    {
        TriangleByIndicesIndexSet tris(triangles.begin(), triangles.end());

        return find_enclosing_triangles(tris, num_layers);
    }

    FindEnclosingTrianglesResult MeshBase::find_enclosing_triangles(const TriangleByIndicesIndexSet& interior_triangles,
                                                                    uint32_t num_layers) const
    {
        ExtractBoundariesResult get_boundaries_result = get_boundary_edge(interior_triangles);

        if (!get_boundaries_result.succeeded())
        {
            return FindEnclosingTrianglesResult::failure(
                get_boundaries_result, FindEnclosingTrianglesResultCodes::UNABLE_TO_RECOMPUTE_BOUNDARY,
                "Unable to recompute boundaries");
        }

        //  Occasionally we may end up with multiple boundaries - we will just find all the
        //      enclosing triangles for however many boundaries we have and press on.

        std::unique_ptr<std::vector<BoundaryEdge>> boundaries(get_boundaries_result.return_ptr().release());

        if (boundaries->size() == 1)
        {
            return find_enclosing_triangles((*boundaries)[0], interior_triangles, num_layers);
        }

        auto union_of_all_boundaries = std::make_unique<TriangleByIndicesIndexSet>();

        for (const BoundaryEdge& current_boundary : *boundaries)
        {
            auto result = find_enclosing_triangles(current_boundary, interior_triangles, 1);

            if (!result.succeeded())
            {
                return result;
            }

            union_of_all_boundaries->merge(*result.return_ptr());
        }

        return FindEnclosingTrianglesResult::success(std::move(union_of_all_boundaries));
    }

    FindEnclosingTrianglesResult MeshBase::find_enclosing_triangles(const BoundaryEdge& boundary,
                                                                    const TriangleByIndicesIndexSet& interior_triangles,
                                                                    uint32_t num_layers) const
    {
        auto current_boundaries = std::make_unique<std::vector<BoundaryEdge>>();
        auto all_enclosing_triangles = std::make_unique<TriangleByIndicesIndexSet>();

        current_boundaries->emplace_back(boundary);

        for (int i = 0; i < num_layers; i++)
        {
            TriangleByIndicesIndexSet enclosing_triangles;

            for (const auto& boundary : *current_boundaries)
            {
                TopoEdgeBoundary topo_edges(std::move(topo_cache().topo_edge_boundary(boundary)));
                std::set<const TopoTri*> tris_on_edge(std::move(topo_cache().tris_along_edges(topo_edges)));

                for (auto next_tri : tris_on_edge)
                {
                    if (!interior_triangles.contains(next_tri->ref()))
                    {
                        enclosing_triangles.insert(next_tri->ref());
                    }
                }
            }

            //  Merge all the triangles into one master set

            all_enclosing_triangles->merge(enclosing_triangles);

            //  Recompute the boundary if there is another enclosing layer to add

            if (i < num_layers - 1)
            {
                TriangleByIndicesIndexSet all_interior_triangles(interior_triangles, *all_enclosing_triangles);

                auto result = get_boundary_edge(all_interior_triangles);

                if (!result.succeeded())
                {
                    return FindEnclosingTrianglesResult::failure(
                        result, FindEnclosingTrianglesResultCodes::UNABLE_TO_RECOMPUTE_BOUNDARY,
                        "Unable to recompute boundary");
                }

                current_boundaries.reset(result.return_ptr().release());
            }
        }

        return FindEnclosingTrianglesResult::success(std::move(all_enclosing_triangles));
    }

    inline std::optional<TriangleByIndicesIndex> MeshBase::tri_containing_all_three_vertices(VertexIndex vert1,
                                                                                             VertexIndex vert2,
                                                                                             VertexIndex vert3) const
    {
        std::vector<const TopoTri*> tris_containing_1_and_2;

        std::set_intersection(topo_cache().vertices().getPool()[vert1].triangles().begin(),
                              topo_cache().vertices().getPool()[vert1].triangles().end(),
                              topo_cache().vertices().getPool()[vert2].triangles().begin(),
                              topo_cache().vertices().getPool()[vert2].triangles().end(),
                              std::back_inserter(tris_containing_1_and_2));

        std::vector<const TopoTri*> tris_containing_all_3;

        std::set_intersection(tris_containing_1_and_2.begin(), tris_containing_1_and_2.end(),
                              topo_cache().vertices().getPool()[vert3].triangles().begin(),
                              topo_cache().vertices().getPool()[vert3].triangles().end(),
                              std::back_inserter(tris_containing_all_3));

        std::optional<TriangleByIndicesIndex> result;

        if (!tris_containing_all_3.empty())
        {
            //  There should only ever be one ....
            result = (*(tris_containing_all_3.begin()))->ref();
        }

        return result;
    }

}  // namespace Cork::Meshes
