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
#include "math/normal_projector.hpp"
#include "mesh/edge_incidence_counter.hpp"

namespace Cork::Meshes
{
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
        tris_ = src.tris_;
        verts_ = src.verts_;
    }

    void SurfaceMesh::scrub_surface()
    {
        find_outside_boundary();

        TriangleByIndicesIndexVector all_indices(tris_->size(), 0u);
        std::iota(all_indices.begin(), all_indices.end(), 0u);

        auto patch_boundary_result = get_boundary_edge(all_indices);

        uint32_t num_verts = 0;

        for (auto boundary : *(patch_boundary_result.return_ptr()))
        {
            num_verts += boundary.vertex_indices().size();
        }

        std::cout << "Found: " << patch_boundary_result.return_ptr()->size()
                  << " boundaries with total of: " << num_verts << " vertices." << std::endl;

        EdgeIncidenceSet edge_incidences = EdgeIncidenceCounter(*this).edges_and_incidences();

        std::unordered_set<EdgeByIndices, EdgeByIndices::HashFunction> non_manifold_edges;

        for (auto incidence : edge_incidences)
        {
            if (incidence.numIncidences() != 2)
            {
                non_manifold_edges.insert(incidence);
            }
        }

        std::cout << "Found: " << non_manifold_edges.size() << " non manifold edges." << std::endl;
    }

    BoundaryEdge SurfaceMesh::find_outside_boundary()
    {
        TriangleByIndicesIndexVector all_indices(tris_->size(), 0u);
        std::iota(all_indices.begin(), all_indices.end(), 0u);

        auto patch_boundary_result = get_boundary_edge(all_indices);

        auto topo_edge_boundary = topo_cache().topo_edge_boundary(patch_boundary_result.return_ptr()->front());

        std::unordered_set<const TopoEdge*>     all_boundary_edges_found;
        std::unordered_set<const TopoTri*>      tris_inside_boundary;
        std::vector<const TopoTri*>             tris_to_process;

        tris_to_process.emplace_back( topo_edge_boundary.front()->triangles().front() );

        while( !tris_to_process.empty() )
        {
            const TopoTri*      current_tri = tris_to_process.back();
            tris_to_process.pop_back();

            tris_inside_boundary.insert( current_tri );

            for( auto current_edge : current_tri->edges() )
            {
                if( current_edge->triangles().size() == 1 )
                {
                    all_boundary_edges_found.insert( current_edge );
                }
                else if( current_edge->triangles().size() == 2 )
                {
                    if( current_edge->triangles()[0] != current_tri )
                    {
                        tris_to_process.push_back( current_edge->triangles()[0] );
                    }
                    else
                    {
                        tris_to_process.push_back( current_edge->triangles()[1] );
                    }
                }
                else
                {
                    std::cout << "Found edge with 3 intersecting triangles" << std::endl;       //  TODO    Fix this !!
                }
            }
        }

        std::cout << "Found: " << all_boundary_edges_found.size() << " boundary edges enclosing: " << tris_inside_boundary.size() << std::endl;

/*
        MeshBase projected_mesh(verts_->size(), tris_->size());
        Math::NormalProjector projector(triangle_by_vertices((*tris_)[TriangleByIndicesIndex(0u)]));

        for (auto vert : *verts_)
        {
            Vertex2D projected_vertex = projector.project(vert);

            projected_mesh.verts_->emplace_back(projected_vertex.x(), projected_vertex.y(), 0);
        }

        for (auto tri : *tris_)
        {
            projected_mesh.add_triangle_and_update_metrics(tri);
        }

        Cork::Files::writeOFF("../../UnitTest/Test Results/projected_patch.off", projected_mesh);
*/
        return (*patch_boundary_result.return_ptr())[0];
    }

    TriangleByIndicesIndexSet       all_tris_in_boundary( const BoundaryEdge&   boundary )
    {
        return TriangleByIndicesIndexSet();
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

        //  Try expanding the enclosing region around the self intersecting triangle.  If the triangle is on the main
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
            boundary_lengths.emplace_back(boundary.length(vertices()));

            max_length = std::max(max_length, boundary_lengths.back());
        }

        for (int i = 0; i < patch_boundary_result.return_ptr()->size(); i++)
        {
            if (boundary_lengths[i] >= max_length)
            {
                continue;
            }

            GetHoleClosingTrianglesResult get_hole_closing_result =
                get_hole_closing_triangles((*(patch_boundary_result.return_ptr()))[i]);

            if (get_hole_closing_result.failed())
            {
                return;
            }

            for (auto tri : *(get_hole_closing_result.return_ptr()))
            {
                tris_->emplace_back(tri);
            }
        }

        Cork::Files::writeOFF("../../UnitTest/Test Results/patch2.off", *this);
    }

}  // namespace Cork::Meshes
