// +-------------------------------------------------------------------------
// | surface_mesh.hpp
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
#pragma once

#include "mesh/edge_graph_cache.hpp"
#include "mesh_base.hpp"
#include "primitives/self_intersecting_edge.hpp"
#include "tbb/concurrent_vector.h"

namespace Cork::Meshes
{
    //
    //	The Mesh class brings together the functionality needed for the boolean operations
    //

    class TopoEdgeBoundaryAndHoles
    {
       public:
        TopoEdgeBoundaryAndHoles(TopoEdgeBoundary&& outside_boundary, TriangleByIndicesIndexSet&& tris_inside_boundary,
                                    std::vector<TopoEdgeBoundary>&& holes)
            : outside_boundary_(std::move(outside_boundary)),
              tris_inside_boundary_(std::move(tris_inside_boundary)),
              holes_(std::move(holes))
        {
        }

        TopoEdgeBoundary outside_boundary_;
        TriangleByIndicesIndexSet tris_inside_boundary_;

        std::vector<TopoEdgeBoundary> holes_;
    };

    using FindTopoEdgeBoundariesResult =
        SEFUtility::ResultWithReturnValue<FindTopoEdgeBoundariesResultCodes, TopoEdgeBoundaryAndHoles>;

    class SurfaceMesh : public MeshBase
    {
       public:
        SurfaceMesh(SurfaceMesh&& src) : MeshBase(std::move(src)){};

        SurfaceMesh(MeshBase&& src) : MeshBase(std::move(src)){};

        explicit SurfaceMesh(const TriangleMesh& inputMesh);

        virtual ~SurfaceMesh();

        void operator=(SurfaceMesh&& src);

        void remove_triangle( TriangleByIndicesIndex    index )
        {
            tris_->erase( tris_->begin() + TriangleByIndicesIndex::integer_type( index ) );

            clear_topo_cache();
        }

        Vector3D    centroid() const
        {
            if( centroid_ )
            {
                return centroid_.value();
            }

            Vector3D        centroid_temp( 0, 0, 0 );

            for( auto current_vert : *verts_ )
            {
                centroid_temp += current_vert;
            }

            const_cast<std::optional<Vector3D>*>(&centroid_)->emplace( centroid_temp / verts_->size() );

            return centroid_.value();
        }

        BestFitPlaneEquation    best_fit_plane() const;

        std::unique_ptr<SurfaceMesh>        project_surface( const Vector3D     projection_surface_normal,
                                                             const Vertex3D     normal_surface_origin );

        FindTopoEdgeBoundariesResult find_topo_edge_boundaries();
        FindTopoEdgeBoundariesResult find_topo_edge_boundaries( const TopoTri*      seed_traingle );

        void remove_self_intersections();

        void remove_self_intersection(const SelfIntersectingEdge& self_intersection);

       protected:
        SurfaceMesh() = default;

        std::optional<Vector3D>     centroid_;
    };

    class ExtractedSurfaceMesh : public SurfaceMesh
    {
       public:
        ExtractedSurfaceMesh(const MeshBase& parent_mesh, const TriangleByIndicesIndexSet& tris_to_extract)
            : parent_mesh_(parent_mesh), remapper_(parent_mesh)

        {
            std::unique_ptr<MeshBase> patch = extract_surface(remapper_, tris_to_extract);

            tris_ = std::move(patch->tris_);
            verts_ = std::move(patch->verts_);

            bounding_box_ = std::move(patch->bounding_box_);

            min_and_max_edge_lengths_ = std::move(patch->min_and_max_edge_lengths_);
            max_vertex_magnitude_ = std::move(patch->max_vertex_magnitude_);

            topo_cache_ = std::move(patch->topo_cache_);
        }

       private:
        const MeshBase& parent_mesh_;

        TriangleRemapper remapper_;
    };

}  // namespace Cork::Meshes