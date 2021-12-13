// +-------------------------------------------------------------------------
// | mesh_builder.cpp
// |
// | Author: Stephan Friedl
// +-------------------------------------------------------------------------
// | COPYRIGHT:
// |    Copyright Stephan Friedl 2013
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

#include <map>
#include <set>
#include <unordered_map>

#include "file_formats/files.hpp"
#include "intersection/self_intersection_finder.hpp"
#include "intersection/triangulator.hpp"
#include "mesh/triangle_mesh_builder.hpp"
#include "primitives/remappers.hpp"
#include "statistics/statistics_engines.hpp"

namespace Cork::Meshes
{
    std::unique_ptr<TriangleMesh>        get_triangle_mesh_wrapper( MeshBase&& mesh_base );

    //
    //	IncrementalVertexIndexTriangleMeshBuilderImpl implements an incremental triangle mesh builder
    //		which can be used to construct a triangle mesh from a list of vertices and
    //		triangles assembled from those vertices.
    //

    class IncrementalVertexIndexTriangleMeshBuilderImpl : public IncrementalVertexIndexTriangleMeshBuilder
    {
       public:
        IncrementalVertexIndexTriangleMeshBuilderImpl(size_t num_vertices, size_t num_triangles)
            : mesh_(num_vertices, num_triangles)
        {
            vertex_index_remapper_.reserve(num_vertices);
        };

        ~IncrementalVertexIndexTriangleMeshBuilderImpl() = default;

        IncrementalVertexIndexTriangleMeshBuilderImpl() = delete;
        IncrementalVertexIndexTriangleMeshBuilderImpl(const IncrementalVertexIndexTriangleMeshBuilderImpl&) = delete;
        IncrementalVertexIndexTriangleMeshBuilderImpl(const IncrementalVertexIndexTriangleMeshBuilderImpl&&) = delete;

        IncrementalVertexIndexTriangleMeshBuilderImpl& operator=(const IncrementalVertexIndexTriangleMeshBuilderImpl&) =
            delete;
        IncrementalVertexIndexTriangleMeshBuilderImpl& operator=(
            const IncrementalVertexIndexTriangleMeshBuilderImpl&&) = delete;

        [[nodiscard]] size_t num_vertices() const final { return mesh_.num_vertices(); }

        [[nodiscard]] const Primitives::BBox3D& boundingBox() const { return mesh_.bounding_box(); }

        [[nodiscard]] double max_vertex_magnitude() const { return mesh_.max_vertex_magnitude(); }
        [[nodiscard]] Primitives::MinAndMaxEdgeLengths min_and_max_edge_lengths() const
        {
            return mesh_.min_and_max_edge_lengths();
        }

        VertexIndex add_vertex(const Primitives::Vertex3D& vertexToAdd) final
        {
            //	Add the vertex, de-duplicate on the fly.

            VertexIndexLookupMap::const_iterator vertexLoc = vertex_indices_.find(vertexToAdd);  //	NOLINT

            if (vertexLoc == vertex_indices_.end())
            {
                //	Vertex is new, update all data structures

                vertex_indices_[vertexToAdd] = mesh_.vertices().size();
                vertex_index_remapper_.push_back(VertexIndex::integer_type(mesh_.vertices().size()));
                mesh_.vertices().push_back(vertexToAdd);
            }
            else
            {
                //	Vertex is a duplicate, so remap to it

                vertex_index_remapper_.push_back(vertexLoc->second);
            }

            //	The index we return should always be the remapper size minus 1

            return (VertexIndex::integer_type(vertex_index_remapper_.size()) - 1u);
        }

        TriangleMeshBuilderResultCodes add_triangle(VertexIndex a, VertexIndex b, VertexIndex c) final
        {
            //	Insure the indices are in bounds

            if ((a >= vertex_index_remapper_.size()) || (b >= vertex_index_remapper_.size()) ||
                (c >= vertex_index_remapper_.size()))
            {
                return (TriangleMeshBuilderResultCodes::VERTEX_INDEX_OUT_OF_BOUNDS);
            }

            //	Remap the triangle indices

            TriangleByIndices remappedTriangle(mesh_.triangles().size(), vertex_index_remapper_[a],
                                               vertex_index_remapper_[b], vertex_index_remapper_[c]);

            //	Add the triangle to the vector

            mesh_.add_triangle_and_update_metrics(remappedTriangle);

            //	All is well if we made it here

            return (TriangleMeshBuilderResultCodes::SUCCESS);
        }

        std::unique_ptr<TriangleMesh> mesh() final
        {
            std::unique_ptr<TriangleMesh> return_value( get_triangle_mesh_wrapper(std::move(mesh_)).release() );

            mesh_.clear();

            return return_value;
        }

       private:
        using VertexIndexLookupMap = std::map<Primitives::Vertex3D, IndexType, Primitives::Vertex3DMapCompare>;

        MeshBase mesh_;

        VertexIndexLookupMap vertex_indices_;
        Primitives::IndexRemapper<VertexIndex> vertex_index_remapper_;
    };

    //
    //	Factory method to return an incremental triangle mesh builder
    //

    std::unique_ptr<IncrementalVertexIndexTriangleMeshBuilder> IncrementalVertexIndexTriangleMeshBuilder::GetBuilder(
        size_t numVertices, size_t numTriangles)
    {
        return (std::unique_ptr<IncrementalVertexIndexTriangleMeshBuilder>(
            new IncrementalVertexIndexTriangleMeshBuilderImpl(numVertices, numTriangles)));
    }

}  // namespace Cork::Meshes
