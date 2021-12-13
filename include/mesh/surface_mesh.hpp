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
#pragma once

#include "mesh/mesh_base.hpp"

namespace Cork::Meshes
{
    class SurfaceMesh : public MeshBase
    {
       public:
        SurfaceMesh(MeshBase&& mesh_base_to_move) : MeshBase(std::move(mesh_base_to_move)) {}

        virtual ~SurfaceMesh() {}

        static std::unique_ptr<SurfaceMesh> extract_surface(const MeshBase& mesh,
                                                            TriangleByIndicesIndex  center_triangle,
                                                            uint32_t                num_rings );

        static std::unique_ptr<SurfaceMesh> extract_surface(const MeshBase& mesh,
                                                            const TriangleByIndicesVector& tris_to_extract)
        {
            return std::make_unique<SurfaceMesh>(std::move(*(mesh.extract_surface(tris_to_extract).release())));
        }

        static std::unique_ptr<SurfaceMesh> extract_surface(const MeshBase& mesh,
                                                            const TriangleByIndicesIndexSet& tris_to_extract)
        {
            return std::make_unique<SurfaceMesh>(std::move(*(mesh.extract_surface(tris_to_extract).release())));
        }

//        bool    is_surface_manifold() const
//        {
//            EdgeIncidenceCounter        edge_incidences( *this );
//
//            BoundaryEdgeBuilder         boundary_builder();
//
//
//        }

       private:
        SurfaceMesh() = default;
    };

}  // namespace Cork::Meshes