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

#include <vector>

#include "primitives/primitives.hpp"

namespace Cork::Meshes
{
    class BoundaryEdgeBuilder;
}

namespace Cork::Primitives
{

    class BoundaryEdge
    {
       public:
        BoundaryEdge(const std::vector<VertexIndex>& vertices) : vertex_indices_(vertices) {}
        BoundaryEdge(const BoundaryEdge&) = default;

        BoundaryEdge& operator=(const BoundaryEdge&) = default;

        const std::vector<VertexIndex>& vertex_indices() const { return vertex_indices_; }

        double  length( const Vertex3DVector&       vertices ) const
        {
            double  len = 0;

            for( int i = 0; i < vertex_indices_.size() - 1; i++ )
            {
                Vector3D    segment = vertices[vertex_indices_[i]] - vertices[vertex_indices_[i+1]];

                len += sqrt(( segment.x() * segment.x() ) + ( segment.y() * segment.y() ) + ( segment.z() * segment.z() )); 
            }

            return len;
        }

        BBox3D      bounding_box( const Vertex3DVector&       vertices ) const
        {
            BBox3D      bounding_box;

            for( auto current_index : vertex_indices_ )
            {
                bounding_box.convex( vertices[current_index] );
            }

            return bounding_box;
        }

       private:
        std::vector<VertexIndex> vertex_indices_;

        friend class Meshes::BoundaryEdgeBuilder;
    };

    std::ostream& operator<<(std::ostream& out, const BoundaryEdge& boundary);

}  // namespace Cork::Primitives

namespace Cork
{
    using BoundaryEdge = Primitives::BoundaryEdge;
}  // namespace Cork
