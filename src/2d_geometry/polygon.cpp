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

#include "polygon.hpp"

namespace Cork::TwoD
{
    inline double line_segment_cross_product(const Vertex2D& p1, const Vertex2D& p2, const Vertex2D& p3,
                                             const Vertex2D& p4)
    {
        Vertex2D p1p2 = p2 - p1;
        Vertex2D p3p4 = p4 - p3;

        return ((p1p2.x() * p3p4.y()) - (p3p4.x() * p1p2.y()));
    }

    inline bool Edge2D::intersects(const Edge2D& edge_to_check) const
    {
        double d1 = line_segment_cross_product(edge_to_check.v0_, v0_, edge_to_check.v0_, edge_to_check.v1_);
        double d2 = line_segment_cross_product(edge_to_check.v0_, v1_, edge_to_check.v0_, edge_to_check.v1_);
        double d3 = line_segment_cross_product(v0_, edge_to_check.v0_, v0_, v1_);
        double d4 = line_segment_cross_product(v0_, edge_to_check.v1_, v0_, v1_);

        bool intersection = ((d1 < 0 && d2 > 0) || (d1 > 0 && d2 < 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0));

        return intersection;
    }

    std::vector<IntersectingEdges2D> Polygon::self_intersections() const
    {
        std::vector<IntersectingEdges2D>     self_intersections;

        //  We do not need to worry about testing immediately adjacent segments as they cannot self intersect

        for (uint32_t i = 0; i < edges_.size() - 2; i++)
        {
            for (uint32_t j = i + 2; j < edges_.size() - 1; j++)
            {
                if (edges_[i].intersects( edges_[j]))
                {
                    self_intersections.emplace_back( edges_[i], edges_[j] );
                }
            }
        }

        //  Last and first segments are immediately adjacent, so we do not need to test them

        return self_intersections;
    }
}  // namespace Cork::TwoD
