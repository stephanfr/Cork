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

#include "triangulator.hpp"

#include <cstddef>

//  NOLINTNEXTLINE( cppcoreguidelines-macro-usage )
#define REAL double  //	This define is for triangle.h below

extern "C"
{
    #include "triangle.h"
}



namespace Cork::Triangulator
{
    //  The following template classes are needed to insure the correct delete function is associated
    //      with the allocation function used for elements passed to the Triangle library.

    template <typename T>
    struct TriangulateDeleter
    {
        void operator()(T* pointer) { trifree(pointer); }
    };

    TriangulateResult Triangulator::compute_triangulation()
    {
        struct triangulateio in;                    //  NOLINT(cppcoreguidelines-pro-type-member-init)
        struct triangulateio out;                   //  NOLINT(cppcoreguidelines-pro-type-member-init)

        in.numberofpoints = (int)points_.size();
        in.numberofpointattributes = 0;

        in.pointlist = reinterpret_cast<double*>(points_.data());                     //  NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        in.pointmarkerlist = point_markers_.data();
        in.pointattributelist = nullptr;

        //  Define the segments linking the points into the edge of the region to be meshed

        in.numberofsegments = (int)segments_.size();
        in.numberofholes = 0;    // No holes
        in.numberofregions = 0;  // Not using regions

        in.segmentlist = reinterpret_cast<int*>(segments_.data());                    //  NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        in.segmentmarkerlist = segment_markers_.data();

        //  No number of triangles so also no triangle attributes

        in.numberoftriangles = 0;
        in.numberoftriangleattributes = 0;

        //  Clear the output data structure

        out.pointlist = nullptr;
        out.pointattributelist = nullptr;  // not necessary if using -N or 0 attr
        out.pointmarkerlist = nullptr;
        out.trianglelist = nullptr;       // not necessary if using -E
        out.segmentlist = nullptr;        // NEED THIS; output segments go here
        out.segmentmarkerlist = nullptr;  // NEED THIS for OUTPUT SEGMENTS

        //	Solve the triangulation problem to get the orientation of the triangles correct.
        //
        //		The number of output points should always equal the number of input points.  I added the
        //		'j' option to cause triangle to jettison verticies not present in the final triangulation,
        //		which is usually the result of duplicated input vertices for our specific case in Cork.
        //		If extra points appear, then something has gone wrong

        char* params = const_cast<char*>("jpzQYY");                 //  NOLINT( cppcoreguidelines-pro-type-const-cast )

        triangulate(params, &in, &out, nullptr);

        std::unique_ptr<REAL, TriangulateDeleter<REAL>> outPointList(out.pointlist);
        std::unique_ptr<REAL, TriangulateDeleter<REAL>> outPointAttributeList(out.pointattributelist);
        std::unique_ptr<int, TriangulateDeleter<int>> outPointMarkerList(out.pointmarkerlist);
        std::unique_ptr<int, TriangulateDeleter<int>> outTriangleList(out.trianglelist);
        std::unique_ptr<int, TriangulateDeleter<int>> outSegmentList(out.segmentlist);
        std::unique_ptr<int, TriangulateDeleter<int>> outSegmentMarkerList(out.segmentmarkerlist);

        if (out.numberofpoints != in.numberofpoints)
        {
            //	When we end up here, it is usually because we have hit some self-intersections.

            return TriangulateResult::failure(
                TriangulationResultCodes::UNEQUAL_NUMBER_OF_INPUT_AND_OUTPUT_POINTS,
                "Unequal number of points before and after triangulation - check input meshes for self "
                "intersections.");
        }

        std::unique_ptr<TriangleList> result(new TriangleList());

        result->reserve(out.numberoftriangles);

        for (int k = 0; k < out.numberoftriangles; k++)
        {
            result->emplace_back(out.trianglelist[(k * 3) + 0], out.trianglelist[(k * 3) + 1], out.trianglelist[(k * 3) + 2]);
        }

        return TriangulateResult::success(std::move(result));
    }

}  // namespace Cork::Triangulator
