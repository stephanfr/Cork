

#include <array>
#include <cstddef>

#define REAL double  //	This define is for triangle.h below

extern "C"
{
#include "intersection/triangle.h"
}

#include "intersection/triangulator.hpp"

namespace Cork::Triangulator
{
    //  The following template classes are needed to insure the correct delete function is associated
    //      with the allocation function used for elements passed to the Triangle library.

    template <typename T>
    struct TriangulateDeleter
    {
        void operator()(T* pointer) { trifree(pointer); }
    };

    template <typename T>
    struct FreeDeleter
    {
        void operator()(T* pointer) { free(pointer); }
    };

    class TriangulatorImpl : public TriangulatorIfx
    {
       public:
        static constexpr size_t MAX_EDGE_POINTS = 4096;

        TriangulatorImpl() : number_of_points_(0), number_of_segments_(0){};
        TriangulatorImpl(const TriangulatorImpl&) = delete;
        TriangulatorImpl(TriangulatorImpl&&) = delete;

        ~TriangulatorImpl() = default;

        TriangulatorImpl& operator=(const TriangulatorImpl&) = delete;
        TriangulatorImpl& operator=(TriangulatorImpl&&) = delete;

        void add_point(Point point)
        {
            points_[number_of_points_] = point.pair();
            point_markers_[number_of_points_++] = point.boundary() ? 1 : 0;
        }

        void add_point(double x, double y, bool boundary)
        {
            points_[number_of_points_] = std::pair<double,double>(x, y );
            point_markers_[number_of_points_++] = boundary;
        }


        void add_segment(Segment segment)
        {
            segments_[number_of_segments_] = segment.pair();
            segment_markers_[number_of_segments_++] = segment.boundary() ? 1 : 0;
        }

        void add_segment(uint32_t start, uint32_t end, bool boundary)
        {
            segments_[number_of_segments_] = std::pair<int,int>(start,end);
            segment_markers_[number_of_segments_++] = boundary ? 1 : 0;
        }


        TriangulateResult compute_triangulation();

       private:
        uint32_t number_of_points_;
        uint32_t number_of_segments_;

        std::array<std::pair<double, double>, MAX_EDGE_POINTS> points_;
        std::array<int, MAX_EDGE_POINTS> point_markers_;
        std::array<std::pair<int, int>, MAX_EDGE_POINTS> segments_;
        std::array<int, MAX_EDGE_POINTS> segment_markers_;
    };

    TriangulateResult TriangulatorImpl::compute_triangulation()
    {
        struct triangulateio in, out;

        in.numberofpoints = (int)number_of_points_;
        in.numberofpointattributes = 0;

        in.pointlist = reinterpret_cast<double*>(&points_);
        in.pointmarkerlist = reinterpret_cast<int*>(&point_markers_);
        in.pointattributelist = nullptr;

        //  Define the segments linking the points into the edge of the region to be meshed

        in.numberofsegments = (int)number_of_segments_;
        in.numberofholes = 0;    // No holes
        in.numberofregions = 0;  // Not using regions

        in.segmentlist = reinterpret_cast<int*>(&segments_);
        in.segmentmarkerlist = reinterpret_cast<int*>(&segment_markers_);

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

        char* params = (char*)("jpzQYY");

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
            result->emplace_back( out.trianglelist[(k * 3) + 0], out.trianglelist[(k * 3) + 1],  out.trianglelist[(k * 3) + 2] );
        }

        return TriangulateResult::success(std::move(result));
    }

    std::unique_ptr<TriangulatorIfx> TriangulatorIfx::get_triangulator()
    {
        return std::unique_ptr<TriangulatorIfx>(new TriangulatorImpl());
    }

}  // namespace Cork::Triangulator
