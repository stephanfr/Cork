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

#include "primitives/primitives.hpp"

namespace Cork::Math
{
    template <typename T>
    Vertex3D centroid(const T& itr_begin, const T& itr_end)
    {
        Vertex3D centroid(0, 0, 0);
        uint32_t num_vertices = 0;

        for (auto itr = itr_begin; itr != itr_end; itr++)
        {
            centroid += *itr;
            num_vertices++;
        }

        centroid /= num_vertices;

        return centroid;
    }

    class PlaneEquation
    {
       public:
        PlaneEquation(const Vector3D& unit_normal, const Vertex3D& centroid)
            : unit_normal_(unit_normal), centroid_(centroid){};

        PlaneEquation(Vector3D&& unit_normal, Vertex3D&& centroid)
            : unit_normal_(std::move(unit_normal)), centroid_(std::move(centroid)){};

        const Vector3D& unit_normal() const { return unit_normal_; }

        const Vertex3D& centroid() const { return centroid_; }

        double distance(Vertex3D point) const { return unit_normal_.dot(point - centroid_); }


       protected:
        PlaneEquation() = default;

        Vector3D unit_normal_;
        Vertex3D centroid_;
    };



    class BestFitPlaneEquation : public PlaneEquation
    {
       public:
        BestFitPlaneEquation(const Vector3D& unit_normal, const Vertex3D& centroid) = delete;
        BestFitPlaneEquation(Vector3D&& unit_normal, Vertex3D&& centroid) = delete;

//        template <typename T>
        BestFitPlaneEquation(uint32_t num_verts, Vector3DVector::const_iterator itr_begin, Vector3DVector::const_iterator itr_end);
/*        {
            //  We will use SVD to determine the best-fit plane.  Since we are using a Nx3 matrix,
            //      the normal to the best-fit plane will be the right singular vector associated
            //      with the smallest singular value.

            centroid_ = Math::centroid(itr_begin, itr_end);

            Eigen::MatrixXd A(num_verts, 3);

            int k = 0;
            for (auto itr = itr_begin; itr != itr_end; itr++)
            {
                Vector3D translated_point = *itr - centroid_;

                A(k, 0) = translated_point.x();
                A(k, 1) = translated_point.y();
                A(k, 2) = translated_point.z();

                k++;
            }

            Eigen::BDCSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);

            //  The best-fit normal is the V column whose index is given by the index of the min svd value.
            //      ONly three to check - not worth a loop.

            double min_value = svd.singularValues()[0];
            uint32_t min_index = 0;

            if (svd.singularValues()[1] < min_value)
            {
                min_value = svd.singularValues()[1];
                min_index = 1;
            }

            if (svd.singularValues()[2] < min_value)
            {
                min_value = svd.singularValues()[2];
                min_index = 2;
            }

            //  Pull the values and return them in a Vector3D

            unit_normal_ =
                Vector3D(svd.matrixV()(0, min_index), svd.matrixV()(1, min_index), svd.matrixV()(2, min_index));

            //  Compute the rms error

            rms_error_ = 0;

            for (auto itr = itr_begin; itr != itr_end; itr++)
            {
                double dist = distance( *itr );

                rms_error_ += dist * dist;
            }

            rms_error_ = sqrt( rms_error_ );
        }
*/

        double  rms_error() const{
            return rms_error_;
        }

       private:
        double  rms_error_;
    };

/*
        double  distance_from_point_to_plane( Vertex3D      point,
                                              Vector3D      plane_normal,
                                              Vertex3D      normal_intersection_with_plane )
                                              {
                                                  distance_from_point_to_plane_at_origin( point -
       normal_intersection_with_plane, plane_normal );
                                              }


        double  distance_from_point_to_plane_at_origin( Vertex3D      point,
                                              Vector3D      plane_normal )
                                              {
                                                  return(( plane_normal.dot( point ) /   )
                                              }
*/
    
};  // namespace Cork::Math

namespace Cork
{
    using PlaneEquation = Cork::Math::PlaneEquation;
    using BestFitPlaneEquation = Cork::Math::BestFitPlaneEquation;
};  // namespace Cork
