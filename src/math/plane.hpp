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

    class PlaneEquationBase
    {
        public :

        [[nodiscard]] virtual const Vector3D& unit_normal() const = 0;

        [[nodiscard]] virtual const Vertex3D& centroid() const = 0;

        [[nodiscard]] double distance(Vertex3D point) const { return unit_normal().dot(point - centroid()); }
    };

    class PlaneEquation : public PlaneEquationBase
    {
       public:

        PlaneEquation() = delete;


        PlaneEquation(const Vector3D& unit_normal, const Vertex3D& centroid)
            : unit_normal_(unit_normal), centroid_(centroid){};

        PlaneEquation(Vector3D&& unit_normal, Vertex3D&& centroid)
            : unit_normal_(unit_normal), centroid_(centroid){};

        [[nodiscard]] const Vector3D& unit_normal() const override { return unit_normal_; }

        [[nodiscard]] const Vertex3D& centroid() const override { return centroid_; }

        [[nodiscard]] double distance(Vertex3D point) const { return unit_normal_.dot(point - centroid_); }

       private:

        Vector3D unit_normal_;
        Vertex3D centroid_;
    };

    class BestFitPlaneEquation : public PlaneEquationBase
    {
       public:
        BestFitPlaneEquation(const Vector3D& unit_normal, const Vertex3D& centroid) = delete;
        BestFitPlaneEquation(Vector3D&& unit_normal, Vertex3D&& centroid) = delete;

        //        template <typename T>
        BestFitPlaneEquation(uint32_t num_verts, Vector3DVector::const_iterator itr_begin,
                             Vector3DVector::const_iterator itr_end);

        [[nodiscard]] const Vector3D& unit_normal() const override { return unit_normal_; }

        [[nodiscard]] const Vertex3D& centroid() const override { return centroid_; }

        [[nodiscard]] double rms_error() const { return rms_error_; }

       private:
       
        Vector3D unit_normal_;
        Vertex3D centroid_;

        double rms_error_;
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
    using PlaneEquationBase = Cork::Math::PlaneEquationBase;
    using PlaneEquation = Cork::Math::PlaneEquation;
    using BestFitPlaneEquation = Cork::Math::BestFitPlaneEquation;
};  // namespace Cork
