// +-------------------------------------------------------------------------
// | normal_projector.hpp
// |
// | Author: Stephan Friedl
// +-------------------------------------------------------------------------
// | COPYRIGHT:
// |    Copyright Stephan Friedl 2015
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

#include "primitives/primitives.hpp"


namespace Cork::Math
{

    class NormalProjector
    {
       public:
        NormalProjector(const Vector3D& v0, const Vector3D& v1, const Vector3D& v2)
        {
            Vector3D normal = (v1 - v0).cross(v2 - v0);
            uint normdim = normal.abs().maxDim();
            proj_dim0_ = (normdim + 1) % 3;
            proj_dim1_ = (normdim + 2) % 3;
            sign_flip_ = (normal[normdim] < 0.0) ? -1.0 : 1.0;
            flip_sign_ = (sign_flip_ != 1.0);
        }

        explicit NormalProjector(const TriangleByVertices& tri)
            : NormalProjector( tri.vertexA(), tri.vertexB(), tri.vertexC() )
        {}


        [[nodiscard]] uint32_t proj_dim0() const { return proj_dim0_; }
        [[nodiscard]] uint32_t proj_dim1() const { return proj_dim1_; }

        [[nodiscard]] double sign_flip() const { return sign_flip_; }
        [[nodiscard]] bool flip_sign() const { return flip_sign_; }

        Vertex2D project(const Vector3D& point) const
        {
            double x = point[proj_dim0_];
            double y = flip_sign_ ? point[proj_dim1_] * sign_flip_ : point[proj_dim1_];

            return Vertex2D(x, y);
        }

       private:
        uint32_t proj_dim0_;
        uint32_t proj_dim1_;

        double sign_flip_;
        bool flip_sign_;
    };
}   //  Cork::Math

