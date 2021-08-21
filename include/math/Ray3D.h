// +-------------------------------------------------------------------------
// | Ray3D.h
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

#include "Primitives.h"

namespace Cork::Math
{
    class Ray3D
    {
       public:
        Ray3D() {}

        Ray3D(const Vector3D& point, const Vector3D& dir) : m_origin(point), m_direction(dir) {}

        Ray3D(const Ray3D& cp) : m_origin(cp.m_origin), m_direction(cp.m_direction) {}

        const Vector3D& origin() const { return (m_origin); }

        const Vector3D& direction() const { return (m_direction); }

       private:
        Vector3D m_origin;
        Vector3D m_direction;
    };

    class Ray3DWithInverseDirection : public Ray3D
    {
       public:
        Ray3DWithInverseDirection() {}

        Ray3DWithInverseDirection(const Vector3D& point, const Vector3D& dir)
            : Ray3D(point, dir),
              m_inverseDirection(1.0 / dir.x(), 1.0 / dir.y(), 1.0 / dir.z()),
              m_signs({(m_inverseDirection.x() < 0), (m_inverseDirection.y() < 0), (m_inverseDirection.z() < 0)})
        {
        }

        Ray3DWithInverseDirection(const Ray3D& cp)
            : Ray3D(cp),
              m_inverseDirection(1.0 / cp.direction().x(), 1.0 / cp.direction().y(), 1.0 / cp.direction().z()),
              m_signs({(m_inverseDirection.x() < 0), (m_inverseDirection.y() < 0), (m_inverseDirection.z() < 0)})
        {
        }

        const Vector3D& inverseDirection() const { return (m_inverseDirection); }

        const std::array<int, 3>& signs() const { return (m_signs); }

       private:
        Vector3D m_inverseDirection;

        std::array<int, 3> m_signs;
    };

    inline std::ostream& operator<<(std::ostream& out, const Ray3D& ray)
    {
        return out << '[' << ray.origin() << ';' << ray.direction() << ']';
    }

}  // namespace Cork::Math
