// +-------------------------------------------------------------------------
// | empty3d.cpp
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

#include "intersection/empty3d.h"

#include <cfloat>

#include "intersection/quantization.h"

namespace Cork::Empty3d
{
    using namespace GMPExt4;

    inline void toVec3d(Cork::Math::Vector3D& out, const GmpExt4_1& in, const Quantization::Quantizer& quantizer)
    {
        std::array<double, 4> tmp{in.e0.get_d(), in.e1.get_d(), in.e2.get_d(), in.e3.get_d()};

        tmp[0] /= tmp[3];
        tmp[1] /= tmp[3];
        tmp[2] /= tmp[3];
        tmp[3] /= tmp[3];

        for (uint k = 0; k < 3; k++)
        {
            out[k] = (NUMERIC_PRECISION)(quantizer.reshrink(tmp[k]));
        }
    }

    constexpr double EPS = DBL_EPSILON;
    constexpr double EPS2 = EPS * EPS;

    inline bool filterCheck(double val, double absval, double coeff) { return (fabs(val) > (absval * coeff)); }

    inline bool TriEdgeIn::isEmpty(ExactArithmeticContext& context) const
    {
        context.callcount++;

        ExteriorCalculusR4::Ext4_2 temp_e2(tri.p0().join(tri.p1()));
        ExteriorCalculusR4::Ext4_3 t_ext3 = temp_e2.join(tri.p2());

        ExteriorCalculusR4::Ext4_2 e_ext2(edge.p0().join(edge.p1()));

        // compute the point of intersection

        ExteriorCalculusR4::Ext4_1 p_isct(e_ext2.meet(t_ext3));

        // need to adjust for negative w-coordinate

        if (p_isct.e3() < 0.0)
        {
            p_isct.negate();
        }

        // a_t0 is the variation of t_ext3 with tp0 replaced with p_isct
        // a_t1 is the variation of t_ext3 with tp1 replaced with p_isct
        // and so on...

        temp_e2 = p_isct.join(tri.p1());
        ExteriorCalculusR4::Ext4_3 a_t0(temp_e2.join(tri.p2()));
        temp_e2 = tri.p0().join(p_isct);
        ExteriorCalculusR4::Ext4_3 a_t1(temp_e2.join(tri.p2()));
        temp_e2 = tri.p0().join(tri.p1());
        ExteriorCalculusR4::Ext4_3 a_t2(temp_e2.join(p_isct));

        ExteriorCalculusR4::Ext4_2 a_e0(p_isct.join(edge.p1()));
        ExteriorCalculusR4::Ext4_2 a_e1(edge.p0().join(p_isct));

        return ((t_ext3.inner(a_t0) < 0.0) || (t_ext3.inner(a_t1) < 0.0) || (t_ext3.inner(a_t2) < 0.0) ||
                (e_ext2.inner(a_e0) < 0.0) || (e_ext2.inner(a_e1) < 0.0));
    }

    inline Cork::Math::Vector3D TriEdgeIn::coords() const
    {
        ExteriorCalculusR4::Ext4_2 temp_e2(tri.p0().join(tri.p1()));
        ExteriorCalculusR4::Ext4_3 t_ext3(temp_e2.join(tri.p2()));

        // construct the edge

        ExteriorCalculusR4::Ext4_2 e_ext2(edge.p0().join(edge.p1()));

        // compute the point of intersection

        ExteriorCalculusR4::Ext4_1 p_isct(e_ext2.meet(t_ext3));

        // no need to adjust for negative w-coordinate.
        // will drop out in divide.

        Cork::Math::Vector3D result(p_isct);

        return result;
    }

    constexpr double COEFF_IT12_PISCT = 10.0 * EPS + 64.0 * EPS2;
    constexpr double COEFF_IT12_S1 = 20.0 * EPS + 256.0 * EPS2;
    constexpr double COEFF_IT12_S2 = 24.0 * EPS + 512.0 * EPS2;

    int TriEdgeIn::emptyFilter() const
    {
        std::array<ExteriorCalculusR4::AbsExt4_1, 2> kep;
        std::array<ExteriorCalculusR4::AbsExt4_1, 3> ktp;

        // load the points

        kep[0] = edge.p0();
        kep[1] = edge.p1();

        ktp[0] = tri.p0();
        ktp[1] = tri.p1();
        ktp[2] = tri.p2();

        // form the edge and triangle

        ExteriorCalculusR4::Ext4_2 e_ext2(edge.p0().join(edge.p1()));
        ExteriorCalculusR4::AbsExt4_2 ke_ext2(kep[0].join(kep[1]));
        ExteriorCalculusR4::Ext4_2 temp2(tri.p0().join(tri.p1()));
        ExteriorCalculusR4::AbsExt4_2 ktemp2(ktp[0].join(ktp[1]));
        ExteriorCalculusR4::Ext4_3 t_ext3(temp2.join(tri.p2()));
        ExteriorCalculusR4::AbsExt4_3 kt_ext3(ktemp2.join(ktp[2]));

        // compute the point of intersection

        ExteriorCalculusR4::Ext4_1 pisct(e_ext2.meet(t_ext3));
        ExteriorCalculusR4::AbsExt4_1 kpisct(ke_ext2.meet(kt_ext3));

        // We perform one of the filter exit tests here...

        if (!filterCheck(pisct.e3(), kpisct.e3(), COEFF_IT12_PISCT))
        {
            return 0;  // i.e. uncertain
        }

        // need to adjust for negative w-coordinate

        if (pisct.e3() < 0.0)
        {
            pisct.negate();
        }

        bool uncertain = false;

        // process edge

        for (int i = 0; i < 2; i++)
        {
            ExteriorCalculusR4::Ext4_2 a(((i == 0) ? pisct : edge.p0()).join((i == 1) ? pisct : edge.p1()));
            ExteriorCalculusR4::AbsExt4_2 ka(((i == 0) ? kpisct : kep[0]).join((i == 1) ? kpisct : kep[1]));

            double dot = e_ext2.inner(a);
            double kdot = ke_ext2.inner(ka);

            // now figure out what to do...

            bool outside = dot < 0.0;
            bool reliable = filterCheck(dot, kdot, COEFF_IT12_S1);

            if (reliable && outside)
            {
                return 1;  // i.e. true
            }

            if (!reliable)
            {
                uncertain = true;
            }
        }

        // process triangle

        for (int i = 0; i < 3; i++)
        {
            temp2 = ((i == 0) ? pisct : tri.p0()).join((i == 1) ? pisct : tri.p1());
            ExteriorCalculusR4::Ext4_3 a(temp2.join((i == 2) ? pisct : tri.p2()));
            ktemp2 = ((i == 0) ? kpisct : ktp[0]).join((i == 1) ? kpisct : ktp[1]);
            ExteriorCalculusR4::AbsExt4_3 ka(ktemp2.join((i == 2) ? kpisct : ktp[2]));

            double dot = t_ext3.inner(a);
            double kdot = kt_ext3.inner(ka);

            // now figure out what to do...

            bool outside = dot < 0.0;
            bool reliable = filterCheck(dot, kdot, COEFF_IT12_S2);

            if (reliable && outside)
            {
                return 1;  // i.e. true
            }

            if (!reliable)
            {
                uncertain = true;
            }
        }

        return uncertain ? 0 : -1;

        //        if (uncertain)
        //        {
        //            return 0;
        //        }
        //        else
        //        {
        //            return -1;  // i.e. false (the intersection is not empty)
        //        }
    }

    bool TriEdgeIn::exactFallback(const Quantization::Quantizer& quantizer, ExactArithmeticContext& context) const
    {
        //  We use 'auto' for type deduction extensively in this method.  The lengths of the
        //      FixExt? types change depending on the operations used.  For example, 64 bits would be
        //      required to store the product of two 32 bit fixed integer types.  The types can get
        //      large and unwieldly.  Type deduction here simply lets the compiler make sure everything
        //      matches up properly.
        //
        //  Unit tests should check that the results of the operations are correct in size and value. 

        // pull in points

        std::array<ExteriorCalculusR4::FixExt4_1<FIXED_INTEGER_BITS>, 2> ep{
            {ExteriorCalculusR4::FixExt4_1<FIXED_INTEGER_BITS>(edge.p0(), quantizer),
             ExteriorCalculusR4::FixExt4_1<FIXED_INTEGER_BITS>(edge.p1(), quantizer)}};
        std::array<ExteriorCalculusR4::FixExt4_1<FIXED_INTEGER_BITS>, 3> tp{
            {ExteriorCalculusR4::FixExt4_1<FIXED_INTEGER_BITS>(tri.p0(), quantizer),
             ExteriorCalculusR4::FixExt4_1<FIXED_INTEGER_BITS>(tri.p1(), quantizer),
             ExteriorCalculusR4::FixExt4_1<FIXED_INTEGER_BITS>(tri.p2(), quantizer)}};

        // construct geometry

        auto e = ep[0].join(ep[1]);
        auto temp_up = tp[0].join(tp[1]);
        auto t = temp_up.join(tp[2]);

        // compute the point of intersection

        auto pisct = e.meet(t);

        // need to adjust for negative w-coordinate

        int e3sign = sign(pisct.e3());

        if (e3sign < 0)
        {
            pisct.negate();
        }
        else if (e3sign == 0)
        {
            context.degeneracy_count++;
            return (true);
        }

        // process edge

        auto ae0 = pisct.join(ep[1]);
        auto ae1 = ep[0].join(pisct);

        auto test_e0 = e.inner(ae0);
        auto test_e1 = e.inner(ae1);
        int sign_e0 = sign(test_e0);
        int sign_e1 = sign(test_e1);

        // process triangle

        auto temp0 = pisct.join(tp[1]);
        auto at0 = temp0.join(tp[2]);

        auto temp1 = tp[0].join( pisct);
        auto at1 = temp1.join(tp[2]);

        auto temp2 = tp[0].join( tp[1]);
        auto at2 = temp2.join(pisct);

        auto test_t0 = t.inner( at0);
        auto test_t1 = t.inner( at1);
        auto test_t2 = t.inner( at2);

        int sign_t0 = sign(test_t0);
        int sign_t1 = sign(test_t1);
        int sign_t2 = sign(test_t2);

        if (sign_e0 < 0 || sign_e1 < 0 || sign_t0 < 0 || sign_t1 < 0 || sign_t2 < 0)
        {
            return (true);
        }

        if (sign_e0 == 0 || sign_e1 == 0 || sign_t0 == 0 || sign_t1 == 0 || sign_t2 == 0)
        {
            context.degeneracy_count++;
        }

        return (false);
    }

    bool TriEdgeIn::emptyExact(const Quantization::Quantizer& quantizer, ExactArithmeticContext& context) const
    {
        context.callcount++;

        int filter = emptyFilter();

        if (filter == 0)
        {
            context.exact_count++;
            return (exactFallback(quantizer, context));
        }
        //        else
        //        {
        return (filter > 0);
        //        }
    }

    Cork::Math::Vector3D TriEdgeIn::coordsExact(const Quantization::Quantizer& quantizer) const
    {
        // pull in points

        std::array<GmpExt4_1, 2> ep;
        std::array<GmpExt4_1, 3> tp;

        toGmpExt(ep[0], edge.p0(), quantizer);
        toGmpExt(ep[1], edge.p1(), quantizer);

        toGmpExt(tp[0], tri.p0(), quantizer);
        toGmpExt(tp[1], tri.p1(), quantizer);
        toGmpExt(tp[2], tri.p2(), quantizer);

        // construct geometry
        GmpExt4_2 e;
        join(e, ep[0], ep[1]);

        GmpExt4_2 temp_up;
        GmpExt4_3 t;
        join(temp_up, tp[0], tp[1]);
        join(t, temp_up, tp[2]);

        // compute the point of intersection
        GmpExt4_1 pisct;
        meet(pisct, e, t);

        // convert to double

        Cork::Math::Vector3D result;
        toVec3d(result, pisct, quantizer);

        // std::cout << result << std::endl;

        return (result);
    }

    bool TriTriTriIn::isEmpty(ExactArithmeticContext& context) const
    {
        context.callcount++;

        // construct the triangles
        std::array<ExteriorCalculusR4::Ext4_3, 3> t_ext3s;

        for (uint ti = 0; ti < 3; ti++)
        {
            ExteriorCalculusR4::Ext4_2 temp_e2(m_tri[ti].p0().join(m_tri[ti].p1()));
            t_ext3s[ti] = temp_e2.join(m_tri[ti].p2());
        }

        // compute the point of intersection

        ExteriorCalculusR4::Ext4_2 temp_e2(t_ext3s[0].meet(t_ext3s[1]));
        ExteriorCalculusR4::Ext4_1 p_isct(temp_e2.meet(t_ext3s[2]));

        // need to adjust for negative w-coordinate

        if (p_isct.e3() < 0.0)
        {
            p_isct.negate();
        }

        // Test whether p_isct is inside each triangle
        // For each triangle, this is done by creating three modified
        // versions, replacing each point with p_isct.
        // If none of these modifications flips the orientation of
        // the resulting plane, then the point must lie inside the triangle

        for (uint ti = 0; ti < 3; ti++)
        {
            for (uint pi = 0; pi < 3; pi++)
            {  // three copies...
                ExteriorCalculusR4::Ext4_3 a;
                ExteriorCalculusR4::Ext4_2 temp_e2(
                    ((pi == 0) ? p_isct : m_tri[ti].p0()).join(((pi == 1) ? p_isct : m_tri[ti].p1())));

                a = temp_e2.join(((pi == 2) ? p_isct : m_tri[ti].p2()));
                double test = t_ext3s[ti].inner(a);

                if (test < 0.0)  // AHA, p_isct IS outside this triangle
                {
                    return true;
                }
            }
        }

        // well, p_isct must be inside all of the triangles.

        return (false);
    }

    Cork::Math::Vector3D TriTriTriIn::coords() const
    {
        // construct the triangles
        std::array<ExteriorCalculusR4::Ext4_3, 3> t_ext3s;

        for (uint ti = 0; ti < 3; ti++)
        {
            ExteriorCalculusR4::Ext4_2 temp_e2(m_tri[ti].p0().join(m_tri[ti].p1()));
            t_ext3s[ti] = temp_e2.join(m_tri[ti].p2());
        }

        // compute the point of intersection

        ExteriorCalculusR4::Ext4_2 temp_e2(t_ext3s[0].meet(t_ext3s[1]));
        ExteriorCalculusR4::Ext4_1 p_isct(temp_e2.meet(t_ext3s[2]));

        // no need to adjust for negative w-coordinate.
        // will come out in the divide.

        Cork::Math::Vector3D result(p_isct);
        //        toVec3d(result, p_isct);

        return (result);
    }

    constexpr double COEFF_IT222_PISCT = 20.0 * EPS + 256.0 * EPS2;
    constexpr double COEFF_IT222_S2 = 34.0 * EPS + 1024.0 * EPS2;

    int TriTriTriIn::emptyFilter() const
    {
        std::array<std::array<ExteriorCalculusR4::AbsExt4_1, 3>, 3> kp;
        std::array<ExteriorCalculusR4::Ext4_3, 3> t;
        std::array<ExteriorCalculusR4::AbsExt4_3, 3> kt;

        // load the points and form triangles

        for (uint i = 0; i < 3; i++)
        {
            kp[i][0] = m_tri[i].p0();
            kp[i][1] = m_tri[i].p1();
            kp[i][2] = m_tri[i].p2();

            ExteriorCalculusR4::Ext4_2 temp2(m_tri[i].p0().join(m_tri[i].p1()));
            ExteriorCalculusR4::AbsExt4_2 ktemp2(kp[i][0].join(kp[i][1]));
            t[i] = temp2.join(m_tri[i].p2());
            kt[i] = ktemp2.join(kp[i][2]);
        }

        // compute the point of intersection

        ExteriorCalculusR4::Ext4_2 temp2(t[0].meet(t[1]));
        ExteriorCalculusR4::AbsExt4_2 ktemp2(kt[0].meet(kt[1]));
        ExteriorCalculusR4::Ext4_1 pisct(temp2.meet(t[2]));
        ExteriorCalculusR4::AbsExt4_1 kpisct(ktemp2.meet(kt[2]));

        // We perform one of the filter exit tests here...

        if (!filterCheck(pisct.e3(), kpisct.e3(), COEFF_IT222_PISCT))
        {
            return 0;  // i.e. uncertain
        }

        // need to adjust for negative w-coordinate

        if (pisct.e3() < 0.0)
        {
            pisct.negate();
        }

        bool uncertain = false;

        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                ExteriorCalculusR4::Ext4_2 b(((j == 0) ? pisct : m_tri[i].p0()).join((j == 1) ? pisct : m_tri[i].p1()));
                ExteriorCalculusR4::Ext4_3 a(b.join((j == 2) ? pisct : m_tri[i].p2()));
                ExteriorCalculusR4::AbsExt4_2 kb(((j == 0) ? kpisct : kp[i][0]).join((j == 1) ? kpisct : kp[i][1]));
                ExteriorCalculusR4::AbsExt4_3 ka(kb.join((j == 2) ? kpisct : kp[i][2]));

                double dot = t[i].inner(a);
                double kdot = kt[i].inner(ka);

                // then consider the filtering...

                bool outside = dot < 0.0;
                bool reliable = filterCheck(dot, kdot, COEFF_IT222_S2);

                if (reliable && outside)
                {
                    return 1;  // i.e. true
                }

                if (!reliable)
                {
                    uncertain = true;
                }
            }
        }

        return uncertain ? 0 : -1;

        //        if (uncertain)
        //        {
        //            return 0;
        //        }
        //        else
        //        {
        //            return -1;  // i.e. false (the intersection is not empty)
        //        }
    }

    bool TriTriTriIn::exactFallback(const Quantization::Quantizer& quantizer, ExactArithmeticContext& context) const
    {
        // How many bits do we need for various intermediary values?
        // Here we label the amount with the relevant type (i.e. EXT2)
        // and the relevant role

        constexpr int EXT2_UP_BITS = 2 * FIXED_INTEGER_BITS + 1;
        constexpr int EXT3_UP_BITS = EXT2_UP_BITS + FIXED_INTEGER_BITS + 2;
        constexpr int EXT2_DN_BITS = 2 * EXT3_UP_BITS + 1;
        constexpr int ISCT_BITS = EXT2_DN_BITS + EXT3_UP_BITS + 2;
        constexpr int EXT2_TA_BITS = ISCT_BITS + FIXED_INTEGER_BITS + 1;
        constexpr int EXT3_TA_BITS = EXT2_TA_BITS + FIXED_INTEGER_BITS + 2;
        constexpr int INNER_BITS = EXT3_TA_BITS + EXT3_UP_BITS + 2;

        std::array<std::array<ExteriorCalculusR4::FixExt4_1<FIXED_INTEGER_BITS>, 3>, 3> p;
        std::array<ExteriorCalculusR4::FixExt4_3<EXT3_UP_BITS>, 3> t;

        for (uint i = 0; i < 3; i++)
        {
//            toFixExt(p[i][0], m_tri[i].p0(), quantizer);
//            toFixExt(p[i][1], m_tri[i].p1(), quantizer);
//            toFixExt(p[i][2], m_tri[i].p2(), quantizer);

            p[i][0] = ExteriorCalculusR4::FixExt4_1<FIXED_INTEGER_BITS>( m_tri[i].p0(), quantizer);
            p[i][1] = ExteriorCalculusR4::FixExt4_1<FIXED_INTEGER_BITS>( m_tri[i].p1(), quantizer);
            p[i][2] = ExteriorCalculusR4::FixExt4_1<FIXED_INTEGER_BITS>( m_tri[i].p2(), quantizer);


//            ExteriorCalculusR4::FixExt4_2<EXT2_UP_BITS> temp;

//            join(temp, p[i][0], p[i][1]);
            auto temp = p[i][0].join( p[i][1]);
//            join(t[i], temp, p[i][2]);
            t[i] = temp.join( p[i][2]);
        }

        // compute the point of intersection

        ExteriorCalculusR4::FixExt4_1<ISCT_BITS> pisct;
        {
//            ExteriorCalculusR4::FixExt4_2<EXT2_DN_BITS> temp;
            auto temp = t[0].meet(t[1]);
//            meet(pisct, temp, t[2]);
            pisct = temp.meet(t[2]);
        }

        // need to adjust for negative w-coordinate

        int e3sign = sign(pisct.e3());

        if (e3sign < 0)
        {
            pisct.negate();
        }
        else if (e3sign == 0)
        {
            context.degeneracy_count++;
            return true;
        }

        bool uncertain = false;

        for (uint i = 0; i < 3; i++)
        {
            std::array<ExteriorCalculusR4::FixExt4_3<EXT3_TA_BITS>, 3> a;
//            ExteriorCalculusR4::FixExt4_2<EXT2_TA_BITS> temp;
//            ExteriorCalculusR4::FixExt4_2<EXT2_UP_BITS> tmp2;

//            join(temp, pisct, p[i][1]);
            auto temp = pisct.join(p[i][1]);

//            join(a[0], temp, p[i][2]);
            a[0] = temp.join(p[i][2]);

//            join(temp, p[i][0], pisct);
            temp = p[i][0].join(pisct);

//            join(a[1], temp, p[i][2]);
            a[1] = temp.join(p[i][2]);

//            join(tmp2, p[i][0], p[i][1]);
            auto tmp2 = p[i][0].join( p[i][1]);
//            join(a[2], tmp2, pisct);
            a[2] = tmp2.join(pisct);


            for (uint j = 0; j < 3; j++)
            {
//                FixInt::BitInt<INNER_BITS>::Rep test;
                auto test = a[j].inner(t[i]);
                int testsign = sign(test);

                if (testsign < 0)
                {
                    return true;
                }

                if (testsign == 0)
                {
                    uncertain = true;
                }
            }
        }

        if (uncertain)
        {
            context.degeneracy_count++;
        }

        return (false);
    }

    bool TriTriTriIn::emptyExact(const Quantization::Quantizer& quantizer, ExactArithmeticContext& context) const
    {
        context.callcount++;
        int filter = emptyFilter();

        if (filter == 0)
        {
            context.exact_count++;
            return (exactFallback(quantizer, context));
        }
        //        else
        //        {
        return (filter > 0);
        //        }
    }

    Cork::Math::Vector3D TriTriTriIn::coordsExact(const Quantization::Quantizer& quantizer) const
    {
        std::array<std::array<GmpExt4_1, 3>, 3> p;
        std::array<GmpExt4_3, 3> t;

        for (uint i = 0; i < 3; i++)
        {
            toGmpExt(p[i][0], m_tri[i].p0(), quantizer);
            toGmpExt(p[i][1], m_tri[i].p1(), quantizer);
            toGmpExt(p[i][2], m_tri[i].p2(), quantizer);

            GmpExt4_2 temp;

            join(temp, p[i][0], p[i][1]);
            join(t[i], temp, p[i][2]);
        }

        // compute the point of intersection
        GmpExt4_1 pisct;

        {
            GmpExt4_2 temp;

            meet(temp, t[0], t[1]);
            meet(pisct, temp, t[2]);
        }

        // convert to double

        Cork::Math::Vector3D result;
        toVec3d(result, pisct, quantizer);

        return result;
    }

    Cork::Math::Vector3D coordsExact(const GMPExt4::GmpExt4_2& edge, const GMPExt4::GmpExt4_3& triangle,
                                     const Quantization::Quantizer& quantizer)
    {
        //	Compute the point of intersection

        GmpExt4_1 pisct;
        meet(pisct, edge, triangle);

        //	Convert to double

        Cork::Math::Vector3D result;
        toVec3d(result, pisct, quantizer);

        return result;
    }

    Cork::Math::Vector3D coordsExact(const GMPExt4::GmpExt4_3& triangle0, const GMPExt4::GmpExt4_3& triangle1,
                                     const GMPExt4::GmpExt4_3& triangle2, const Quantization::Quantizer& quantizer)
    {

        // compute the point of intersection
        GmpExt4_1 pisct;

        {
            GmpExt4_2 temp;

            meet(temp, triangle0, triangle1);
            meet(pisct, temp, triangle2);
        }

        // convert to double

        Cork::Math::Vector3D result;
        toVec3d(result, pisct, quantizer);

        return result;
    }

}  // namespace Cork::Empty3d
