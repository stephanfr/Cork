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
    using namespace AbsExt4;
    using namespace FixExt4;
    using namespace GMPExt4;

    inline void toAbsExt(AbsExt4_1& out, const Cork::Math::Vector3D& in)
    {
        Cork::Math::Vector3D vec = in.abs();
        out.e0 = vec.x();
        out.e1 = vec.y();
        out.e2 = vec.z();
        out.e3 = 1.0f;
    }

    //    inline void toVec3d(Cork::Math::Vector3D& out, const Ext4::Ext4_1& in)
    //    {
    // Warning: beware of division by zero!

    //        out = Cork::Math::Vector3D((NUMERIC_PRECISION)(in.e0 / in.e3), (NUMERIC_PRECISION)(in.e1 / in.e3),
    //                                   (NUMERIC_PRECISION)(in.e2 / in.e3));
    //    }

    constexpr int IN_BITS = QUANTIZATION_BITS + 1;  // +1 for sign bit

    inline void toFixExt(FixExt4_1<IN_BITS>& out, const Ext4::Ext4_1& in, const Quantization::Quantizer& quantizer)
    {
        out.e0 = FixInt::BitInt<IN_BITS>::Rep(quantizer.quantize2int(in.e0()));
        out.e1 = FixInt::BitInt<IN_BITS>::Rep(quantizer.quantize2int(in.e1()));
        out.e2 = FixInt::BitInt<IN_BITS>::Rep(quantizer.quantize2int(in.e2()));
        out.e3 = FixInt::BitInt<IN_BITS>::Rep(1);
    }

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

        ;

        Ext4::Ext4_3 t_ext3;
        Ext4::Ext4_2 temp_e2(tri.p0().join(tri.p1()));
        t_ext3 = temp_e2.join(tri.p2());

        Ext4::Ext4_2 e_ext2(edge.p0().join(edge.p1()));

        // compute the point of intersection

        Ext4::Ext4_1 p_isct( e_ext2.meet( t_ext3 ) );

        // need to adjust for negative w-coordinate

        if (p_isct.e3() < 0.0)
        {
            neg(p_isct, p_isct);
        }

        // a_t0 is the variation of t_ext3 with tp0 replaced with p_isct
        // a_t1 is the variation of t_ext3 with tp1 replaced with p_isct
        // and so on...

        Ext4::Ext4_3 a_t0, a_t1, a_t2;

        temp_e2 = p_isct.join(tri.p1());
        a_t0 = temp_e2.join(tri.p2());
        temp_e2 = tri.p0().join(p_isct);
        a_t1 = temp_e2.join(tri.p2());
        temp_e2 = tri.p0().join(tri.p1());
        a_t2 = temp_e2.join(p_isct);

        Ext4::Ext4_2 a_e0(p_isct.join(edge.p1()));
        Ext4::Ext4_2 a_e1(edge.p0().join(p_isct));

        return ((inner(t_ext3, a_t0) < 0.0) || (inner(t_ext3, a_t1) < 0.0) || (inner(t_ext3, a_t2) < 0.0) ||
                (inner(e_ext2, a_e0) < 0.0) || (inner(e_ext2, a_e1) < 0.0));
    }

    inline Cork::Math::Vector3D TriEdgeIn::coords() const
    {
        Ext4::Ext4_2 temp_e2(tri.p0().join(tri.p1()));
        Ext4::Ext4_3 t_ext3(temp_e2.join(tri.p2()));

        // construct the edge

        Ext4::Ext4_2 e_ext2(edge.p0().join(edge.p1()));

        // compute the point of intersection

        Ext4::Ext4_1 p_isct( e_ext2.meet( t_ext3 ) );

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
        AbsExt4_2 ktemp2;
        std::array<AbsExt4_1, 2> kep;
        std::array<AbsExt4_1, 3> ktp;
        AbsExt4_2 ke_ext2;
        Ext4::Ext4_3 t_ext3;
        AbsExt4_3 kt_ext3;

        // load the points

        abs(kep[0], edge.p0());
        abs(kep[1], edge.p1());

        abs(ktp[0], tri.p0());
        abs(ktp[1], tri.p1());
        abs(ktp[2], tri.p2());

        // form the edge and triangle

        Ext4::Ext4_2 e_ext2(edge.p0().join(edge.p1()));
        join(ke_ext2, kep[0], kep[1]);
        Ext4::Ext4_2 temp2(tri.p0().join(tri.p1()));
        join(ktemp2, ktp[0], ktp[1]);
        t_ext3 = temp2.join(tri.p2());
        join(kt_ext3, ktemp2, ktp[2]);

        // compute the point of intersection

        Ext4::Ext4_1 pisct( e_ext2.meet( t_ext3 ) );
        AbsExt4_1 kpisct;
        meet(kpisct, ke_ext2, kt_ext3);

        // We perform one of the filter exit tests here...

        if (!filterCheck(pisct.e3(), kpisct.e3, COEFF_IT12_PISCT))
        {
            return 0;  // i.e. uncertain
        }

        // need to adjust for negative w-coordinate

        if (pisct.e3() < 0.0)
        {
            neg(pisct, pisct);
        }

        bool uncertain = false;

        // process edge

        for (int i = 0; i < 2; i++)
        {
            AbsExt4_2 ka;

            Ext4::Ext4_2 a(((i == 0) ? pisct : edge.p0()).join((i == 1) ? pisct : edge.p1()));
            join(ka, (i == 0) ? kpisct : kep[0], (i == 1) ? kpisct : kep[1]);

            double dot = inner(e_ext2, a);
            double kdot = inner(ke_ext2, ka);

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
            Ext4::Ext4_3 a;
            AbsExt4_3 ka;

            temp2 = ((i == 0) ? pisct : tri.p0()).join((i == 1) ? pisct : tri.p1());
            a = temp2.join((i == 2) ? pisct : tri.p2());
            join(ktemp2, (i == 0) ? kpisct : ktp[0], (i == 1) ? kpisct : ktp[1]);
            join(ka, ktemp2, (i == 2) ? kpisct : ktp[2]);

            double dot = inner(t_ext3, a);
            double kdot = inner(kt_ext3, ka);

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

        if (uncertain)
        {
            return 0;
        }
        else
        {
            return -1;  // i.e. false (the intersection is not empty)
        }
    }

    bool TriEdgeIn::exactFallback(const Quantization::Quantizer& quantizer, ExactArithmeticContext& context) const
    {
        // How many bits do we need for various intermediary values?
        // Here we label the amount with the relevant type (i.e. EXT2)
        // and the relevant role

        constexpr int LINE_BITS = 2 * IN_BITS + 1;
        constexpr int TRI_BITS = LINE_BITS + IN_BITS + 2;
        constexpr int ISCT_BITS = TRI_BITS + LINE_BITS + 2;
        constexpr int LINE_A_BITS = ISCT_BITS + IN_BITS + 1;
        constexpr int TRI_A_BITS = LINE_A_BITS + IN_BITS + 2;
        constexpr int INNER_LINE_BITS = LINE_A_BITS + LINE_BITS + 3;
        constexpr int INNER_TRI_BITS = TRI_A_BITS + TRI_BITS + 2;

        // pull in points

        std::array<FixExt4_1<IN_BITS>, 2> ep;
        std::array<FixExt4_1<IN_BITS>, 3> tp;

        toFixExt(ep[0], edge.p0(), quantizer);
        toFixExt(ep[1], edge.p1(), quantizer);

        toFixExt(tp[0], tri.p0(), quantizer);
        toFixExt(tp[1], tri.p1(), quantizer);
        toFixExt(tp[2], tri.p2(), quantizer);

        // construct geometry

        FixExt4_2<LINE_BITS> e;
        join(e, ep[0], ep[1]);
        FixExt4_2<LINE_BITS> temp_up;
        FixExt4_3<TRI_BITS> t;
        join(temp_up, tp[0], tp[1]);
        join(t, temp_up, tp[2]);

        // compute the point of intersection

        FixExt4_1<ISCT_BITS> pisct;
        meet(pisct, e, t);

        // need to adjust for negative w-coordinate

        int e3sign = sign(pisct.e3);

        if (e3sign < 0)
        {
            neg(pisct, pisct);
        }
        else if (e3sign == 0)
        {
            context.degeneracy_count++;
            return (true);
        }

        // process edge

        FixExt4_2<LINE_A_BITS> ae0, ae1;
        FixInt::BitInt<INNER_LINE_BITS>::Rep test_e0, test_e1;

        join(ae0, pisct, ep[1]);
        join(ae1, ep[0], pisct);
        inner(test_e0, e, ae0);
        inner(test_e1, e, ae1);
        int sign_e0 = sign(test_e0);
        int sign_e1 = sign(test_e1);

        // process triangle

        FixExt4_3<TRI_A_BITS> at0, at1, at2;
        FixExt4_2<LINE_A_BITS> temp0, temp1;
        FixExt4_2<LINE_BITS> temp2;
        FixInt::BitInt<INNER_TRI_BITS>::Rep test_t0, test_t1, test_t2;
        int sign_t0, sign_t1, sign_t2;

        join(temp0, pisct, tp[1]);
        join(at0, temp0, tp[2]);
        join(temp1, tp[0], pisct);
        join(at1, temp1, tp[2]);
        join(temp2, tp[0], tp[1]);
        join(at2, temp2, pisct);
        inner(test_t0, t, at0);
        inner(test_t1, t, at1);
        inner(test_t2, t, at2);
        sign_t0 = sign(test_t0);
        sign_t1 = sign(test_t1);
        sign_t2 = sign(test_t2);

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
        else
        {
            return (filter > 0);
        }
    }

    Cork::Math::Vector3D TriEdgeIn::coordsExact(const Quantization::Quantizer& quantizer) const
    {
        // How many bits do we need for various intermediary values?
        // Here we label the amount with the relevant type (i.e. EXT2)
        // and the relevant role
        // const static int LINE_BITS       = 2*IN_BITS + 1;
        // const static int TRI_BITS        = LINE_BITS + IN_BITS + 2;
        // const static int ISCT_BITS       = TRI_BITS + LINE_BITS + 2;
        // const static int LINE_A_BITS     = ISCT_BITS + IN_BITS + 1;
        // const static int TRI_A_BITS      = LINE_A_BITS + IN_BITS + 2;
        // const static int INNER_LINE_BITS = LINE_A_BITS + LINE_BITS + 3;
        // const static int INNER_TRI_BITS  = TRI_A_BITS + TRI_BITS + 2;

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
        std::array<Ext4::Ext4_3, 3> t_ext3s;

        for (uint ti = 0; ti < 3; ti++)
        {
            Ext4::Ext4_2 temp_e2(m_tri[ti].p0().join(m_tri[ti].p1()));
            t_ext3s[ti] = temp_e2.join(m_tri[ti].p2());
        }

        // compute the point of intersection

        Ext4::Ext4_2 temp_e2( meet( t_ext3s[0], t_ext3s[1]));
        Ext4::Ext4_1 p_isct(temp_e2.meet( t_ext3s[2]));

        // need to adjust for negative w-coordinate

        if (p_isct.e3() < 0.0)
        {
            neg(p_isct, p_isct);
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
                Ext4::Ext4_3 a;
                Ext4::Ext4_2 temp_e2(((pi == 0) ? p_isct : m_tri[ti].p0()).join(((pi == 1) ? p_isct : m_tri[ti].p1())));
                a = temp_e2.join(((pi == 2) ? p_isct : m_tri[ti].p2()));
                double test = inner(t_ext3s[ti], a);
                if (test < 0.0)  // AHA, p_isct IS outside this triangle
                    return true;
            }
        }

        // well, p_isct must be inside all of the triangles.

        return (false);
    }

    Cork::Math::Vector3D TriTriTriIn::coords() const
    {
        // construct the triangles
        std::array<Ext4::Ext4_3, 3> t_ext3s;

        for (uint ti = 0; ti < 3; ti++)
        {
            Ext4::Ext4_2 temp_e2(m_tri[ti].p0().join(m_tri[ti].p1()));
            t_ext3s[ti] = temp_e2.join(m_tri[ti].p2());
        }

        // compute the point of intersection

        Ext4::Ext4_2 temp_e2( meet( t_ext3s[0], t_ext3s[1]));
        Ext4::Ext4_1 p_isct( temp_e2.meet(t_ext3s[2]));

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
        AbsExt4_2 ktemp2;
        std::array<std::array<AbsExt4_1, 3>, 3> kp;
        std::array<Ext4::Ext4_3, 3> t;
        std::array<AbsExt4_3, 3> kt;

        // load the points and form triangles

        for (uint i = 0; i < 3; i++)
        {
            abs(kp[i][0], m_tri[i].p0());
            abs(kp[i][1], m_tri[i].p1());
            abs(kp[i][2], m_tri[i].p2());

            Ext4::Ext4_2 temp2(m_tri[i].p0().join(m_tri[i].p1()));
            join(ktemp2, kp[i][0], kp[i][1]);
            t[i] = temp2.join(m_tri[i].p2());
            join(kt[i], ktemp2, kp[i][2]);
        }

        // compute the point of intersection
        AbsExt4_1 kpisct;

        Ext4::Ext4_2 temp2( meet( t[0], t[1]));
        meet(ktemp2, kt[0], kt[1]);
        Ext4::Ext4_1 pisct( temp2.meet( t[2]));
        meet(kpisct, ktemp2, kt[2]);

        // We perform one of the filter exit tests here...

        if (!filterCheck(pisct.e3(), kpisct.e3, COEFF_IT222_PISCT))
        {
            return 0;  // i.e. uncertain
        }

        // need to adjust for negative w-coordinate

        if (pisct.e3() < 0.0)
        {
            neg(pisct, pisct);
        }

        bool uncertain = false;

        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                AbsExt4_2 kb;
                Ext4::Ext4_3 a;
                AbsExt4_3 ka;

                Ext4::Ext4_2 b(((j == 0) ? pisct : m_tri[i].p0()).join((j == 1) ? pisct : m_tri[i].p1()));
                a = b.join((j == 2) ? pisct : m_tri[i].p2());
                join(kb, (j == 0) ? kpisct : kp[i][0], (j == 1) ? kpisct : kp[i][1]);
                join(ka, kb, (j == 2) ? kpisct : kp[i][2]);

                double dot = inner(t[i], a);
                double kdot = inner(kt[i], ka);

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

        if (uncertain)
        {
            return 0;
        }
        else
        {
            return -1;  // i.e. false (the intersection is not empty)
        }
    }

    bool TriTriTriIn::exactFallback(const Quantization::Quantizer& quantizer, ExactArithmeticContext& context) const
    {
        // How many bits do we need for various intermediary values?
        // Here we label the amount with the relevant type (i.e. EXT2)
        // and the relevant role

        constexpr int EXT2_UP_BITS = 2 * IN_BITS + 1;
        constexpr int EXT3_UP_BITS = EXT2_UP_BITS + IN_BITS + 2;
        constexpr int EXT2_DN_BITS = 2 * EXT3_UP_BITS + 1;
        constexpr int ISCT_BITS = EXT2_DN_BITS + EXT3_UP_BITS + 2;
        constexpr int EXT2_TA_BITS = ISCT_BITS + IN_BITS + 1;
        constexpr int EXT3_TA_BITS = EXT2_TA_BITS + IN_BITS + 2;
        constexpr int INNER_BITS = EXT3_TA_BITS + EXT3_UP_BITS + 2;

        FixExt4_1<IN_BITS> p[3][3];
        FixExt4_3<EXT3_UP_BITS> t[3];

        for (uint i = 0; i < 3; i++)
        {
            toFixExt(p[i][0], m_tri[i].p0(), quantizer);
            toFixExt(p[i][1], m_tri[i].p1(), quantizer);
            toFixExt(p[i][2], m_tri[i].p2(), quantizer);

            FixExt4_2<EXT2_UP_BITS> temp;

            join(temp, p[i][0], p[i][1]);
            join(t[i], temp, p[i][2]);
        }

        // compute the point of intersection

        FixExt4_1<ISCT_BITS> pisct;
        {
            FixExt4_2<EXT2_DN_BITS> temp;
            meet(temp, t[0], t[1]);
            meet(pisct, temp, t[2]);
        }

        // need to adjust for negative w-coordinate

        int e3sign = sign(pisct.e3);

        if (e3sign < 0)
        {
            neg(pisct, pisct);
        }
        else if (e3sign == 0)
        {
            context.degeneracy_count++;
            return true;
        }

        bool uncertain = false;

        for (uint i = 0; i < 3; i++)
        {
            FixExt4_3<EXT3_TA_BITS> a[3];
            FixExt4_2<EXT2_TA_BITS> temp;
            FixExt4_2<EXT2_UP_BITS> tmp2;

            join(temp, pisct, p[i][1]);
            join(a[0], temp, p[i][2]);
            join(temp, p[i][0], pisct);
            join(a[1], temp, p[i][2]);
            join(tmp2, p[i][0], p[i][1]);
            join(a[2], tmp2, pisct);

            for (uint j = 0; j < 3; j++)
            {
                FixInt::BitInt<INNER_BITS>::Rep test;
                inner(test, a[j], t[i]);
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
        else
        {
            return (filter > 0);
        }
    }

    Cork::Math::Vector3D TriTriTriIn::coordsExact(const Quantization::Quantizer& quantizer) const
    {
        // How many bits do we need for various intermediary values?
        // Here we label the amount with the relevant type (i.e. EXT2)
        // and the relevant role
        // const static int EXT2_UP_BITS = 2*IN_BITS + 1;
        // const static int EXT3_UP_BITS = EXT2_UP_BITS + IN_BITS + 2;
        // const static int EXT2_DN_BITS = 2*EXT3_UP_BITS + 1;
        // const static int ISCT_BITS    = EXT2_DN_BITS + EXT3_UP_BITS + 2;
        // const static int EXT2_TA_BITS = ISCT_BITS + IN_BITS + 1;
        // const static int EXT3_TA_BITS = EXT2_TA_BITS + IN_BITS + 2;
        // const static int INNER_BITS   = EXT3_TA_BITS + EXT3_UP_BITS + 2;

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
        // How many bits do we need for various intermediary values?
        // Here we label the amount with the relevant type (i.e. EXT2)
        // and the relevant role
        // const static int EXT2_UP_BITS = 2*IN_BITS + 1;
        // const static int EXT3_UP_BITS = EXT2_UP_BITS + IN_BITS + 2;
        // const static int EXT2_DN_BITS = 2*EXT3_UP_BITS + 1;
        // const static int ISCT_BITS    = EXT2_DN_BITS + EXT3_UP_BITS + 2;
        // const static int EXT2_TA_BITS = ISCT_BITS + IN_BITS + 1;
        // const static int EXT3_TA_BITS = EXT2_TA_BITS + IN_BITS + 2;
        // const static int INNER_BITS   = EXT3_TA_BITS + EXT3_UP_BITS + 2;

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
