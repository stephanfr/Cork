/*
 Copyright (c) 2021 Stephan Friedl

 Permission is hereby granted, free of charge, to any person obtaining a copy of
 this software and associated documentation files (the "Software"), to deal in
 the Software without restriction, including without limitation the rights to
 use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 the Software, and to permit persons to whom the Software is furnished to do so,
 subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <catch2/catch_all.hpp>
#include <catch2/catch_approx.hpp>

#include "2d_geometry/polygon.hpp"

//  The pragma below is to disable to false errors flagged by intellisense for Catch2 REQUIRE macros.

#if __INTELLISENSE__
#pragma diag_suppress 2486
#endif

//  NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers)

TEST_CASE("Polygon Tests", "[2D Basic]")
{
    const std::vector<Cork::Primitives::Vertex2D> NON_SELF_INTERSECTING_POLYGON_1 = {
        {-2.2707, -0.671873}, {-1.87269, -1.11342}, {-1.7617, -1.21776},  {-1.80882, -1.24153}, {-1.81922, -1.24714},
        {-1.74258, -1.32806}, {-1.67081, -1.41219}, {-1.6136, -1.50057},  {-1.56223, -1.60986}, {-1.52142, -1.56724},
        {-1.49817, -1.58469}, {-1.37304, -1.64217}, {-1.04275, -1.97584}, {-0.733766, 1.36335}};

    const std::vector<Cork::Primitives::Vertex2D> NON_SELF_INTERSECTING_POLYGON_2 = {
        {2.04311, -2.60638}, {2.07328, -2.29542}, {2.04436, -2.1135},  {2.07723, -2.11157}, {2.22505, -2.1908},
        {2.2044, -2.09395},  {2.29885, -2.06831}, {2.63428, -1.63493}, {2.6341, -1.55385},  {2.62903, -1.49845},
        {1.75374, -2.08527}, {1.65385, -2.17183}, {1.6755, -2.37402},  {1.78514, -2.63299}, {1.89026, -2.62838}};

    const std::vector<Cork::Primitives::Vertex2D> SELF_INTERSECTING_POLYGON_1 = {
        {2.04311, -2.60638}, {2.07328, -2.29542}, {2.04436, -2.1135},  {2.07723, -2.11157}, {2.22505, -1.6908},
        {2.2044, -2.09395},  {2.29885, -2.06831}, {2.63428, -1.63493}, {2.6341, -1.55385},  {2.62903, -1.49845},
        {1.75374, -2.08527}, {1.65385, -2.17183}, {1.6755, -2.37402},  {1.78514, -2.63299}, {1.89026, -2.62838}};

    const std::vector<Cork::Primitives::Vertex2D> SELF_INTERSECTING_POLYGON_2 = {
        {4.45327, 4.0822},  {4.25486, 3.90112}, {4.14446, 3.58195},  {4.10713, 3.5055},  {4.02483, 3.50932},
        {3.95703, 3.66378}, {4.14724, 3.86212}, {4.42137, 4.02941},  {4.61051, 4.19243}, {4.87458, 4.15157},
        {5.07155, 4.34398}, {5.30994, 4.44361}, {5.49269, 4.33694},  {5.89203, 4.30996}, {6.16475, 4.32853},
        {6.42676, 4.4449},  {6.75477, 4.68576}, {6.95534, 4.53065},  {7.27227, 4.40478}, {7.5418, 4.30457},
        {7.81271, 4.21289}, {8.17287, 4.2469},  {8.14899, 3.9465},   {8.4078, 4.07965},  {8.70196, 4.12787},
        {8.83405, 3.9786},  {9.00349, 4.13456}, {9.26543, 4.03087},  {9.39334, 3.88889}, {9.63696, 3.7445},
        {9.81179, 3.59153}, {10.1371, 3.44291}, {10.419, 3.22306},   {10.3839, 2.97313}, {10.3589, 2.74564},
        {10.3661, 2.53377}, {10.6817, 2.48854}, {11.0348, 2.53838},  {11.1159, 2.36883}, {11.1011, 2.0861},
        {11.3644, 1.93185}, {11.2023, 1.66064}, {10.9909, 1.39433},  {10.8817, 1.13261}, {10.7544, 1.0609},
        {10.7026, 1.06779}, {10.73, 1.0171},    {10.8125, 0.998518}, {10.9128, 0.96359}, {11.0119, 1.13152},
        {11.101, 1.48218},  {11.0324, 1.70447}, {11.0227, 2.00339},  {10.8903, 2.25114}, {10.786, 2.55914},
        {10.5466, 2.69772}, {10.5249, 2.97604}, {10.4788, 3.17118},  {10.3136, 3.36128}, {10.0416, 3.39964},
        {9.89797, 3.28802}, {9.89152, 3.49719}, {9.78851, 3.57638},  {9.63648, 3.58777}, {9.48121, 3.77765},
        {9.26596, 3.82067}, {9.29147, 4.02735}, {8.97868, 4.15724},  {8.74854, 3.99991}, {8.51846, 4.09383},
        {8.27966, 4.06963}, {8.14935, 3.88406}, {8.06143, 4.01364},  {7.89129, 4.06204}, {7.83425, 4.29102},
        {7.53634, 4.4172},  {7.24735, 4.52993}, {6.96612, 4.5311},   {6.62102, 4.45428}, {6.28793, 4.43222},
        {6.05952, 4.58138}, {5.80921, 4.6242},  {5.61968, 4.4473},   {5.37562, 4.30019}, {5.50612, 3.9949},
        {5.32726, 4.07613}, {5.06169, 4.22001}, {4.74451, 4.16549}};

    const std::vector<Cork::Primitives::Vertex2D> SELF_INTERSECTING_POLYGON_3 = {
        {1.95935, -1.06816},   {2.03657, -0.958725},  {2.09231, -0.834391},   {2.17588, -0.748562},
        {2.17956, -0.667772},  {2.1802, -0.585333},   {2.19978, 0.0551595},   {1.88977, 0.397893},
        {1.82898, 0.477325},   {1.77325, 0.556991},   {1.70475, 0.604511},    {1.58619, 0.845729},
        {1.47955, 0.977455},   {1.36262, 1.09416},    {1.29639, 1.21897},     {1.21279, 1.265},
        {1.19986, 1.28623},    {1.14087, 1.33021},    {1.06675, 1.37141},     {0.979473, 1.4481},
        {0.909823, 1.50672},   {0.821894, 1.57982},   {0.753623, 1.62236},    {0.677478, 1.70691},
        {0.585419, 1.78576},   {0.5809, 1.79024},     {0.428965, 1.93578},    {0.287446, 2.047},
        {0.216996, 2.10734},   {0.157291, 2.1578},    {0.092434, 2.20018},    {0.0232267, 2.22498},
        {-0.148401, 2.23719},  {-0.213918, 2.20521},  {-0.281968, 2.15697},   {-0.321865, 2.1713},
        {-0.425435, 2.12354},  {-0.512606, 2.03148},  {-0.573862, 2.00154},   {-0.644653, 1.95093},
        {-0.713868, 1.92983},  {-0.826894, 1.77832},  {-0.907591, 1.62844},   {-0.97866, 1.57536},
        {-1.00942, 1.49328},   {-1.08128, 1.45712},   {-1.07079, 1.42897},    {-1.15402, 1.3989},
        {-1.15289, 1.36314},   {-1.27268, 1.27318},   {-1.3591, 1.18618},     {-1.33966, 1.17289},
        {-1.32549, 1.17318},   {-1.30867, 1.18356},   {-1.28892, 1.18917},    {-1.27916, 1.18345},
        {-1.33165, 1.10101},   {-1.4053, 1.02438},    {-1.4142, 0.954791},    {-1.50514, 0.865025},
        {-1.55162, 0.791523},  {-1.6106, 0.710221},   {-1.67635, 0.623157},   {-1.7024, 0.528378},
        {-1.75097, 0.46044},   {-1.82165, 0.37245},   {-1.86613, 0.366217},   {-2.04861, 0.213455},
        {-2.1548, 0.127433},   {-2.3279, -0.052161},  {-2.20184, 0.00357705}, {-2.27727, -0.0809568},
        {-2.36896, -0.200651}, {-2.38692, -0.189147}, {-2.44727, -0.285606},  {-2.46921, -0.358873},
        {-2.47921, -0.439633}, {-2.49539, -0.511149}, {-2.52905, -0.519139},  {-2.51295, -0.594366},
        {-2.47883, -0.677467}, {-2.44212, -0.768172}, {-2.48412, -0.77002},   {-2.44535, -0.858211},
        {-2.49148, -0.933307}, {-2.48314, -1.01667},  {-2.39089, -1.09338},   {-2.33759, -1.25702},
        {-2.23793, -1.22359},  {-2.04524, -1.36844},  {-2.04418, -1.42932},   {-2.00761, -1.49809},
        {-2.00204, -1.5744},   {-1.91743, -1.60818},  {-1.90511, -1.74621},   {-1.78665, -1.77436},
        {-1.67757, -1.81911},  {-1.68389, -1.97044},  {-1.68919, -2.07086},   {-1.57517, -2.07216},
        {-1.50051, -2.07299},  {-1.42833, -2.07687},  {-1.34904, -2.0825},    {-1.31679, -2.05344},
        {-1.28312, -2.04236},  {-1.24901, -2.04033},  {-1.31482, -1.9895},    {-1.37622, -1.93265},
        {-1.35974, -1.95166},  {-1.2683, -2.01416},   {-1.19467, -2.04553},   {-1.14795, -2.08175},
        {-1.12385, -2.14632},  {-1.04586, -2.22675},  {-0.936395, -2.27667},  {-0.820015, -2.32899},
        {-0.730704, -2.38393}, {-0.652627, -2.44544}, {-0.525162, -2.57372},  {-0.406811, -2.69134},
        {-0.26671, -2.83517},  {-0.159789, -2.89657}, {-0.08109, -2.9202},    {-0.00602179, -2.91571},
        {0.066261, -2.89309},  {0.142085, -2.84638},  {0.18614, -2.79538},    {0.257397, -2.72466},
        {0.349353, -2.61477},  {0.419151, -2.54106},  {0.53464, -2.40912},    {0.640105, -2.29917},
        {0.726704, -2.22249},  {0.743246, -2.15886},  {0.809312, -2.16836},   {0.870535, -2.13776},
        {0.892425, -2.06249},  {0.958485, -2.04571},  {1.05352, -1.98112},    {1.13594, -1.90747},
        {1.20661, -1.8967},    {1.2782, -1.86922},    {1.33368, -1.88036},    {1.37162, -1.80719},
        {1.39659, -1.73857},   {1.45695, -1.6621},    {1.5296, -1.57352},     {1.5538, -1.56832},
        {1.61087, -1.50097},   {1.62306, -1.45942},   {1.77735, -1.24461},    {1.86121, -1.17367},
        {1.85747, -1.09603}};

    SECTION("Non Intersection 1")
    {
        std::vector<Cork::Primitives::Vertex2D> vertices = NON_SELF_INTERSECTING_POLYGON_1;

        Cork::TwoD::Polygon test_poly(std::move(vertices));

        auto self_intersections = test_poly.self_intersections();

        REQUIRE(self_intersections.empty());
    }

    SECTION("Non Intersection 2")
    {
        std::vector<Cork::Primitives::Vertex2D> vertices = NON_SELF_INTERSECTING_POLYGON_2;

        Cork::TwoD::Polygon test_poly(std::move(vertices));

        auto self_intersections = test_poly.self_intersections();

        REQUIRE(self_intersections.empty());
    }

    SECTION("Intersection 1")
    {
        std::vector<Cork::Primitives::Vertex2D> vertices = SELF_INTERSECTING_POLYGON_1;

        Cork::TwoD::Polygon test_poly(std::move(vertices));

        auto self_intersections = test_poly.self_intersections();

        REQUIRE(self_intersections.size() == 2);

        REQUIRE(self_intersections[0].edge1().reference_index() == 3);
        REQUIRE(self_intersections[0].edge2().reference_index() == 9);

        REQUIRE(self_intersections[1].edge1().reference_index() == 4);
        REQUIRE(self_intersections[1].edge2().reference_index() == 9);
    }

    SECTION("Intersection 2")
    {
        std::vector<Cork::Primitives::Vertex2D> vertices = SELF_INTERSECTING_POLYGON_2;

        Cork::TwoD::Polygon test_poly(std::move(vertices));

        auto self_intersections = test_poly.self_intersections();

        REQUIRE(self_intersections.size() == 14);
    }

    SECTION("Intersection 3")
    {
        std::vector<Cork::Primitives::Vertex2D> vertices = SELF_INTERSECTING_POLYGON_3;

        Cork::TwoD::Polygon test_poly(std::move(vertices));

        auto self_intersections = test_poly.self_intersections();

        REQUIRE(self_intersections.size() == 1);
        REQUIRE(self_intersections[0].edge1().reference_index() == 106);
        REQUIRE(self_intersections[0].edge2().reference_index() == 108);
    }

    SECTION("Centroid")
    {
        std::vector<Cork::Primitives::Vertex2D> vertices = NON_SELF_INTERSECTING_POLYGON_1;

        Cork::TwoD::Polygon test_poly(std::move(vertices));

        auto centroid = test_poly.centroid();

        REQUIRE(centroid.x() == Catch::Approx(-1.592249714).epsilon(10e-6));
        REQUIRE(centroid.y() == Catch::Approx(-1.196356643).epsilon(10e-6));
    }
}

//  NOLINTEND(cppcoreguidelines-avoid-magic-numbers)
