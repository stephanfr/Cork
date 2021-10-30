// +-------------------------------------------------------------------------
// | intersection.cpp
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

#include "intersection/perturbation_epsilon.hpp"

#include <random>
#include <vector>

#include "primitives/primitives.hpp"

namespace Cork::Intersection
{
    using Vector3D = Primitives::Vector3D;

    class PerturbationRandomizationMatrix
    {
       public:
        PerturbationRandomizationMatrix() : m_mersenneTwister(time(0))  //  NOLINT(cert-msc32-c, cert-msc51-cpp)
        {
            for (int numPermutations = 4; numPermutations <= 32;
                 numPermutations <<= 1)  //  NOLINT(cppcoreguidelines-avoid-magic-numbers)
            {
                m_randomizationMatrix.push_back(std::vector<std::tuple<long, long, long>>());

                auto& currentVec = m_randomizationMatrix.back();

                for (int i = 0; i <= numPermutations; i++)
                {
                    for (int j = 0; j <= numPermutations; j++)
                    {
                        for (int k = 0; k <= numPermutations; k++)
                        {
                            currentVec.push_back(std::tuple<long, long, long>(i, j, k));

                            if (i > 0)
                            {
                                currentVec.push_back(std::tuple<long, long, long>(-i, j, k));
                            }

                            if (j > 0)
                            {
                                currentVec.push_back(std::tuple<long, long, long>(i, -j, k));
                            }

                            if (k > 0)
                            {
                                currentVec.push_back(std::tuple<long, long, long>(i, j, -k));
                            }

                            if ((i > 0) && (j > 0))
                            {
                                currentVec.push_back(std::tuple<long, long, long>(-i, -j, k));
                            }

                            if ((i > 0) && (k > 0))
                            {
                                currentVec.push_back(std::tuple<long, long, long>(-i, j, -k));
                            }

                            if ((j > 0) && (k > 0))
                            {
                                currentVec.push_back(std::tuple<long, long, long>(i, -j, -k));
                            }

                            if ((i > 0) && (j > 0) && (k > 0))
                            {
                                currentVec.push_back(std::tuple<long, long, long>(-i, -j, -k));
                            }
                        }
                    }
                }

                m_numEntries.push_back(currentVec.size());
            }
        }

        Vector3D getPerturbation(int index, double quantum)
        {
            if (index >= m_numEntries.size())
            {
                return (getBruteForcePerturbation(index, quantum));
            }

            //	Using the standard clib random number generator std::rand() resulted in spurious errors
            //		in triangle consolidation.  This seems not to be the case with the MT generator.
            //
            //	I don't quite understand why this approach to creating an array of purturbation combinations
            //		and then choosing the right combo with a single random number is more fragile than
            //		the prior method of generating random values for each offset but with std::rand() there
            //		were certainly problems.

            long arrayIndex(m_mersenneTwister() % m_numEntries[index]);

            const std::tuple<long, long, long>& randEntry = m_randomizationMatrix[index][arrayIndex];

            return (Vector3D(std::get<0>(randEntry) * quantum, std::get<1>(randEntry) * quantum,
                             std::get<2>(randEntry) * quantum));
        }

       private:
        std::vector<size_t> m_numEntries;

        std::vector<std::vector<std::tuple<long, long, long>>> m_randomizationMatrix;

        std::mt19937 m_mersenneTwister;

        Vector3D getBruteForcePerturbation(int index, double quantum)
        {
            //	We have overrun the size of the randomization table so compute the perturbation
            //		brute force with lots of random calls.

            int perturbRange = 1 << (index + 2);

            Vector3D perturbation;

            perturbation =
                Vector3D((m_mersenneTwister() % perturbRange) * quantum, (m_mersenneTwister() % perturbRange) * quantum,
                         (m_mersenneTwister() % perturbRange) * quantum);

            if ((m_mersenneTwister() % 2) == 1)
            {
                perturbation[0] = -perturbation[0];
            }

            if ((m_mersenneTwister() % 2) == 1)
            {
                perturbation[1] = -perturbation[1];
            }

            if ((m_mersenneTwister() % 2) == 1)
            {
                perturbation[2] = -perturbation[2];
            }

            return (perturbation);
        }
    };

    //	Define the static global so it is initialized

    static PerturbationRandomizationMatrix m_randMatrix;  //  NOLINT(cert-err58-cpp)

    Vector3D PerturbationEpsilon::getPerturbation() const
    {
        return (m_randMatrix.getPerturbation(m_numAdjustments, m_quantum));
    }

}  // namespace Cork::Intersection
