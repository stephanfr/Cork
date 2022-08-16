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

#include "perturbation_epsilon.hpp"

#include <random>
#include <vector>

#include "../constants.hpp"

namespace Cork::Intersection
{
    class PerturbationRandomizationMatrix
    {
       public:
        PerturbationRandomizationMatrix() : mersenne_twister_(time(nullptr))  //  NOLINT(cert-msc32-c, cert-msc51-cpp)
        {
            for (uint32_t num_permutations = MIN_NUM_PERMUTATIONS; num_permutations <= MAX_NUM_PERMUTATIONS;
                 num_permutations <<= 1U)
            {
                randomization_matrix_.emplace_back(std::vector<std::tuple<int32_t, int32_t, int32_t>>());

                auto& current_vec = randomization_matrix_.back();

                for (int32_t i = 0; i <= num_permutations; i++)
                {
                    for (int32_t j = 0; j <= num_permutations; j++)
                    {
                        for (int32_t k = 0; k <= num_permutations; k++)
                        {
                            current_vec.emplace_back(std::tuple<int32_t, int32_t, int32_t>(i, j, k));

                            if (i > 0)
                            {
                                current_vec.emplace_back(std::tuple<int32_t, int32_t, int32_t>(-i, j, k));
                            }

                            if (j > 0)
                            {
                                current_vec.emplace_back(std::tuple<int32_t, int32_t, int32_t>(i, -j, k));
                            }

                            if (k > 0)
                            {
                                current_vec.emplace_back(std::tuple<int32_t, int32_t, int32_t>(i, j, -k));
                            }

                            if ((i > 0) && (j > 0))
                            {
                                current_vec.emplace_back(std::tuple<int32_t, int32_t, int32_t>(-i, -j, k));
                            }

                            if ((i > 0) && (k > 0))
                            {
                                current_vec.emplace_back(std::tuple<int32_t, int32_t, int32_t>(-i, j, -k));
                            }

                            if ((j > 0) && (k > 0))
                            {
                                current_vec.emplace_back(std::tuple<int32_t, int32_t, int32_t>(i, -j, -k));
                            }

                            if ((i > 0) && (j > 0) && (k > 0))
                            {
                                current_vec.emplace_back(std::tuple<int32_t, int32_t, int32_t>(-i, -j, -k));
                            }
                        }
                    }
                }

                num_entries_.emplace_back(current_vec.size());
            }
        }

        Vector3D getPerturbation(int index, double quantum)
        {
            if (index >= num_entries_.size())
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

            size_t array_index(mersenne_twister_() % num_entries_[index]);

            const std::tuple<int32_t, int32_t, int32_t>& rand_entry = randomization_matrix_[index][array_index];

            return (Vector3D(std::get<0>(rand_entry) * quantum, std::get<1>(rand_entry) * quantum,
                             std::get<2>(rand_entry) * quantum));
        }

       private:
        friend class PerturbationEpsilon;

        std::vector<size_t> num_entries_;

        std::vector<std::vector<std::tuple<int32_t, int32_t, int32_t>>> randomization_matrix_;

        std::mt19937 mersenne_twister_;

        static PerturbationRandomizationMatrix rand_matrix_;        //  NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

        Vector3D getBruteForcePerturbation(uint32_t index, double quantum)
        {
            //	We have overrun the size of the randomization table so compute the perturbation
            //		brute force with lots of random calls.

            uint32_t perturb_range = 1U << (index + 2);

            Vector3D perturbation;

            perturbation = Vector3D(double(mersenne_twister_() % perturb_range) * quantum,
                                    double(mersenne_twister_() % perturb_range) * quantum,
                                    double(mersenne_twister_() % perturb_range) * quantum);

            if ((mersenne_twister_() % 2) == 1)
            {
                perturbation[0] = -perturbation[0];
            }

            if ((mersenne_twister_() % 2) == 1)
            {
                perturbation[1] = -perturbation[1];
            }

            if ((mersenne_twister_() % 2) == 1)
            {
                perturbation[2] = -perturbation[2];
            }

            return perturbation;
        }
    };

    //	Define the static so it is initialized

    PerturbationRandomizationMatrix PerturbationRandomizationMatrix::rand_matrix_;        //  NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

    Vector3D PerturbationEpsilon::getPerturbation() const
    {
        return PerturbationRandomizationMatrix::rand_matrix_.getPerturbation(num_adjustments_, quantum_);
    }

}  // namespace Cork::Intersection
