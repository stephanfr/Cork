// +-------------------------------------------------------------------------
// | perturbation_epsilon.hpp
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

#include "math/quantization.hpp"

namespace Cork::Intersection
{

    using AdjustPerturbationResult = SEFUtility::ResultWithReturnValue<AdjustPerturbationResultCodes, int>;

    class PerturbationEpsilon
    {
       public:
        explicit PerturbationEpsilon(const Math::Quantizer& quantizer)
            : bits_of_purturbation_range_(quantizer.bitsOfPurturbationRange()),
              quantum_(quantizer.purturbationQuantum()),
              num_adjustments_(0)
        {
        }

        PerturbationEpsilon() = delete;
        PerturbationEpsilon(const PerturbationEpsilon&) = delete;
        PerturbationEpsilon(PerturbationEpsilon&&) = delete;

        ~PerturbationEpsilon() = default;

        PerturbationEpsilon& operator=(const PerturbationEpsilon&) = delete;
        PerturbationEpsilon& operator=(PerturbationEpsilon&&) = delete;

        [[nodiscard]] bool sufficientRange() const
        {
            return ((bits_of_purturbation_range_ - num_adjustments_) >=
                    PERTURBATION_BUFFER_BITS + MINIMUM_PERTURBATION_RANGE_BITS);
        }

        [[nodiscard]] NUMERIC_PRECISION quantum() const { return (quantum_); }

        [[nodiscard]] int numAdjustments() const { return (num_adjustments_); }

        [[nodiscard]] AdjustPerturbationResult adjust()
        {
            num_adjustments_++;

            if (!sufficientRange())
            {
                return (AdjustPerturbationResult::failure(AdjustPerturbationResultCodes::MAXIMUM_PERTURBATION_REACHED,
                                                          "Maximum Perturbation reached"));
            }

            return (AdjustPerturbationResult(num_adjustments_));
        }

        [[nodiscard]] Primitives::Vector3D getPerturbation() const;

       private:
        int bits_of_purturbation_range_;
        double quantum_;

        int num_adjustments_;
    };
}  // namespace Cork::Intersection
