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

#include "math/quantization.hpp"

namespace Cork::Intersection
{
    enum class AdjustPerturbationResultCodes
    {
        SUCCESS = 0,
        MAXIMUM_PERTURBATION_REACHED
    };

    using AdjustPerturbationResult = SEFUtility::ResultWithReturnValue<AdjustPerturbationResultCodes, int>;

    class PerturbationEpsilon
    {
       public:
        explicit PerturbationEpsilon(const Math::Quantizer& quantizer)
            : m_bitsOfPurturbationRange(quantizer.bitsOfPurturbationRange()),
              m_quantum(quantizer.purturbationQuantum()),
              m_numAdjustments(0)
        {
        }

        PerturbationEpsilon() = delete;
        PerturbationEpsilon(const PerturbationEpsilon&) = delete;
        PerturbationEpsilon(PerturbationEpsilon&&) = delete;

        ~PerturbationEpsilon() = default;

        PerturbationEpsilon& operator=(const PerturbationEpsilon&) = delete;
        PerturbationEpsilon& operator=(PerturbationEpsilon&&) = delete;

        bool sufficientRange() const
        {
            return ((m_bitsOfPurturbationRange - m_numAdjustments) >=
                    PERTURBATION_BUFFER_BITS + MINIMUM_PERTURBATION_RANGE_BITS);
        }

        NUMERIC_PRECISION quantum() const { return (m_quantum); }

        int numAdjustments() const { return (m_numAdjustments); }

        AdjustPerturbationResult adjust()
        {
            m_numAdjustments++;

            m_randomRange <<= 1;

            if (!sufficientRange())
            {
                return (AdjustPerturbationResult::failure(AdjustPerturbationResultCodes::MAXIMUM_PERTURBATION_REACHED,
                                                          "Maximum Perturbation reached"));
            }

            return (AdjustPerturbationResult(m_numAdjustments));
        }

        Primitives::Vector3D getPerturbation() const;

       private:
        int m_bitsOfPurturbationRange;
        double m_quantum;

        int m_numAdjustments;

        int m_randomRange;
    };
}  // namespace Cork::Intersection
