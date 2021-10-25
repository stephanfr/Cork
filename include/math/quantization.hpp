// +-------------------------------------------------------------------------
// | quantization.h
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

#include <cmath>

#include "CPPResult.hpp"
#include "primitives/primitives.hpp"

namespace Cork::Math
{
    constexpr int MINIMUM_BITS_REQUIRED_FOR_QUANTIZATION = 10;

    class Quantizer
    {
       public:
        enum class QuantizerResultCodes
        {
            SUCCESS = 0,

            EXCESSIVE_DYNAMIC_RANGE_FOR_QUANTIZATION,
            INSUFFICIENT_PERTURBATION_RANGE
        };
        using GetQuantizerResult = SEFUtility::ResultWithReturnValue<QuantizerResultCodes, Quantizer>;

        static GetQuantizerResult get_quantizer(double max_magnitude, double min_edge_length)
        {
            Quantizer quantizer;

            QuantizerResultCodes result_code = quantizer.calibrate(max_magnitude, min_edge_length);

            if (result_code != QuantizerResultCodes::SUCCESS)
            {
                std::cout << "GetQuantizer Excessive dynamic range" << std::endl;

                return GetQuantizerResult::failure(result_code,
                                                   "Mesh triangulation likely has excessive "
                                                   "difference between the max and min edge lengths");
            }

            return GetQuantizerResult::success(quantizer);
        }

        Quantizer(const Quantizer& quantizerToCopy)
            : m_magnify(quantizerToCopy.m_magnify), m_reshrink(quantizerToCopy.m_reshrink)
        {
        }

        int bitsOfPurturbationRange() const { return (m_bitsOfPurturbationRange); }

        double purturbationQuantum() const { return (m_reshrink); }

        int quantize2int(double number) const { return (int(number * m_magnify)); }

        double quantizedInt2double(int number) const { return (m_reshrink * double(number)); }

        double quantize(double number) const { return (m_reshrink * double(int(number * m_magnify))); }

        double quantize(float number) const { return ((float)m_reshrink * float(int(number * (float)m_magnify))); }

        double reshrink(double number) const { return (m_reshrink * number); }

        Primitives::Vector3D quantize(const Primitives::Vector3D& vectorToQuantize) const
        {
            return (Primitives::Vector3D(quantize(vectorToQuantize.x()), quantize(vectorToQuantize.y()),
                                         quantize(vectorToQuantize.z())));
        }

       private:
        Quantizer() = default;

        //	Given the specified number of bits, and bound on the coordinate values of points,
        //		fit as fine-grained a grid as possible over the space.

        QuantizerResultCodes calibrate(double maxMagnitude, double minEdgeLength)
        {
            int maxCoordinateValueBinaryExponent;

            std::frexp(maxMagnitude, &maxCoordinateValueBinaryExponent);

            maxCoordinateValueBinaryExponent++;  // ensure that 2^max_exponent > maximumMagnitude

            //	Set constants
            m_magnify = std::pow(2.0, QUANTIZATION_BITS - maxCoordinateValueBinaryExponent);

            //	We are guaranteed that maximumMagnitude * MAGNIFY < 2.0^BITS
            m_reshrink = std::pow(2.0, maxCoordinateValueBinaryExponent - QUANTIZATION_BITS);

            int minEdgeLengthBinaryExponent;

            std::frexp(minEdgeLength, &minEdgeLengthBinaryExponent);

            int quantaBitsPerMinEdge =
                minEdgeLengthBinaryExponent - (maxCoordinateValueBinaryExponent - QUANTIZATION_BITS);

            if (quantaBitsPerMinEdge < MINIMUM_BITS_REQUIRED_FOR_QUANTIZATION)
            {
                return QuantizerResultCodes::EXCESSIVE_DYNAMIC_RANGE_FOR_QUANTIZATION;
            }

            m_bitsOfPurturbationRange = abs(quantaBitsPerMinEdge);

            if (m_bitsOfPurturbationRange < PERTURBATION_BUFFER_BITS + MINIMUM_PERTURBATION_RANGE_BITS)
            {
                return QuantizerResultCodes::INSUFFICIENT_PERTURBATION_RANGE;
            }

            return QuantizerResultCodes::SUCCESS;
        }

       private:
        double m_magnify;
        double m_reshrink;

        int m_bitsOfPurturbationRange;
    };

}  // namespace Cork::Math
