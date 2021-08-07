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

#include<cmath>

#include "../Math/Primitives.h"



namespace Cork
{
	namespace Quantization
	{

		class Quantizer
		{
		public:

			Quantizer( double	maxMagnitude,
				       double	minEdgeLength)
			{
				calibrate( maxMagnitude, minEdgeLength);
			}


			Quantizer(const Quantizer&		quantizerToCopy)
				: m_magnify(quantizerToCopy.m_magnify),
				  m_reshrink(quantizerToCopy.m_reshrink)
			{}


			bool	sufficientPerturbationRange() const
			{
				return( m_bitsOfPurturbationRange >= PERTURBATION_BUFFER_BITS + MINIMUM_PERTURBATION_RANGE_BITS);
			}

			int	bitsOfPurturbationRange() const
			{
				return(m_bitsOfPurturbationRange);
			}

			double	purturbationQuantum() const
			{
				return(m_reshrink);
			}

			int quantize2int(double number) const
			{
				return(int(number * m_magnify));
			}

			double quantizedInt2double(int number) const
			{
				return(m_reshrink * double(number));
			}

			double quantize(double number) const
			{
				return(m_reshrink * double(int(number * m_magnify)));
			}

			double quantize(float number) const
			{
				return((float)m_reshrink * float(int(number * (float)m_magnify)));
			}

			double reshrink(double  number) const
			{
				return(m_reshrink * number);
			}



			Cork::Math::Vector3D	quantize(const Cork::Math::Vector3D&		vectorToQuantize) const
			{
				return(Cork::Math::Vector3D(quantize(vectorToQuantize.x()), quantize(vectorToQuantize.y()), quantize(vectorToQuantize.z())));
			}


		private :

			//	Given the specified number of bits, and bound on the coordinate values of points,
			//		fit as fine-grained a grid as possible over the space.

			void calibrate( double	maxMagnitude,
							double	minEdgeLength )
			{
				int maxCoordinateValueBinaryExponent;

				std::frexp(maxMagnitude, &maxCoordinateValueBinaryExponent);

				maxCoordinateValueBinaryExponent++; // ensure that 2^max_exponent > maximumMagnitude

				// set constants
				m_magnify = std::pow(2.0, QUANTIZATION_BITS - maxCoordinateValueBinaryExponent);

				// we are guaranteed that maximumMagnitude * MAGNIFY < 2.0^BITS
				m_reshrink = std::pow(2.0, maxCoordinateValueBinaryExponent - QUANTIZATION_BITS);

				int		minEdgeLengthBinaryExponent;

				std::frexp(minEdgeLength, &minEdgeLengthBinaryExponent);

				int		quantaBitsPerMinEdge = minEdgeLengthBinaryExponent - (maxCoordinateValueBinaryExponent - QUANTIZATION_BITS);

				if (quantaBitsPerMinEdge < 10 )
				{
					std::cout << "Insufficient number of quanta bits for min edge length" << std::endl;
				}

				m_bitsOfPurturbationRange = abs(quantaBitsPerMinEdge);
			}

		private :

			double	m_magnify;
			double	m_reshrink;

			int		m_bitsOfPurturbationRange;
		};

	} // end namespace Quantization

}	//	end namespace Cork

