// +-------------------------------------------------------------------------
// | IntersectionProblem.h
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




namespace Cork
{
	namespace Intersection
	{

		
		class PerturbationEpsilon
		{
		public :

			explicit
			PerturbationEpsilon( NUMERIC_PRECISION		value )
				: m_baseEpsilon( value ),
				  m_epsilon( value ),
				  m_epsilonVector( m_epsilon, m_epsilon, m_epsilon ),
				  m_numAdjustments( 0 ),
				  m_adjustmentMultiplier( 0.25 )
			{}



			operator const NUMERIC_PRECISION() const
			{
				return( m_epsilon );
			}

			operator const Cork::Math::Vector3D&() const
			{
				return( m_epsilonVector );
			}


			void			adjust()
			{
				NUMERIC_PRECISION		exponent = (NUMERIC_PRECISION)floor( m_numAdjustments / 2 ) + 1;
				bool					takeReciprocal = (( floor( m_numAdjustments / 2 ) * 2 ) != m_numAdjustments );

				NUMERIC_PRECISION		adjustment = pow( m_adjustmentMultiplier, (float)exponent );

				if( takeReciprocal )
				{
					adjustment = 1 / adjustment;
				}

				m_epsilon = m_baseEpsilon * adjustment;

				m_epsilonVector = Cork::Math::Vector3D( m_epsilon, m_epsilon, m_epsilon );

#ifdef CORK_DEBUG
				std::cout << "***** Adjusting Purturbation.  New Epsilon: " << m_epsilon << " *****" << std::endl;
#endif

				m_numAdjustments++;
			}

		private :

			NUMERIC_PRECISION			m_baseEpsilon;
			NUMERIC_PRECISION			m_epsilon;

			Cork::Math::Vector3D		m_epsilonVector;

			NUMERIC_PRECISION			m_adjustmentMultiplier;
			int							m_numAdjustments;
		};



		class IntersectionProblemIfx
		{
		public:

			
			enum class IntersectionProblemResultCodes { SUCCESS = 0, SUBDIVIDE_FAILED, EXHAUSTED_PURTURBATION_RETRIES };

			typedef SEFUtility::Result<IntersectionProblemResultCodes>		IntersectionProblemResult;




			static std::unique_ptr<IntersectionProblemIfx>		GetProblem( MeshBase&							owner,
																			const Cork::Math::BBox3D&			intersectionBBox,
																			const PerturbationEpsilon&			purturbation );

			virtual ~IntersectionProblemIfx()
			{}
	
//			virtual SelfIntersectionStatistics			ComputeSelfIntersectionStatistics() = 0;

			virtual IntersectionProblemResult			FindIntersections() = 0;
			virtual IntersectionProblemResult			ResolveAllIntersections() = 0;

			virtual void								commit() = 0;
		};

	}
}



