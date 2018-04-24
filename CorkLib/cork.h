// +-------------------------------------------------------------------------
// | cork.h
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


// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the CORKLIB_EXPORTS
// symbol defined on the command line. This symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// CORKLIB_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.
#ifdef CORKLIB_EXPORTS
#define CORKLIB_API __declspec(dllexport)
#else
#define CORKLIB_API __declspec(dllimport)
#endif


#include "Util\Result.h"

#include "TriangleMesh.h"



#ifndef uint
#define uint unsigned int
#endif

namespace Cork
{

	class SolverControlBlock
	{
	public :

		explicit SolverControlBlock( bool		useMultipleThreads,
									 long		minTrianglesForThreading,
									 bool		usePooledWorkspaces )
			: m_useMultipleThreads( useMultipleThreads ),
			  m_minTrianglesForThreading( minTrianglesForThreading ),
			  m_usePooledWorkspaces( usePooledWorkspaces ),
			  m_numTriangles(0)
		{}


		void	setNumTriangles( long		numTriangles )
		{
			m_numTriangles = numTriangles;
		}


		bool	useMultipleThreads() const
		{
			return( m_useMultipleThreads && ( m_numTriangles > m_minTrianglesForThreading ));
		}

		void	setUseMultipleThreads( bool		newValue )
		{
			m_useMultipleThreads = newValue;
		}

		long	minTrianglesForThreading() const
		{
			return( m_minTrianglesForThreading );
		}

		void	setMinTrianglesForThreading( long		minTriangles )
		{
			m_minTrianglesForThreading = minTriangles;
		}

		bool	usePooledWorkspaces() const
		{
			return( m_usePooledWorkspaces );
		}



	private :

		long					m_numTriangles;

		bool					m_useMultipleThreads;
		long					m_minTrianglesForThreading;

		bool					m_usePooledWorkspaces;
	};



	class SolverPerformanceStatisticsIfx
	{
	public :

		virtual ~SolverPerformanceStatisticsIfx()
		{};

		virtual unsigned long 			numberOfTrianglesInDisjointUnion() const = 0;
		virtual unsigned long 			numberOfTrianglesInFinalMesh() const = 0;
		virtual unsigned long long		elapsedCPUTimeInNanoSeconds() const = 0;
		virtual unsigned long long		elapsedWallTimeInNanoSeconds() const = 0;
		virtual unsigned long 			startingVirtualMemorySizeInMB() const = 0;
		virtual unsigned long 			endingVirtualMemorySizeInMB() const = 0;
	};
	


	enum class BooleanOperationResultCodes
	{
		SUCCESS = 0,

		ERROR_DURING_BOOLEAN_PROBLEM_SETUP
	};



	class CorkMesh
	{
	public :

		typedef SEFUtility::ResultWithUniqueReturnPtr<BooleanOperationResultCodes, CorkMesh>		BooleanOperationResult;



		CORKLIB_API
		static std::unique_ptr<CorkMesh>				FromTriangleMesh( const Cork::TriangleMesh&		triangleMesh );

		CORKLIB_API
		static const SolverControlBlock&						GetDefaultControlBlock();


		virtual ~CorkMesh() {};


		virtual BooleanOperationResult					Union( const CorkMesh&									rhs,
															   const SolverControlBlock&							solverControlBlock = GetDefaultControlBlock() ) const = 0;

		virtual BooleanOperationResult					Difference( const CorkMesh&								rhs,
																	const SolverControlBlock&						solverControlBlock = GetDefaultControlBlock() ) const = 0;

		virtual BooleanOperationResult					Intersection( const CorkMesh&							rhs,
																	  const SolverControlBlock&					solverControlBlock = GetDefaultControlBlock() ) const = 0;

		virtual BooleanOperationResult					SymmetricDifference( const CorkMesh&					rhs,
																			 const SolverControlBlock&			solverControlBlock = GetDefaultControlBlock() ) const = 0;


		virtual std::unique_ptr<TriangleMesh>			ToTriangleMesh() const = 0;

		virtual const SolverPerformanceStatisticsIfx&	GetPerformanceStats() const = 0;
	};

}

