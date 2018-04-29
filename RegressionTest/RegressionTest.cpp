// +-------------------------------------------------------------------------
// | RegressionTest.cpp
// | 
// | Author: Stephan Friedl
// +-------------------------------------------------------------------------
// | COPYRIGHT:
// |    Copyright Stephan Friedl 2015
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

// RegressionTest.cpp : Defines the entry point for the console application.
//


#include <list>
#include <iostream>
#include <iomanip>

#include <boost\filesystem\fstream.hpp>
#include <boost\timer\timer.hpp>

#include <boost\program_options.hpp>

#include "cork.h"
#include "FileFormats\files.h"


//#include "C:\Program Files (x86)\Microsoft Visual Studio 12.0\Team Tools\Performance Tools\PerfSDK\VSPerf.h"

#define	StopProfile( ignore1, ignore2 )
#define StartProfile( ignore1, ignore2 )


// Declare the supported options.




void		WriteMeshStatistics( const Cork::TriangleMesh&			mesh,
								 const std::string&					filename,
								 boost::filesystem::ofstream&		geotopoResults )
{
	Cork::Statistics::GeometricStatistics		stats = mesh.ComputeGeometricStatistics();
	Cork::Statistics::TopologicalStatistics		topoStats = mesh.ComputeTopologicalStatistics();

	geotopoResults << filename << "\t" << topoStats.IsTwoManifold() << "\t";
	geotopoResults << stats.numVertices() << "\t" << topoStats.numEdges() << "\t" << stats.numTriangles() << "\t";
	geotopoResults << stats.area() << "\t" << stats.volume() << "\t";
//	geotopoResults << stats.maxEdgeLength() << "\t" << stats.minEdgeLength() << "\t";
	geotopoResults << stats.boundingBox().minima().x() << "\t" << stats.boundingBox().minima().y() << "\t" << stats.boundingBox().minima().z() << "\t";
	geotopoResults << stats.boundingBox().maxima().x() << "\t" << stats.boundingBox().maxima().y() << "\t" << stats.boundingBox().maxima().z() << std::endl;
}




int main(int argc, char* argv[])
{
	StopProfile( PROFILE_THREADLEVEL, PROFILE_CURRENTID );


	boost::program_options::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("union", "Compute Boolean Union" )
		("intersection", "Compute Boolean Intersection" )
		("difference", "Compute Boolean Difference" )
		("xor", "Compute Boolean Exclusive Or" )
		("input-directory", boost::program_options::value<std::string>(), "Directory containing input meshes")
		("output-directory", boost::program_options::value<std::string>(), "Directory to receive output files")
		("write-results", "Write result meshes")
		("write-statistics", "Write Statistics");

	boost::program_options::variables_map vm;
	boost::program_options::store( boost::program_options::parse_command_line( argc, argv, desc ), vm );
	boost::program_options::notify(vm);    

	if (vm.count("help"))
	{
		std::cout << desc << "\n";
		return 1;
	}

	if (vm.count("input-directory"))
	{
		std::cout << "Input Directory: " << vm["input-directory"].as<std::string>() << std::endl;
	}

	if (vm.count("output-directory"))
	{
		std::cout << "Output Directory: " << vm["output-directory"].as<std::string>() << std::endl;
	}

	bool	computeUnion = vm.count("union") > 0 ;
	bool	computeDifference = vm.count("difference") > 0;
	bool	computeIntersection = vm.count("intersection") > 0;
	bool	computeXOR = vm.count("xor") > 0;

	bool	writeResults = vm.count("write-results") > 0;
	bool	writeStats = vm.count("write-statistics") > 0;

	bool	multiThreaded = false;


	Cork::SolverControlBlock		controlBlock = Cork::CorkMesh::GetDefaultControlBlock();

	controlBlock.setUseMultipleThreads( true );

	boost::filesystem::directory_iterator		modelRepository(boost::filesystem::path( vm["input-directory"].as<std::string>() ));
	boost::filesystem::directory_entry			resultsDirectory( vm["output-directory"].as<std::string>() );

	//	Collect a list of all the model files available
	
	std::list<boost::filesystem::path>			modelFiles;

	for (boost::filesystem::directory_entry modelFile : modelRepository)
	{
		if (modelFile.path().extension() == ".off")
		{
			modelFiles.push_back(modelFile.path());
		}
	}

	//	Open a detailed timing results file

	boost::filesystem::ofstream			timingResults;
	boost::filesystem::ofstream			cumulativeTimingResults;
	boost::filesystem::ofstream			geotopoResults;

	boost::filesystem::path				timingResultsFilePath( resultsDirectory );
	boost::filesystem::path				cumulativeTimingResultsFilePath(resultsDirectory);
	boost::filesystem::path				geotopoResultsFilePath(resultsDirectory);


	if( writeStats )
	{
		timingResultsFilePath += "\\timing_results.txt";

		timingResults.open( timingResultsFilePath );

		//	Open a cumulative timing results file

		cumulativeTimingResultsFilePath += "\\cumulative_timing_results.txt";

		cumulativeTimingResults.open(cumulativeTimingResultsFilePath);

		//	Open a file for collecting statistics on the results


		geotopoResultsFilePath += "\\geotopo_results.txt";

		geotopoResults.open(geotopoResultsFilePath);

		geotopoResults << std::fixed << std::setprecision(2);

		geotopoResults << "Name\tIs2Man?\t#Verts\t#Edges\t#Faces\tArea\tVolume\tBBox(min,max)" << std::endl;
	}

	//	Accumulator for all boolean operation elapsed time

	boost::timer::nanosecond_type		cumulativeCPUTime = 0;
	boost::timer::nanosecond_type		cumulativeWallTime = 0;

	//	Create Unions, Intersections, Differences and XORs of all the files taken two at a time

	typedef std::pair < boost::filesystem::path, std::shared_ptr < Cork::TriangleMesh >>	NameAndModel;
	
	std::list< NameAndModel >		models;

	for ( boost::filesystem::path currentModel : modelFiles )
	{
		Cork::Files::ReadFileResult readModelResult = Cork::Files::readOFF( currentModel );

		if ( !readModelResult.Succeeded() )
		{
			std::cout << "Error Reading Model: " << currentModel.filename().string() << std::endl;
			exit( -1 );
		}

//		if ( !readModelResult.ReturnPtr()->edgeTracker().IsTwoManifold() )
//		{
//			std::cout << "Model is not 2 Manifold: " << currentModel.filename().string() << std::endl;
//			exit( -2 );
//		}

//		Cork::SelfIntersectionStatistics		selfIntersectionStats = readModelResult.ReturnPtr()->ComputeSelfIntersectionStatistics();

//		if ( selfIntersectionStats.hasObviousSelfIntersections() )
//		{
//			std::cout << currentModel.filename().string() << " : Has Self Intersecting Faces" << std::endl;
//		}

		models.emplace_back( std::make_pair( currentModel, std::shared_ptr < Cork::TriangleMesh >( readModelResult.ReturnPtr().release() ) ) );
	}
			
	for ( const NameAndModel& firstModel : models )
	{
		for ( const NameAndModel& secondModel : models )
		{
			if ( firstModel.first == secondModel.first )
			{
				continue;
			}

			std::cout << firstModel.first.filename() << "    " << secondModel.first.filename() << std::endl;

			std::unique_ptr<Cork::CorkMesh>		firstMesh = Cork::CorkMesh::FromTriangleMesh( *firstModel.second );
			std::unique_ptr<Cork::CorkMesh>		secondMesh = Cork::CorkMesh::FromTriangleMesh( *secondModel.second );

			if( computeUnion )
			{
				StartProfile( PROFILE_THREADLEVEL, PROFILE_CURRENTID );

				Cork::CorkMesh::BooleanOperationResult		booleanOpResult = firstMesh->Union( *secondMesh, controlBlock );

				StopProfile( PROFILE_THREADLEVEL, PROFILE_CURRENTID );

				std::string		filename = firstModel.first.filename().stem().string() + "_" + secondModel.first.filename().stem().string() + "_union";

				if( !booleanOpResult.Succeeded() )
				{
					std::cout << "Union Failed: " << booleanOpResult.message() << std::endl;
					geotopoResults << filename << "    Failed" << std::endl;
					timingResults << filename << "    Failed" << std::endl;
				}
				else
				{
					std::unique_ptr<Cork::CorkMesh>		unionedMesh( booleanOpResult.ReturnPtr().release() );

					std::cout << "Components in finished Mesh: " << unionedMesh->CountComponents() << std::endl;


					cumulativeCPUTime += unionedMesh->GetPerformanceStats().elapsedCPUTimeInNanoSeconds();
					cumulativeWallTime += unionedMesh->GetPerformanceStats().elapsedWallTimeInNanoSeconds();

					std::unique_ptr<Cork::TriangleMesh>	unionedTriangleMesh(unionedMesh->ToTriangleMesh());

					boost::filesystem::path	resultFilePath(resultsDirectory);

					resultFilePath += "\\";
					resultFilePath += filename;
					resultFilePath += ".off";

					if (writeResults)
					{
						Cork::Files::writeOFF(resultFilePath, *unionedTriangleMesh);
					}

					if( writeStats )
					{
						timingResults << filename << "\t" << unionedMesh->GetPerformanceStats().elapsedCPUTimeInNanoSeconds()
							                      << "\t" << unionedMesh->GetPerformanceStats().elapsedWallTimeInNanoSeconds()
												  << "\t" << unionedMesh->GetPerformanceStats().numberOfTrianglesInDisjointUnion()
												  << "\t" << unionedMesh->GetPerformanceStats().numberOfTrianglesInFinalMesh()
												  << "\t" << unionedMesh->GetPerformanceStats().startingVirtualMemorySizeInMB()
												  << "\t" << unionedMesh->GetPerformanceStats().endingVirtualMemorySizeInMB()
												  << std::endl;

						WriteMeshStatistics( *unionedTriangleMesh, filename, geotopoResults );
					}
				}
			}

			if( computeDifference )
			{
				StartProfile( PROFILE_THREADLEVEL, PROFILE_CURRENTID );
				
				Cork::CorkMesh::BooleanOperationResult		booleanOpResult = firstMesh->Difference( *secondMesh, controlBlock );
				
				StopProfile( PROFILE_THREADLEVEL, PROFILE_CURRENTID );
				
				std::string		filename = firstModel.first.filename().stem().string() + "_" + secondModel.first.filename().stem().string() + "_difference";

				if( !booleanOpResult.Succeeded() )
				{
					std::cout << "Difference Failed: " << booleanOpResult.message() << std::endl;
					geotopoResults << filename << "    Failed" << std::endl;
					timingResults << filename << "    Failed" << std::endl;
				}
				else
				{
					std::unique_ptr<Cork::CorkMesh>		differenceMesh( booleanOpResult.ReturnPtr().release() );

					std::cout << "Components in finished Mesh: " << differenceMesh->CountComponents() << std::endl;

					cumulativeCPUTime += differenceMesh->GetPerformanceStats().elapsedCPUTimeInNanoSeconds();
					cumulativeWallTime += differenceMesh->GetPerformanceStats().elapsedWallTimeInNanoSeconds();

					std::unique_ptr<Cork::TriangleMesh>	differenceTriangleMesh( differenceMesh->ToTriangleMesh() );

					boost::filesystem::path	resultFilePath(resultsDirectory);

					resultFilePath += "\\";
					resultFilePath += filename;
					resultFilePath += ".off";

					if (writeResults)
					{
						Cork::Files::writeOFF(resultFilePath, *differenceTriangleMesh);
					}

					if( writeStats )
					{
						timingResults << filename << "\t" << differenceMesh->GetPerformanceStats().elapsedCPUTimeInNanoSeconds()
							                      << "\t" << differenceMesh->GetPerformanceStats().elapsedWallTimeInNanoSeconds()
							                      << "\t" << differenceMesh->GetPerformanceStats().numberOfTrianglesInDisjointUnion()
												  << "\t" << differenceMesh->GetPerformanceStats().numberOfTrianglesInFinalMesh()
												  << "\t" << differenceMesh->GetPerformanceStats().startingVirtualMemorySizeInMB()
												  << "\t" << differenceMesh->GetPerformanceStats().endingVirtualMemorySizeInMB()
												  << std::endl;

						WriteMeshStatistics( *differenceTriangleMesh, filename, geotopoResults );
					}
				}
			}

			if( computeIntersection )
			{
				StartProfile( PROFILE_THREADLEVEL, PROFILE_CURRENTID );

				Cork::CorkMesh::BooleanOperationResult		booleanOpResult = firstMesh->Intersection( *secondMesh, controlBlock );

				StopProfile( PROFILE_THREADLEVEL, PROFILE_CURRENTID );

				std::string		filename = firstModel.first.filename().stem().string() + "_" + secondModel.first.filename().stem().string() + "_intersection";

				if( !booleanOpResult.Succeeded() )
				{
					std::cout << "Intersection Failed: " << booleanOpResult.message() << std::endl;
					geotopoResults << filename << "    Failed" << std::endl;
					timingResults << filename << "    Failed" << std::endl;
				}
				else
				{
					std::unique_ptr<Cork::CorkMesh>		intersectionMesh( booleanOpResult.ReturnPtr().release() );

					std::cout << "Components in finished Mesh: " << intersectionMesh->CountComponents() << std::endl;

					cumulativeCPUTime += intersectionMesh->GetPerformanceStats().elapsedCPUTimeInNanoSeconds();
					cumulativeWallTime += intersectionMesh->GetPerformanceStats().elapsedWallTimeInNanoSeconds();

					std::unique_ptr<Cork::TriangleMesh>	intersectionTriangleMesh( intersectionMesh->ToTriangleMesh() );

					boost::filesystem::path	resultFilePath(resultsDirectory);

					resultFilePath += "\\";
					resultFilePath += filename;
					resultFilePath += ".off";

					if (writeResults)
					{
						Cork::Files::writeOFF(resultFilePath, *intersectionTriangleMesh);
					}

					if( writeStats )
					{
						timingResults << filename << "\t" << intersectionMesh->GetPerformanceStats().elapsedCPUTimeInNanoSeconds()
							                      << "\t" << intersectionMesh->GetPerformanceStats().elapsedWallTimeInNanoSeconds()
							                      << "\t" << intersectionMesh->GetPerformanceStats().numberOfTrianglesInDisjointUnion()
												  << "\t" << intersectionMesh->GetPerformanceStats().numberOfTrianglesInFinalMesh()
												  << "\t" << intersectionMesh->GetPerformanceStats().startingVirtualMemorySizeInMB()
												  << "\t" << intersectionMesh->GetPerformanceStats().endingVirtualMemorySizeInMB()
												  << std::endl;

						WriteMeshStatistics( *intersectionTriangleMesh, filename, geotopoResults );
					}
				}
			}

			if( computeXOR )
			{
				StartProfile( PROFILE_THREADLEVEL, PROFILE_CURRENTID );

				Cork::CorkMesh::BooleanOperationResult		booleanOpResult = firstMesh->SymmetricDifference( *secondMesh, controlBlock );

				StopProfile( PROFILE_THREADLEVEL, PROFILE_CURRENTID );

				std::string		filename = firstModel.first.filename().stem().string() + "_" + secondModel.first.filename().stem().string() + "_xor";

				if( !booleanOpResult.Succeeded() )
				{
					std::cout << "Symmetric Difference Failed: " << booleanOpResult.message() << std::endl;
					geotopoResults << filename << "    Failed" << std::endl;
					timingResults << filename << "    Failed" << std::endl;
				}
				else
				{
					std::unique_ptr<Cork::CorkMesh>		XORMesh( booleanOpResult.ReturnPtr().release() );

					std::cout << "Components in finished Mesh: " << XORMesh->CountComponents() << std::endl;

					cumulativeCPUTime += XORMesh->GetPerformanceStats().elapsedCPUTimeInNanoSeconds();
					cumulativeWallTime += XORMesh->GetPerformanceStats().elapsedWallTimeInNanoSeconds();

					std::unique_ptr<Cork::TriangleMesh>	XORTriangleMesh( XORMesh->ToTriangleMesh() );

					boost::filesystem::path	resultFilePath(resultsDirectory);

					resultFilePath += "\\";
					resultFilePath += filename;
					resultFilePath += ".off";

					if (writeResults)
					{
						Cork::Files::writeOFF(resultFilePath, *XORTriangleMesh);
					}

					if( writeStats )
					{
						timingResults << filename << "\t" << XORMesh->GetPerformanceStats().elapsedCPUTimeInNanoSeconds()
							                      << "\t" << XORMesh->GetPerformanceStats().elapsedWallTimeInNanoSeconds()
							                      << "\t" << XORMesh->GetPerformanceStats().numberOfTrianglesInDisjointUnion()
												  << "\t" << XORMesh->GetPerformanceStats().numberOfTrianglesInFinalMesh()
												  << "\t" << XORMesh->GetPerformanceStats().startingVirtualMemorySizeInMB()
												  << "\t" << XORMesh->GetPerformanceStats().endingVirtualMemorySizeInMB()
												  << std::endl;

						WriteMeshStatistics( *XORTriangleMesh, filename, geotopoResults );
					}
				}
			}
		}

		if( writeStats )
		{
			cumulativeTimingResults << firstModel.first.filename() << "\t" << cumulativeCPUTime << "\t" << cumulativeWallTime << std::endl;
		}
	}

	return( 0 );
}
