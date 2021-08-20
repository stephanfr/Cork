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

#include <boost/filesystem/fstream.hpp>
#include <boost/program_options.hpp>
#include <boost/timer/timer.hpp>
#include <iomanip>
#include <iostream>
#include <list>
#include <ctime>

#include "cork.h"
#include "file_formats/files.h"


#define StopProfile(ignore1, ignore2)
#define StartProfile(ignore1, ignore2)

// Declare the supported options.

//	Accumulator for geotopo metrics

int num_successful_operations = 0;
int num_failed_operations = 0;

int num_two_manifold_results = 0;

uint64_t total_num_vertices = 0;
uint64_t total_num_edges = 0;
uint64_t total_num_triangles = 0;

void WriteMeshStatistics(const Cork::TriangleMesh& mesh, const std::string& filename,
                         boost::filesystem::ofstream& geotopoResults)
{
    Cork::Statistics::GeometricStatistics stats = mesh.ComputeGeometricStatistics();
    Cork::Statistics::TopologicalStatistics topoStats = mesh.ComputeTopologicalStatistics();

    geotopoResults << filename << "\t" << topoStats.IsTwoManifold() << "\t";
    geotopoResults << stats.numVertices() << "\t" << topoStats.numEdges() << "\t" << stats.numTriangles() << "\t";
    geotopoResults << stats.area() << "\t" << stats.volume() << "\t";
    //	geotopoResults << stats.maxEdgeLength() << "\t" << stats.minEdgeLength() << "\t";
    geotopoResults << stats.boundingBox().minima().x() << "\t" << stats.boundingBox().minima().y() << "\t"
                   << stats.boundingBox().minima().z() << "\t";
    geotopoResults << stats.boundingBox().maxima().x() << "\t" << stats.boundingBox().maxima().y() << "\t"
                   << stats.boundingBox().maxima().z() << std::endl;

    num_successful_operations++;

    if (topoStats.IsTwoManifold())
    {
        num_two_manifold_results++;
    }

    total_num_vertices += stats.numVertices();
    total_num_edges += topoStats.numEdges();
    total_num_triangles += stats.numTriangles();
}

int main(int argc, char* argv[])
{
    StopProfile(PROFILE_THREADLEVEL, PROFILE_CURRENTID);

    boost::program_options::options_description desc("Allowed options");
    desc.add_options()("help", "produce help message")("union", "Compute Boolean Union")(
        "intersection", "Compute Boolean Intersection")("difference", "Compute Boolean Difference")(
        "xor", "Compute Boolean Exclusive Or")("input-directory", boost::program_options::value<std::string>(),
                                               "Directory containing input meshes")(
        "output-directory", boost::program_options::value<std::string>(), "Directory to receive output files")(
        "write-results", "Write result meshes")("write-statistics", "Write Statistics");

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
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

    bool computeUnion = vm.count("union") > 0;
    bool computeDifference = vm.count("difference") > 0;
    bool computeIntersection = vm.count("intersection") > 0;
    bool computeXOR = vm.count("xor") > 0;

    bool writeResults = vm.count("write-results") > 0;
    bool writeStats = vm.count("write-statistics") > 0;

    Cork::SolverControlBlock controlBlock = Cork::CorkMesh::GetDefaultControlBlock();

    controlBlock.setUseMultipleThreads(true);

    boost::filesystem::directory_iterator modelRepository(
        boost::filesystem::path(vm["input-directory"].as<std::string>()));
    boost::filesystem::directory_entry resultsDirectory(vm["output-directory"].as<std::string>());

    //	Collect a list of all the model files available

    std::list<boost::filesystem::path> modelFiles;

    for (boost::filesystem::directory_entry modelFile : modelRepository)
    {
        if (modelFile.path().extension() == ".off")
        {
            modelFiles.push_back(modelFile.path());
        }
    }

    //	Open a detailed timing results file

    boost::filesystem::ofstream timingResults;
    boost::filesystem::ofstream cumulativeTimingResults;
    boost::filesystem::ofstream geotopoResults;

    boost::filesystem::path timingResultsFilePath(resultsDirectory);
    boost::filesystem::path cumulativeTimingResultsFilePath(resultsDirectory);
    boost::filesystem::path geotopoResultsFilePath(resultsDirectory);

    if (writeStats)
    {
        timingResultsFilePath += "/stats/timing_results.txt";

        timingResults.open(timingResultsFilePath);

        timingResults << "Filename"
                      << "\t"
                      << "CPU Time"
                      << "\t"
                      << "Wall Time"
                      << "\t"
                      << "# Tris Starting"
                      << "\t"
                      << "# Tris Final"
                      << "\t"
                      << "Virt Mem Start"
                      << "\t"
                      << "Virt Mem End" << std::endl
                      << std::endl;

        timingResults << std::fixed << std::setprecision(4);

        //	Open a cumulative timing results file

        cumulativeTimingResultsFilePath += "/stats/cumulative_timing_results.txt";

        cumulativeTimingResults.open(cumulativeTimingResultsFilePath);

        //	Open a file for collecting statistics on the results

        geotopoResultsFilePath += "/stats/geotopo_results.txt";

        geotopoResults.open(geotopoResultsFilePath);

        geotopoResults << std::fixed << std::setprecision(2);

        geotopoResults << "Name\tIs2Man?\t#Verts\t#Edges\t#Faces\tArea\tVolume\tBBox(min,max)" << std::endl;
    }

    //	Accumulator for all boolean operation elapsed time

    boost::timer::nanosecond_type grand_total_CPU_time = 0;
    boost::timer::nanosecond_type grand_total_wall_time = 0;

    boost::timer::nanosecond_type cumulativeCPUTime = 0;
    boost::timer::nanosecond_type cumulativeWallTime = 0;

    //	Create Unions, Intersections, Differences and XORs of all the files taken two at a time

    typedef std::pair<boost::filesystem::path, std::shared_ptr<Cork::TriangleMesh>> NameAndModel;

    std::list<NameAndModel> models;

    for (boost::filesystem::path currentModel : modelFiles)
    {
        Cork::Files::ReadFileResult readModelResult = Cork::Files::readOFF(currentModel);

        if (!readModelResult.Succeeded())
        {
            std::cout << "Error Reading Model: " << currentModel.filename().string() << std::endl;
            exit(-1);
        }

        //		if ( !readModelResult.ReturnPtr()->edgeTracker().IsTwoManifold() )
        //		{
        //			std::cout << "Model is not 2 Manifold: " << currentModel.filename().string() << std::endl;
        //			exit( -2 );
        //		}

        //		Cork::SelfIntersectionStatistics		selfIntersectionStats =
        //readModelResult.ReturnPtr()->ComputeSelfIntersectionStatistics();

        //		if ( selfIntersectionStats.hasObviousSelfIntersections() )
        //		{
        //			std::cout << currentModel.filename().string() << " : Has Self Intersecting Faces" << std::endl;
        //		}

        models.emplace_back(
            std::make_pair(currentModel, std::shared_ptr<Cork::TriangleMesh>(readModelResult.ReturnPtr().release())));
    }

    for (const NameAndModel& firstModel : models)
    {
        for (const NameAndModel& secondModel : models)
        {
            std::wstring firstName((wchar_t*)firstModel.first.stem().c_str());

            if (firstName.find('_') != std::string::npos)
            {
                firstName = firstName.substr(0, firstName.find('_') - 1);
            }

            std::wstring secondName((wchar_t*)secondModel.first.stem().c_str());

            if (secondName.find('_') != std::string::npos)
            {
                secondName = secondName.substr(0, secondName.find('_') - 1);
            }

            if (firstName == secondName)
            {
                continue;
            }

            std::cout << firstModel.first.filename() << "    " << secondModel.first.filename() << std::endl;

            std::unique_ptr<Cork::CorkMesh> firstMesh = Cork::CorkMesh::FromTriangleMesh(*firstModel.second);
            std::unique_ptr<Cork::CorkMesh> secondMesh = Cork::CorkMesh::FromTriangleMesh(*secondModel.second);

            if (computeUnion)
            {
                StartProfile(PROFILE_THREADLEVEL, PROFILE_CURRENTID);

                Cork::CorkMesh::BooleanOperationResult booleanOpResult = firstMesh->Union(*secondMesh, controlBlock);

                StopProfile(PROFILE_THREADLEVEL, PROFILE_CURRENTID);

                std::string filename = firstModel.first.filename().stem().string() + "_" +
                                       secondModel.first.filename().stem().string() + "_union";

                if (!booleanOpResult.Succeeded())
                {
                    std::cout << "Union Failed: " << booleanOpResult.message() << std::endl;
                    geotopoResults << filename << "    Failed" << std::endl;
                    timingResults << filename << "    Failed" << std::endl;

                    num_failed_operations++;
                }
                else
                {
                    std::unique_ptr<Cork::CorkMesh> unionedMesh(booleanOpResult.ReturnPtr().release());

                    //					std::cout << "Components in finished Mesh: " << unionedMesh->CountComponents() <<
                    //std::endl;

                    cumulativeCPUTime += unionedMesh->GetPerformanceStats().elapsedCPUTimeInNanoSeconds();
                    cumulativeWallTime += unionedMesh->GetPerformanceStats().elapsedWallTimeInNanoSeconds();

                    std::unique_ptr<Cork::TriangleMesh> unionedTriangleMesh(unionedMesh->ToTriangleMesh());

                    boost::filesystem::path resultFilePath(resultsDirectory);

                    resultFilePath += "/";
                    resultFilePath += filename;
                    resultFilePath += ".off";

                    if (writeResults)
                    {
                        Cork::Files::writeOFF(resultFilePath, *unionedTriangleMesh);
                    }

                    if (writeStats)
                    {
                        timingResults << filename << "\t"
                                      << unionedMesh->GetPerformanceStats().elapsedCPUTimeInNanoSeconds() / 1.0E9
                                      << "\t"
                                      << unionedMesh->GetPerformanceStats().elapsedWallTimeInNanoSeconds() / 1.0E9
                                      << "\t" << unionedMesh->GetPerformanceStats().numberOfTrianglesInDisjointUnion()
                                      << "\t" << unionedMesh->GetPerformanceStats().numberOfTrianglesInFinalMesh()
                                      << "\t" << unionedMesh->GetPerformanceStats().startingVirtualMemorySizeInMB()
                                      << "\t" << unionedMesh->GetPerformanceStats().endingVirtualMemorySizeInMB()
                                      << std::endl;

                        WriteMeshStatistics(*unionedTriangleMesh, filename, geotopoResults);
                    }
                }
            }

            if (computeDifference)
            {
                StartProfile(PROFILE_THREADLEVEL, PROFILE_CURRENTID);

                Cork::CorkMesh::BooleanOperationResult booleanOpResult =
                    firstMesh->Difference(*secondMesh, controlBlock);

                StopProfile(PROFILE_THREADLEVEL, PROFILE_CURRENTID);

                std::string filename = firstModel.first.filename().stem().string() + "_" +
                                       secondModel.first.filename().stem().string() + "_difference";

                if (!booleanOpResult.Succeeded())
                {
                    std::cout << "Difference Failed: " << booleanOpResult.message() << std::endl;
                    geotopoResults << filename << "    Failed" << std::endl;
                    timingResults << filename << "    Failed" << std::endl;

                    num_failed_operations++;
                }
                else
                {
                    std::unique_ptr<Cork::CorkMesh> differenceMesh(booleanOpResult.ReturnPtr().release());

                    //					std::cout << "Components in finished Mesh: " << differenceMesh->CountComponents() <<
                    //std::endl;

                    cumulativeCPUTime += differenceMesh->GetPerformanceStats().elapsedCPUTimeInNanoSeconds();
                    cumulativeWallTime += differenceMesh->GetPerformanceStats().elapsedWallTimeInNanoSeconds();

                    std::unique_ptr<Cork::TriangleMesh> differenceTriangleMesh(differenceMesh->ToTriangleMesh());

                    boost::filesystem::path resultFilePath(resultsDirectory);

                    resultFilePath += "/";
                    resultFilePath += filename;
                    resultFilePath += ".off";

                    if (writeResults)
                    {
                        Cork::Files::writeOFF(resultFilePath, *differenceTriangleMesh);
                    }

                    if (writeStats)
                    {
                        timingResults << filename << "\t"
                                      << differenceMesh->GetPerformanceStats().elapsedCPUTimeInNanoSeconds() / 1.0E9
                                      << "\t"
                                      << differenceMesh->GetPerformanceStats().elapsedWallTimeInNanoSeconds() / 1.0E9
                                      << "\t"
                                      << differenceMesh->GetPerformanceStats().numberOfTrianglesInDisjointUnion()
                                      << "\t" << differenceMesh->GetPerformanceStats().numberOfTrianglesInFinalMesh()
                                      << "\t" << differenceMesh->GetPerformanceStats().startingVirtualMemorySizeInMB()
                                      << "\t" << differenceMesh->GetPerformanceStats().endingVirtualMemorySizeInMB()
                                      << std::endl;

                        WriteMeshStatistics(*differenceTriangleMesh, filename, geotopoResults);
                    }
                }
            }

            if (computeIntersection)
            {
                StartProfile(PROFILE_THREADLEVEL, PROFILE_CURRENTID);

                Cork::CorkMesh::BooleanOperationResult booleanOpResult =
                    firstMesh->Intersection(*secondMesh, controlBlock);

                StopProfile(PROFILE_THREADLEVEL, PROFILE_CURRENTID);

                std::string filename = firstModel.first.filename().stem().string() + "_" +
                                       secondModel.first.filename().stem().string() + "_intersection";

                if (!booleanOpResult.Succeeded())
                {
                    std::cout << "Intersection Failed: " << booleanOpResult.message() << std::endl;
                    geotopoResults << filename << "    Failed" << std::endl;
                    timingResults << filename << "    Failed" << std::endl;

                    num_failed_operations++;
                }
                else
                {
                    std::unique_ptr<Cork::CorkMesh> intersectionMesh(booleanOpResult.ReturnPtr().release());

                    //					std::cout << "Components in finished Mesh: " << intersectionMesh->CountComponents() <<
                    //std::endl;

                    cumulativeCPUTime += intersectionMesh->GetPerformanceStats().elapsedCPUTimeInNanoSeconds();
                    cumulativeWallTime += intersectionMesh->GetPerformanceStats().elapsedWallTimeInNanoSeconds();

                    std::unique_ptr<Cork::TriangleMesh> intersectionTriangleMesh(intersectionMesh->ToTriangleMesh());

                    boost::filesystem::path resultFilePath(resultsDirectory);

                    resultFilePath += "/";
                    resultFilePath += filename;
                    resultFilePath += ".off";

                    if (writeResults)
                    {
                        Cork::Files::writeOFF(resultFilePath, *intersectionTriangleMesh);
                    }

                    if (writeStats)
                    {
                        timingResults << filename << "\t"
                                      << intersectionMesh->GetPerformanceStats().elapsedCPUTimeInNanoSeconds() / 1.0E9
                                      << "\t"
                                      << intersectionMesh->GetPerformanceStats().elapsedWallTimeInNanoSeconds() / 1.0E9
                                      << "\t"
                                      << intersectionMesh->GetPerformanceStats().numberOfTrianglesInDisjointUnion()
                                      << "\t" << intersectionMesh->GetPerformanceStats().numberOfTrianglesInFinalMesh()
                                      << "\t" << intersectionMesh->GetPerformanceStats().startingVirtualMemorySizeInMB()
                                      << "\t" << intersectionMesh->GetPerformanceStats().endingVirtualMemorySizeInMB()
                                      << std::endl;

                        WriteMeshStatistics(*intersectionTriangleMesh, filename, geotopoResults);
                    }
                }
            }

            if (computeXOR)
            {
                StartProfile(PROFILE_THREADLEVEL, PROFILE_CURRENTID);

                Cork::CorkMesh::BooleanOperationResult booleanOpResult =
                    firstMesh->SymmetricDifference(*secondMesh, controlBlock);

                StopProfile(PROFILE_THREADLEVEL, PROFILE_CURRENTID);

                std::string filename = firstModel.first.filename().stem().string() + "_" +
                                       secondModel.first.filename().stem().string() + "_xor";

                if (!booleanOpResult.Succeeded())
                {
                    std::cout << "Symmetric Difference Failed: " << booleanOpResult.message() << std::endl;
                    geotopoResults << filename << "    Failed" << std::endl;
                    timingResults << filename << "    Failed" << std::endl;

                    num_failed_operations++;
                }
                else
                {
                    std::unique_ptr<Cork::CorkMesh> XORMesh(booleanOpResult.ReturnPtr().release());

                    //					std::cout << "Components in finished Mesh: " << XORMesh->CountComponents() <<
                    //std::endl;

                    cumulativeCPUTime += XORMesh->GetPerformanceStats().elapsedCPUTimeInNanoSeconds();
                    cumulativeWallTime += XORMesh->GetPerformanceStats().elapsedWallTimeInNanoSeconds();

                    std::unique_ptr<Cork::TriangleMesh> XORTriangleMesh(XORMesh->ToTriangleMesh());

                    boost::filesystem::path resultFilePath(resultsDirectory);

                    resultFilePath += "/";
                    resultFilePath += filename;
                    resultFilePath += ".off";

                    if (writeResults)
                    {
                        Cork::Files::writeOFF(resultFilePath, *XORTriangleMesh);
                    }

                    if (writeStats)
                    {
                        timingResults << filename << "\t"
                                      << XORMesh->GetPerformanceStats().elapsedCPUTimeInNanoSeconds() / 1.0E9 << "\t"
                                      << XORMesh->GetPerformanceStats().elapsedWallTimeInNanoSeconds() / 1.0E9 << "\t"
                                      << XORMesh->GetPerformanceStats().numberOfTrianglesInDisjointUnion() << "\t"
                                      << XORMesh->GetPerformanceStats().numberOfTrianglesInFinalMesh() << "\t"
                                      << XORMesh->GetPerformanceStats().startingVirtualMemorySizeInMB() << "\t"
                                      << XORMesh->GetPerformanceStats().endingVirtualMemorySizeInMB() << std::endl;

                        WriteMeshStatistics(*XORTriangleMesh, filename, geotopoResults);
                    }
                }
            }
        }

        if (writeStats)
        {
            cumulativeTimingResults << firstModel.first.filename() << "\t" << cumulativeCPUTime / 1.0E9 << "\t"
                                    << cumulativeWallTime / 1.0E9 << std::endl;

            grand_total_CPU_time += cumulativeCPUTime;
            grand_total_wall_time += cumulativeWallTime;
        }
    }

    if (writeStats)
    {
		std::time_t		current_date_time;

		time( &current_date_time );

		timingResults << std::endl << std::endl << asctime( localtime( &current_date_time ) ) << std::endl; 

        cumulativeTimingResults << std::endl
                                << "Success"
                                << "\t"
                                << "Failed"
                                << "\t"
                                << "Num 2 Manifold"
                                << "\t"
                                << "Total Verts"
                                << "\t"
                                << "Total Edges"
                                << "\t"
                                << "Total Triangles" << std::endl;

        cumulativeTimingResults << num_successful_operations << "\t" << num_failed_operations << "\t"
                                << num_two_manifold_results << "\t" << total_num_vertices << "\t" << total_num_edges
                                << "\t" << total_num_triangles << std::endl;

        cumulativeTimingResults << std::endl
                                << "Total CPU Time:\t" << grand_total_CPU_time / 1.0E9 << std::endl
								<< "Total Wall Time:\t" << grand_total_wall_time / 1.0E9
                                << std::endl;

		cumulativeTimingResults << std::endl << std::endl << asctime( localtime( &current_date_time ) ) << std::endl;

        geotopoResults << std::endl
                       << "Success"
                       << "\t"
                       << "Failed"
                       << "\t"
                       << "Num 2 Manifold"
                       << "\t"
                       << "Total Verts"
                       << "\t"
                       << "Total Edges"
                       << "\t"
                       << "Total Triangles" << std::endl;

        geotopoResults << num_successful_operations << "\t" << num_failed_operations << "\t" << num_two_manifold_results
                       << "\t" << total_num_vertices << "\t" << total_num_edges << "\t" << total_num_triangles
                       << std::endl;

		geotopoResults << std::endl << std::endl << asctime( localtime( &current_date_time ) ) << std::endl;
    }

    cumulativeTimingResults.flush();
    cumulativeTimingResults.close();

    geotopoResults.flush();
    geotopoResults.close();

    timingResults.flush();
    timingResults.close();

    return (0);
}
