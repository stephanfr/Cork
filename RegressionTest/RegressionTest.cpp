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

#include <filesystem>
#include <fstream>

#include <boost/program_options.hpp>
#include <boost/timer/timer.hpp>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <list>

#include "cork.hpp"
#include "file_formats/files.hpp"

constexpr double NUM_NANOSECONDS_PER_SECOND = 1E09;


// Declare the supported options.

//	Accumulator for geotopo metrics

int num_successful_operations = 0;
int num_failed_operations = 0;

int num_two_manifold_results = 0;

uint64_t total_num_vertices = 0;
uint64_t total_num_edges = 0;
uint64_t total_num_triangles = 0;

void WriteMeshStatistics(const Cork::TriangleMesh& mesh, const std::string& filename,
                         std::ofstream& geotopoResults)
{
    Cork::Statistics::GeometricStatistics stats = mesh.ComputeGeometricStatistics( Cork::Statistics::GeometricProperties::GEOM_ALL );
    Cork::Meshes::TopologicalStatisticsResult topo_stats = mesh.ComputeTopologicalStatistics( Cork::Statistics::TopologicalProperties::TOPO_BASE );

    geotopoResults << filename << "\t" << topo_stats.return_value().is_two_manifold() << "\t";
    geotopoResults << stats.num_vertices() << "\t" << topo_stats.return_value().num_edges() << "\t" << stats.num_triangles() << "\t";
    geotopoResults << stats.area() << "\t" << stats.volume() << "\t";
    //	geotopoResults << stats.maxEdgeLength() << "\t" << stats.minEdgeLength() << "\t";
    geotopoResults << stats.bounding_box().minima().x() << "\t" << stats.bounding_box().minima().y() << "\t"
                   << stats.bounding_box().minima().z() << "\t";
    geotopoResults << stats.bounding_box().maxima().x() << "\t" << stats.bounding_box().maxima().y() << "\t"
                   << stats.bounding_box().maxima().z() << std::endl;

    num_successful_operations++;

    if (topo_stats.return_value().is_two_manifold())
    {
        num_two_manifold_results++;
    }

    total_num_vertices += stats.num_vertices();
    total_num_edges += topo_stats.return_value().num_edges();
    total_num_triangles += stats.num_triangles();
}

//  NOLINTNEXTLINE
int main(int argc, char* argv[])
{
    Cork::CorkService     cork_service;

    boost::program_options::options_description desc("Allowed options");

    desc.add_options()
        ("help", "produce help message")
        ("union", "Compute Boolean Union")
        ("intersection", "Compute Boolean Intersection")
        ("difference", "Compute Boolean Difference")
        ("xor", "Compute Boolean Exclusive Or")
        ("input-directory", boost::program_options::value<std::string>(), "Directory containing input meshes")
        ("output-directory", boost::program_options::value<std::string>(), "Directory to receive output files")
        ("write-results", "Write result meshes")("write-statistics", "Write Statistics");

    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (vm.count("help") > 0 )
    {
        std::cout << desc << "\n";
        return 1;
    }

    if (vm.count("input-directory") > 0 )
    {
        std::cout << "Input Directory: " << vm["input-directory"].as<std::string>() << std::endl;
    }

    if (vm.count("output-directory") > 0)
    {
        std::cout << "Output Directory: " << vm["output-directory"].as<std::string>() << std::endl;
    }

    bool compute_union = vm.count("union") > 0;
    bool compute_difference = vm.count("difference") > 0;
    bool compute_intersection = vm.count("intersection") > 0;
    bool compute_XOR = vm.count("xor") > 0;

    bool write_results = vm.count("write-results") > 0;
    bool write_stats = vm.count("write-statistics") > 0;

    Cork::SolverControlBlock control_block = Cork::CorkService::get_default_control_block();

    control_block.set_use_multiple_threads(true);

    std::filesystem::directory_iterator model_repository(
        std::filesystem::path(vm["input-directory"].as<std::string>()));
    std::filesystem::directory_entry results_directory(vm["output-directory"].as<std::string>());

    //	Collect a list of all the model files available

    std::list<std::filesystem::path> model_files;

    for ( const std::filesystem::directory_entry& model_file : model_repository)
    {
        if (model_file.path().extension() == ".off")
        {
            model_files.push_back(model_file.path());
        }
    }

    //	Open a detailed timing results file

    std::ofstream timing_results;
    std::ofstream cumulative_timing_results;
    std::ofstream geotopo_results;

    std::filesystem::path timing_results_file_path(results_directory);
    std::filesystem::path cumulative_timing_results_file_path(results_directory);
    std::filesystem::path geotopo_results_file_path(results_directory);

    if (write_stats)
    {
        timing_results_file_path += "/stats/timing_results.txt";

        timing_results.open(timing_results_file_path);

        timing_results << "Filename"
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

        timing_results << std::fixed << std::setprecision(4);

        //	Open a cumulative timing results file

        cumulative_timing_results_file_path += "/stats/cumulative_timing_results.txt";

        cumulative_timing_results.open(cumulative_timing_results_file_path);

        //	Open a file for collecting statistics on the results

        geotopo_results_file_path += "/stats/geotopo_results.txt";

        geotopo_results.open(geotopo_results_file_path);

        geotopo_results << std::fixed << std::setprecision(2);

        geotopo_results << "Name\tIs2Man?\t#Verts\t#Edges\t#Faces\tArea\tVolume\tBBox(min,max)" << std::endl;
    }

    //	Accumulator for all boolean operation elapsed time

    boost::timer::nanosecond_type grand_total_CPU_time = 0;
    boost::timer::nanosecond_type grand_total_wall_time = 0;

    boost::timer::nanosecond_type cumulative_CPU_time = 0;
    boost::timer::nanosecond_type cumulative_wall_time = 0;

    //	Create Unions, Intersections, Differences and XORs of all the files taken two at a time

    typedef std::pair<std::filesystem::path, std::shared_ptr<Cork::TriangleMesh>> NameAndModel;

    std::list<NameAndModel> models;

    for ( const std::filesystem::path& current_model : model_files)
    {
        Cork::Files::ReadFileResult read_model_result = Cork::Files::readOFF(current_model);

        if (!read_model_result.succeeded())
        {
            std::cout << "Error Reading Model: " << current_model.filename().string() << "    " << read_model_result.message() << std::endl;
            exit(-1);
        }

        std::cout << "Read: " << current_model.filename().string() << std::endl;

        Cork::Meshes::TopologicalStatisticsResult topo_stats = read_model_result.return_ptr()->ComputeTopologicalStatistics( Cork::Statistics::TopologicalProperties::TOPO_ALL );

        if( topo_stats.succeeded() )
        {
            std::cout << "Num holes: " << topo_stats.return_value().holes().size() << "    Num Self Intersections: " << topo_stats.return_value().self_intersecting_edges().size() << std::endl;
        }
        else
        {
            std::cout << "Unable to compute Topo Stats for Mesh" << std::endl;
        }

        models.emplace_back(
            std::make_pair(current_model, std::shared_ptr<Cork::TriangleMesh>(read_model_result.return_ptr().release())));
    }

    for (const NameAndModel& first_model : models)
    {
        for (const NameAndModel& second_model : models)
        {
            std::string first_name( first_model.first.stem() );

            if (first_name.find('_') != std::string::npos)
            {
                first_name = first_name.substr(0, first_name.find('_') - 1);
            }

            std::string second_name( second_model.first.stem() );

            if (second_name.find('_') != std::string::npos)
            {
                second_name = second_name.substr(0, second_name.find('_') - 1);
            }

            if (first_name == second_name)
            {
                continue;
            }

            std::cout << first_model.first.filename() << "    " << second_model.first.filename() << std::endl;

            std::unique_ptr<Cork::SolidObjectMesh> first_mesh = Cork::CorkService::from_triangle_mesh(*first_model.second);
            std::unique_ptr<Cork::SolidObjectMesh> second_mesh = Cork::CorkService::from_triangle_mesh(*second_model.second);

            if (compute_union)
            {
                Cork::SolidObjectMesh::BooleanOperationResult boolean_op_result = first_mesh->Union(*second_mesh, control_block);

                std::string filename = first_model.first.filename().stem().string() + "_" +
                                       second_model.first.filename().stem().string() + "_union";

                if (!boolean_op_result.succeeded())
                {
                    std::cout << "Union Failed: " << boolean_op_result.message() << " : " << boolean_op_result.inner_error()->message() << std::endl;
                    geotopo_results << filename << "    Failed" << std::endl;
                    timing_results << filename << "    Failed" << std::endl;

                    num_failed_operations++;
                }
                else
                {
                    std::unique_ptr<Cork::SolidObjectMesh> unioned_mesh(boolean_op_result.return_ptr().release());

                    //					std::cout << "Components in finished Mesh: " << unionedMesh->CountComponents()
                    //<< std::endl;

                    cumulative_CPU_time += unioned_mesh->GetPerformanceStats().elapsed_cpu_time_in_nanoseconds();
                    cumulative_wall_time += unioned_mesh->GetPerformanceStats().elapsed_wall_time_in_nanoseconds();

                    std::unique_ptr<Cork::TriangleMesh> unioned_triangle_mesh(unioned_mesh->ToTriangleMesh());

                    std::filesystem::path result_file_path(results_directory);

                    result_file_path += "/";
                    result_file_path += filename;
                    result_file_path += ".off";

                    if (write_results)
                    {
                        Cork::Files::writeOFF(result_file_path, *unioned_triangle_mesh);
                    }

                    if (write_stats)
                    {
                        timing_results << filename << "\t"
                                      << static_cast<double>(unioned_mesh->GetPerformanceStats().elapsed_cpu_time_in_nanoseconds()) / NUM_NANOSECONDS_PER_SECOND
                                      << "\t"
                                      << static_cast<double>(unioned_mesh->GetPerformanceStats().elapsed_wall_time_in_nanoseconds()) / NUM_NANOSECONDS_PER_SECOND
                                      << "\t" << unioned_mesh->GetPerformanceStats().number_of_triangles_in_disjoint_union()
                                      << "\t" << unioned_mesh->GetPerformanceStats().number_of_triangles_in_final_mesh()
                                      << "\t" << unioned_mesh->GetPerformanceStats().starting_virtual_memory_size_in_MB()
                                      << "\t" << unioned_mesh->GetPerformanceStats().ending_virtual_memory_size_in_MB()
                                      << std::endl;

                        WriteMeshStatistics(*unioned_triangle_mesh, filename, geotopo_results);
                    }
                }
            }

            if (compute_difference)
            {
                Cork::SolidObjectMesh::BooleanOperationResult boolean_op_result =
                    first_mesh->Difference(*second_mesh, control_block);

                std::string filename = first_model.first.filename().stem().string() + "_" +
                                       second_model.first.filename().stem().string() + "_difference";

                if (!boolean_op_result.succeeded())
                {
                    std::cout << "Difference Failed: " << boolean_op_result.message() << std::endl;
                    geotopo_results << filename << "    Failed" << std::endl;
                    timing_results << filename << "    Failed" << std::endl;

                    num_failed_operations++;
                }
                else
                {
                    std::unique_ptr<Cork::SolidObjectMesh> difference_mesh(boolean_op_result.return_ptr().release());

                    cumulative_CPU_time += difference_mesh->GetPerformanceStats().elapsed_cpu_time_in_nanoseconds();
                    cumulative_wall_time += difference_mesh->GetPerformanceStats().elapsed_wall_time_in_nanoseconds();

                    std::unique_ptr<Cork::TriangleMesh> difference_triangle_mesh(difference_mesh->ToTriangleMesh());

                    std::filesystem::path result_file_path(results_directory);

                    result_file_path += "/";
                    result_file_path += filename;
                    result_file_path += ".off";

                    if (write_results)
                    {
                        Cork::Files::writeOFF(result_file_path, *difference_triangle_mesh);
                    }

                    if (write_stats)
                    {
                        timing_results << filename << "\t"
                                      << static_cast<double>(difference_mesh->GetPerformanceStats().elapsed_cpu_time_in_nanoseconds()) / NUM_NANOSECONDS_PER_SECOND
                                      << "\t"
                                      << static_cast<double>(difference_mesh->GetPerformanceStats().elapsed_wall_time_in_nanoseconds()) / NUM_NANOSECONDS_PER_SECOND
                                      << "\t"
                                      << difference_mesh->GetPerformanceStats().number_of_triangles_in_disjoint_union()
                                      << "\t" << difference_mesh->GetPerformanceStats().number_of_triangles_in_final_mesh()
                                      << "\t" << difference_mesh->GetPerformanceStats().starting_virtual_memory_size_in_MB()
                                      << "\t" << difference_mesh->GetPerformanceStats().ending_virtual_memory_size_in_MB()
                                      << std::endl;

                        WriteMeshStatistics(*difference_triangle_mesh, filename, geotopo_results);
                    }
                }
            }

            if (compute_intersection)
            {
                Cork::SolidObjectMesh::BooleanOperationResult boolean_op_result =
                    first_mesh->Intersection(*second_mesh, control_block);

                std::string filename = first_model.first.filename().stem().string() + "_" +
                                       second_model.first.filename().stem().string() + "_intersection";

                if (!boolean_op_result.succeeded())
                {
                    std::cout << "Intersection Failed: " << boolean_op_result.message() << std::endl;
                    geotopo_results << filename << "    Failed" << std::endl;
                    timing_results << filename << "    Failed" << std::endl;

                    num_failed_operations++;
                }
                else
                {
                    std::unique_ptr<Cork::SolidObjectMesh> intersection_mesh(boolean_op_result.return_ptr().release());

                    //					std::cout << "Components in finished Mesh: " <<
                    //intersectionMesh->CountComponents()
                    //<< std::endl;

                    cumulative_CPU_time += intersection_mesh->GetPerformanceStats().elapsed_cpu_time_in_nanoseconds();
                    cumulative_wall_time += intersection_mesh->GetPerformanceStats().elapsed_wall_time_in_nanoseconds();

                    std::unique_ptr<Cork::TriangleMesh> intersection_triangle_mesh(intersection_mesh->ToTriangleMesh());

                    std::filesystem::path result_file_path(results_directory);

                    result_file_path += "/";
                    result_file_path += filename;
                    result_file_path += ".off";

                    if (write_results)
                    {
                        Cork::Files::writeOFF(result_file_path, *intersection_triangle_mesh);
                    }

                    if (write_stats)
                    {
                        timing_results << filename << "\t"
                                      << static_cast<double>(intersection_mesh->GetPerformanceStats().elapsed_cpu_time_in_nanoseconds()) / NUM_NANOSECONDS_PER_SECOND
                                      << "\t"
                                      << static_cast<double>(intersection_mesh->GetPerformanceStats().elapsed_wall_time_in_nanoseconds()) / NUM_NANOSECONDS_PER_SECOND
                                      << "\t"
                                      << intersection_mesh->GetPerformanceStats().number_of_triangles_in_disjoint_union()
                                      << "\t" << intersection_mesh->GetPerformanceStats().number_of_triangles_in_final_mesh()
                                      << "\t" << intersection_mesh->GetPerformanceStats().starting_virtual_memory_size_in_MB()
                                      << "\t" << intersection_mesh->GetPerformanceStats().ending_virtual_memory_size_in_MB()
                                      << std::endl;

                        WriteMeshStatistics(*intersection_triangle_mesh, filename, geotopo_results);
                    }
                }
            }

            if (compute_XOR)
            {
                Cork::SolidObjectMesh::BooleanOperationResult boolean_op_result =
                    first_mesh->SymmetricDifference(*second_mesh, control_block);

                std::string filename = first_model.first.filename().stem().string() + "_" +
                                       second_model.first.filename().stem().string() + "_xor";

                if (!boolean_op_result.succeeded())
                {
                    std::cout << "Symmetric Difference Failed: " << boolean_op_result.message() << std::endl;
                    geotopo_results << filename << "    Failed" << std::endl;
                    timing_results << filename << "    Failed" << std::endl;

                    num_failed_operations++;
                }
                else
                {
                    std::unique_ptr<Cork::SolidObjectMesh> XOR_mesh(boolean_op_result.return_ptr().release());

                    cumulative_CPU_time += XOR_mesh->GetPerformanceStats().elapsed_cpu_time_in_nanoseconds();
                    cumulative_wall_time += XOR_mesh->GetPerformanceStats().elapsed_wall_time_in_nanoseconds();

                    std::unique_ptr<Cork::TriangleMesh> XOR_triangle_mesh(XOR_mesh->ToTriangleMesh());

                    std::filesystem::path result_file_path(results_directory);

                    result_file_path += "/";
                    result_file_path += filename;
                    result_file_path += ".off";

                    if (write_results)
                    {
                        Cork::Files::writeOFF(result_file_path, *XOR_triangle_mesh);
                    }

                    if (write_stats)
                    {
                        timing_results << filename << "\t"
                                      << static_cast<double>(XOR_mesh->GetPerformanceStats().elapsed_cpu_time_in_nanoseconds()) / NUM_NANOSECONDS_PER_SECOND << "\t"
                                      << static_cast<double>(XOR_mesh->GetPerformanceStats().elapsed_wall_time_in_nanoseconds()) / NUM_NANOSECONDS_PER_SECOND << "\t"
                                      << XOR_mesh->GetPerformanceStats().number_of_triangles_in_disjoint_union() << "\t"
                                      << XOR_mesh->GetPerformanceStats().number_of_triangles_in_final_mesh() << "\t"
                                      << XOR_mesh->GetPerformanceStats().starting_virtual_memory_size_in_MB() << "\t"
                                      << XOR_mesh->GetPerformanceStats().ending_virtual_memory_size_in_MB() << std::endl;

                        WriteMeshStatistics(*XOR_triangle_mesh, filename, geotopo_results);
                    }
                }
            }
        }

        if (write_stats)
        {
            cumulative_timing_results << first_model.first.filename() << "\t" << cumulative_CPU_time / NUM_NANOSECONDS_PER_SECOND << "\t"
                                    << cumulative_wall_time / NUM_NANOSECONDS_PER_SECOND << std::endl;

            grand_total_CPU_time += cumulative_CPU_time;
            grand_total_wall_time += cumulative_wall_time;
        }
    }

    if (write_stats)
    {
        std::time_t current_date_time;          //  NOLINT

        time(&current_date_time);

        timing_results << std::endl << std::endl << asctime(localtime(&current_date_time)) << std::endl;

        cumulative_timing_results << std::endl
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

        cumulative_timing_results << num_successful_operations << "\t" << num_failed_operations << "\t"
                                << num_two_manifold_results << "\t" << total_num_vertices << "\t" << total_num_edges
                                << "\t" << total_num_triangles << std::endl;

        cumulative_timing_results << std::endl
                                << "Total CPU Time:\t" << grand_total_CPU_time / NUM_NANOSECONDS_PER_SECOND << std::endl
                                << "Total Wall Time:\t" << grand_total_wall_time / NUM_NANOSECONDS_PER_SECOND << std::endl;

        cumulative_timing_results << std::endl << std::endl << asctime(localtime(&current_date_time)) << std::endl;

        geotopo_results << std::endl
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

        geotopo_results << num_successful_operations << "\t" << num_failed_operations << "\t" << num_two_manifold_results
                       << "\t" << total_num_vertices << "\t" << total_num_edges << "\t" << total_num_triangles
                       << std::endl;

        geotopo_results << std::endl << std::endl << asctime(localtime(&current_date_time)) << std::endl;
    }

    cumulative_timing_results.flush();
    cumulative_timing_results.close();

    geotopo_results.flush();
    geotopo_results.close();

    timing_results.flush();
    timing_results.close();

    return (0);
}
