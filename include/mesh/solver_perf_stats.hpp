
#include "cork.hpp"

namespace Cork::Meshes
{
    class SolverPerfStats : public SolverPerformanceStatistics
    {
       public:
        SolverPerfStats()
            : number_of_triangles_in_disjoint_union_(0),
              number_of_triangles_in_final_mesh_(0),
              elapsed_cpu_time_in_nanoseconds_(0),
              elapsed_wall_time_in_nanoseconds_(0),
              starting_virtual_memory_size_in_MB_(0),
              ending_virtual_memory_size_in_MB_(0)
        {
        }

        uint64_t number_of_triangles_in_disjoint_union() const { return (number_of_triangles_in_disjoint_union_); }

        uint64_t number_of_triangles_in_final_mesh() const { return (number_of_triangles_in_final_mesh_); }

        uint64_t elapsed_cpu_time_in_nanoseconds() const { return (elapsed_cpu_time_in_nanoseconds_); }

        uint64_t elapsed_wall_time_in_nanoseconds() const { return (elapsed_wall_time_in_nanoseconds_); }

        uint64_t starting_virtual_memory_size_in_MB() const { return (starting_virtual_memory_size_in_MB_); }

        uint64_t ending_virtual_memory_size_in_MB() const { return (ending_virtual_memory_size_in_MB_); }

        void set_number_of_triangles_in_disjoint_union(uint64_t number_of_triangles_in_disjoint_union)
        {
            number_of_triangles_in_disjoint_union_ = number_of_triangles_in_disjoint_union;
        }

        void set_number_of_triangles_in_final_mesh(uint64_t number_of_triangles_in_final_mesh)
        {
            number_of_triangles_in_final_mesh_ = number_of_triangles_in_final_mesh;
        }

        void set_elapsed_cpu_time_in_nano_seconds(uint64_t elapsed_cpu_time_in_nano_seconds)
        {
            elapsed_cpu_time_in_nanoseconds_ = elapsed_cpu_time_in_nano_seconds;
        }

        void set_elapsed_wall_time_in_nano_seconds(uint64_t elapsed_wall_time_in_nano_seconds)
        {
            elapsed_wall_time_in_nanoseconds_ = elapsed_wall_time_in_nano_seconds;
        }

        void set_starting_virtual_memory_size_in_MB(uint64_t starting_virtual_memory_size_in_MB)
        {
            starting_virtual_memory_size_in_MB_ = starting_virtual_memory_size_in_MB;
        }

        void set_ending_virtual_memory_size_in_MB(uint64_t ending_virtual_memory_size_in_MB)
        {
            ending_virtual_memory_size_in_MB_ = ending_virtual_memory_size_in_MB;
        }

       private:
        uint64_t number_of_triangles_in_disjoint_union_;
        uint64_t number_of_triangles_in_final_mesh_;

        uint64_t elapsed_cpu_time_in_nanoseconds_;
        uint64_t elapsed_wall_time_in_nanoseconds_;

        uint64_t starting_virtual_memory_size_in_MB_;
        uint64_t ending_virtual_memory_size_in_MB_;
    };
}  // namespace Cork::Meshes
