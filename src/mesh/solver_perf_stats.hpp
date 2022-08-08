// Copyright (c) 2021 Stephan Friedl
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files (the "Software"), to deal in
// the Software without restriction, including without limitation the rights to
// use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
// the Software, and to permit persons to whom the Software is furnished to do so,
// subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
// IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
// CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#pragma once

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

        uint64_t number_of_triangles_in_disjoint_union() const { return number_of_triangles_in_disjoint_union_; }

        uint64_t number_of_triangles_in_final_mesh() const { return number_of_triangles_in_final_mesh_; }

        boost::timer::nanosecond_type elapsed_cpu_time_in_nanoseconds() const { return elapsed_cpu_time_in_nanoseconds_; }

        boost::timer::nanosecond_type elapsed_wall_time_in_nanoseconds() const { return elapsed_wall_time_in_nanoseconds_; }

        uint64_t starting_virtual_memory_size_in_MB() const { return starting_virtual_memory_size_in_MB_; }

        uint64_t ending_virtual_memory_size_in_MB() const { return ending_virtual_memory_size_in_MB_; }

        void set_number_of_triangles_in_disjoint_union(uint64_t number_of_triangles_in_disjoint_union)
        {
            number_of_triangles_in_disjoint_union_ = number_of_triangles_in_disjoint_union;
        }

        void set_number_of_triangles_in_final_mesh(uint64_t number_of_triangles_in_final_mesh)
        {
            number_of_triangles_in_final_mesh_ = number_of_triangles_in_final_mesh;
        }

        void set_elapsed_cpu_time_in_nano_seconds(boost::timer::nanosecond_type elapsed_cpu_time_in_nano_seconds)
        {
            elapsed_cpu_time_in_nanoseconds_ = elapsed_cpu_time_in_nano_seconds;
        }

        void set_elapsed_wall_time_in_nano_seconds(boost::timer::nanosecond_type elapsed_wall_time_in_nano_seconds)
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

        boost::timer::nanosecond_type elapsed_cpu_time_in_nanoseconds_;
        boost::timer::nanosecond_type elapsed_wall_time_in_nanoseconds_;

        uint64_t starting_virtual_memory_size_in_MB_;
        uint64_t ending_virtual_memory_size_in_MB_;
    };
}  // namespace Cork::Meshes
