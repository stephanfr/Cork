/*
 Copyright (c) 2021 Stephan Friedl

 Permission is hereby granted, free of charge, to any person obtaining a copy of
 this software and associated documentation files (the "Software"), to deal in
 the Software without restriction, including without limitation the rights to
 use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 the Software, and to permit persons to whom the Software is furnished to do so,
 subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <catch2/catch_all.hpp>
#include <list>
#include <iostream>
#include <fstream>

#include "file_formats/files.hpp"
#include "intersection/triangulator.hpp"
#include "mesh/boundary_edge_builder.hpp"
#include "mesh/topo_cache.hpp"
#include "math/quantization.hpp"
#include "mesh/mesh.hpp"
#include "cork.hpp"

//  The pragma below is to disable to false errors flagged by intellisense for Catch2 REQUIRE macros.

#if __INTELLISENSE__
#pragma diag_suppress 2486
#endif

TEST_CASE("Topo Cache Tests", "[core]")
{
    SECTION("Simple Test")
    {
        //  This test blindly deletes a bunch of triangles and as a result a bunch of vertices
        //      and edges are deleted as a consequence.

        auto read_result = Cork::Files::readOFF("../../UnitTest/Test Files/Quadrilateral1.off");

        REQUIRE(read_result.succeeded());

        auto* triangle_mesh(read_result.return_ptr().release());

        REQUIRE(triangle_mesh->num_vertices() == 26);
        REQUIRE(triangle_mesh->num_triangles() == 48);

        std::unique_ptr<Cork::Meshes::Mesh> mesh( new Cork::Meshes::Mesh(*triangle_mesh,Cork::SolverControlBlock::get_default_control_block()));

        Cork::Meshes::MeshTopoCache& topo_cache = mesh->topo_cache();

        REQUIRE( topo_cache.triangles().size() == 48 );
        REQUIRE( topo_cache.vertices().size() == 26 );
        REQUIRE( topo_cache.edges().size() == 72 );

        topo_cache.delete_tri( &(*(mesh->topo_cache().triangles().begin())) );
        topo_cache.delete_tri( &(*(mesh->topo_cache().triangles().begin())) );
        topo_cache.delete_tri( &(*(mesh->topo_cache().triangles().begin())) );
        topo_cache.delete_tri( &(*(mesh->topo_cache().triangles().begin())) );
        topo_cache.delete_tri( &(*(mesh->topo_cache().triangles().begin())) );
        topo_cache.delete_tri( &(*(mesh->topo_cache().triangles().begin())) );
        topo_cache.delete_tri( &(*(mesh->topo_cache().triangles().begin())) );
        topo_cache.delete_tri( &(*(mesh->topo_cache().triangles().begin())) );
        topo_cache.delete_tri( &(*(mesh->topo_cache().triangles().begin())) );
        topo_cache.delete_tri( &(*(mesh->topo_cache().triangles().begin())) );
        topo_cache.delete_tri( &(*(mesh->topo_cache().triangles().begin())) );
        topo_cache.delete_tri( &(*(mesh->topo_cache().triangles().begin())) );
        topo_cache.delete_tri( &(*(mesh->topo_cache().triangles().begin())) );
        topo_cache.delete_tri( &(*(mesh->topo_cache().triangles().begin())) );
        topo_cache.delete_tri( &(*(mesh->topo_cache().triangles().begin())) );
        topo_cache.delete_tri( &(*(mesh->topo_cache().triangles().begin())) );
        topo_cache.delete_tri( &(*(mesh->topo_cache().triangles().begin())) );
        topo_cache.delete_tri( &(*(mesh->topo_cache().triangles().begin())) );
        topo_cache.delete_tri( &(*(mesh->topo_cache().triangles().begin())) );
        topo_cache.delete_tri( &(*(mesh->topo_cache().triangles().begin())) );
        topo_cache.delete_tri( &(*(mesh->topo_cache().triangles().begin())) );
        topo_cache.delete_tri( &(*(mesh->topo_cache().triangles().begin())) );
        topo_cache.delete_tri( &(*(mesh->topo_cache().triangles().begin())) );
        topo_cache.delete_tri( &(*(mesh->topo_cache().triangles().begin())) );
        topo_cache.delete_tri( &(*(mesh->topo_cache().triangles().begin())) );
        topo_cache.delete_tri( &(*(mesh->topo_cache().triangles().begin())) );
        topo_cache.delete_tri( &(*(mesh->topo_cache().triangles().begin())) );
        topo_cache.delete_tri( &(*(mesh->topo_cache().triangles().begin())) );
        topo_cache.delete_tri( &(*(mesh->topo_cache().triangles().begin())) );
        topo_cache.delete_tri( &(*(mesh->topo_cache().triangles().begin())) );

        REQUIRE( topo_cache.triangles().size() == 18 );
        REQUIRE( topo_cache.vertices().size() == 18 );
        REQUIRE( topo_cache.edges().size() == 36 );
    }

    SECTION("Surgical Triangle Removal Test")
    {
        //  This test removes all the triangles attached to one vertex, which results in that
        //      vertex and all the edges connecting it to be removed.  If there is an accounting or
        //      self-consistency issue in the cache, this test should fail as a result.

        auto read_result = Cork::Files::readOFF("../../UnitTest/Test Files/Quadrilateral1.off");

        REQUIRE(read_result.succeeded());

        auto* triangle_mesh(read_result.return_ptr().release());

        REQUIRE(triangle_mesh->num_vertices() == 26);
        REQUIRE(triangle_mesh->num_triangles() == 48);

        std::unique_ptr<Cork::Meshes::Mesh> mesh( new Cork::Meshes::Mesh(*triangle_mesh,Cork::SolverControlBlock::get_default_control_block()));

        Cork::Meshes::MeshTopoCache& topo_cache = mesh->topo_cache();

        REQUIRE( topo_cache.triangles().size() == 48 );
        REQUIRE( topo_cache.vertices().size() == 26 );
        REQUIRE( topo_cache.edges().size() == 72 );

        auto vert_to_be_deleted = topo_cache.vertices().begin();
        vert_to_be_deleted++;
        vert_to_be_deleted++;
        vert_to_be_deleted++;
        vert_to_be_deleted++;

        size_t  edges_that_will_be_deleted = vert_to_be_deleted->edges().size();

        std::vector<const Cork::Meshes::TopoTri*>   tris_to_delete;
        
        for( auto tri_to_delete : vert_to_be_deleted->triangles() )
        {
            tris_to_delete.push_back( tri_to_delete );
        }

        for( auto tri_to_delete : tris_to_delete )
        {
            topo_cache.delete_tri( const_cast<Cork::Meshes::TopoTri*>(tri_to_delete) );     //  NOLINT(cppcoreguidelines-pro-type-const-cast)
        }

        REQUIRE( topo_cache.triangles().size() == 48 - tris_to_delete.size() );
        REQUIRE( topo_cache.vertices().size() == 25 );
        REQUIRE( topo_cache.edges().size() == 72 - edges_that_will_be_deleted );
    }
}
