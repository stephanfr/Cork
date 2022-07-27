// +-------------------------------------------------------------------------
// | topo_cache.cpp
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

#include "topo_cache.hpp"

#include <tbb/spin_mutex.h>
#include <tbb/task_group.h>

#include "math/gmpext4.hpp"
#include "mesh_base.hpp"

namespace Cork::Meshes
{

    //
    //  The following looks strange but I was getting potential leak warnings from Valigrind I think b/c the
    //      default allocator for the small_vector class was not releasing memory on exit.  Thus the static
    //      manager class below.
    //
    //  This cleaned up the complaints from Valgrind.  I think that the TopoTri* and TopoEdge* pools decay to
    //      the same pool, but I purge both just in case anything changes in the future.
    //

    class SingletonPoolManager
    {
        public :
        SingletonPoolManager() = default;

        ~SingletonPoolManager()
        {
            std::cout << "TopoTri pool purged with result: " << boost::singleton_pool<boost::pool_allocator_tag, sizeof(const TopoTri*)>::purge_memory() << std::endl;
            std::cout << "TopoEdge pool purged with result: " << boost::singleton_pool<boost::pool_allocator_tag, sizeof(const TopoEdge*)>::purge_memory() << std::endl;
        }
    };

    static SingletonPoolManager g_singletonPoolManager;


    TriangleUID TopoEdgeBoundary::triangle_on_boundary_uid() const
    {
        return edges_.front()->triangles().front()->source_triangle_uid();
    }

    void TopoEdgeBoundary::smooth()
    {
        //  Pass through the boundary, looking for any pair of edges on the boundary that belong to
        //      the same triangle.  We should be able to remove the two edges and replace them
        //      with the single far edge of the triangle - making the perimeter shorter and the area larger.

        bool replaced_edge = false;

        do
        {
            replaced_edge = false;

            for (uint32_t i = 0; i < edges_.size() - 1; i++)
            {
                for (uint32_t j = i + 1; j < edges_.size(); j++)
                {
                    for (auto current_tri1 : edges_[i]->triangles())
                    {
                        for (auto current_tri2 : edges_[j]->triangles())
                        {
                            if (current_tri1 == current_tri2)
                            {
                                for (auto edge_to_test : current_tri1->edges())
                                {
                                    if (edge_to_test != edges_[i] && edge_to_test != edges_[j])
                                    {
                                        edges_[i] = edge_to_test;
                                        edges_.erase(edges_.begin() + j);
                                        replaced_edge = true;
                                        break;
                                    }
                                }
                            }

                            if (replaced_edge)
                            {
                                break;
                            }
                        }

                        if (replaced_edge)
                        {
                            break;
                        }
                    }

                    if (replaced_edge)
                    {
                        break;
                    }
                }

                if (replaced_edge)
                {
                    break;
                }
            }

        } while (replaced_edge);

        //  TODO check last edge with first....
    }

    TriangleByIndicesVectorTopoCache::TriangleByIndicesVectorTopoCache(TriangleByIndicesVector& triangles,
                                                                       Vertex3DVector& vertices, uint32_t num_edges,
                                                                       const Math::Quantizer& quantizer)
        : TopoCacheBase(triangles, vertices, num_edges, quantizer)
    {
    }

    TriangleByIndicesVectorTopoCache::~TriangleByIndicesVectorTopoCache() {}

    MeshTopoCache::MeshTopoCache(MeshBase& owner, const Math::Quantizer& quantizer)
        : TriangleByIndicesVectorTopoCache(owner.triangles(), owner.vertices(), owner.triangles().size() * 3,
                                           quantizer),
          mesh_(owner)
    {
    }

    MeshTopoCache::~MeshTopoCache() {}

    void MeshTopoCache::commit()
    {
        // record which vertices are live

        BooleanVector<VertexIndex> live_verts(mesh_vertices_.size(), false);  //	All initialized to zero

        for (auto& vert : topo_vertex_list_)
        {
            live_verts[vert.index()] = true;
        }

        // record which triangles are live, and record connectivity

        BooleanVector<TriangleByIndicesIndex> live_tris(mesh_triangles_.size());

        for (auto& tri : topo_tri_list_)
        {
            live_tris[tri.ref()] = true;

            for (size_t k = 0; k < 3; k++)
            {
                mesh_triangles_[tri.ref()][k] = tri.verts()[k]->index();
            }
        }

        // compact the vertices and build a remapping function

        class VertexMap : public std::vector<VertexIndex>
        {
            public :

            VertexIndex& operator[]( VertexIndex vi )
            {
                return std::vector<VertexIndex>::operator[]( static_cast<size_t>(vi) );
            }
        };
        
        VertexMap vertex_map;

        vertex_map.reserve(mesh_vertices_.size());

        VertexIndex vert_write(0u);

        for (VertexIndex read{0}; read < mesh_vertices_.size(); read++)
        {
            if (live_verts[read])
            {
                vertex_map.emplace_back(vert_write);
                mesh_vertices_[vert_write] = mesh_vertices_[VertexIndex(read)];
                vert_write++;
            }
            else
            {
                vertex_map.emplace_back(Primitives::UNINITIALIZED_INDEX);
            }
        }

        mesh_vertices_.resize(static_cast<size_t>(vert_write));

        // rewrite the vertex reference ids

        for (auto& vert : topo_vertex_list_)
        {
            vert.set_index(vertex_map[vert.index()]);
        }
        
        TriangleByIndicesIndexVector    tmap;

        tmap.reserve(mesh_triangles_.size());

        TriangleByIndicesIndex tri_write{0UL};

        for (TriangleByIndicesIndex read{0UL}; read < mesh_triangles_.size(); read++)
        {
            if (live_tris[read])
            {
                tmap.emplace_back(tri_write);
                mesh_triangles_[tri_write] = mesh_triangles_[read];

                for (uint k = 0; k < 3; k++)
                {
                    mesh_triangles_[tri_write][k] =
                        vertex_map[mesh_triangles_[tri_write][k]];
                }

                tri_write++;
            }
            else
            {
                tmap.emplace_back(Primitives::UNINITIALIZED_INDEX);
            }
        }

        mesh_triangles_.resize(static_cast<size_t>(tri_write));

        // rewrite the triangle reference ids

        for (auto& tri : triangles())
        {
            tri.set_ref(tmap[tri.ref()]);
        }
    }

    TopoEdgeBoundary MeshTopoCache::topo_edge_boundary(const BoundaryEdge& boundary) const
    {
        std::vector<const TopoEdge*> topo_edge_boundary;

        topo_edge_boundary.reserve(boundary.vertex_indices().size() + 4);

        for (int i = 0; i < boundary.vertex_indices().size() - 1; i++)
        {
            VertexIndex vertex1 = boundary.vertex_indices()[i];
            VertexIndex vertex2 = boundary.vertex_indices()[i + 1];

            for (auto edge : vertices().getPool()[vertex1].edges())
            {
                if (((edge->vert_0().index() == vertex1) && (edge->vert_1().index() == vertex2)) ||
                    ((edge->vert_0().index() == vertex2) && (edge->vert_1().index() == vertex1)))
                {
                    topo_edge_boundary.emplace_back(edge);

                    break;
                }
            }
        }

        VertexIndex vertex1 = boundary.vertex_indices()[0];
        VertexIndex vertex2 = boundary.vertex_indices()[boundary.vertex_indices().size() - 1];

        for (auto edge : vertices().getPool()[vertex1].edges())
        {
            if (((edge->vert_0().index() == vertex1) && (edge->vert_1().index() == vertex2)) ||
                ((edge->vert_0().index() == vertex2) && (edge->vert_1().index() == vertex1)))
            {
                topo_edge_boundary.emplace_back(edge);

                break;
            }
        }

        return TopoEdgeBoundary(std::move(topo_edge_boundary));
    }

    std::set<const TopoTri*> MeshTopoCache::tris_along_edges(const TopoEdgeBoundary& boundary) const
    {
        std::set<const TopoVert*> edge_verts;
        std::set<const TopoTri*> edge_tris;
        std::set<const TopoEdge*> all_edges_for_edge_tris;

        //  First, add all triangles that share an edge.

        for (auto edge : boundary.edges())
        {
            //  Insert both edge vertices as we are not assured of ordering
            //      (i.e. we could have v1->v2, v2->v3, v4->v3 so simply adding the first vert would miss v3)

            edge_verts.insert(&edge->vert_0());
            edge_verts.insert(&edge->vert_1());

            for (auto tri : edge->triangles())
            {
                edge_tris.insert(tri);

                all_edges_for_edge_tris.insert(tri->edges()[0]);
                all_edges_for_edge_tris.insert(tri->edges()[1]);
                all_edges_for_edge_tris.insert(tri->edges()[2]);
            }
        }

        //  Next, add any triangles which have at least one vertex on the boundary and share at least 1 edge with an
        //  adjacent
        //      triangle on the edge.
        //
        //  Repeat until we add no more new triangles.

        bool repeat_search = false;

        do
        {
            repeat_search = false;

            for (auto vert : edge_verts)
            {
                for (auto tri : vert->triangles())
                {
                    if (edge_tris.contains(tri))
                    {
                        continue;
                    }

                    bool vert0_on = edge_verts.contains(tri->verts()[0]);
                    bool vert1_on = edge_verts.contains(tri->verts()[1]);
                    bool vert2_on = edge_verts.contains(tri->verts()[2]);

                    if (vert0_on || vert1_on || vert2_on)
                    {
                        //   The triangle has one vertex on the boundary - check edges now.

                        bool edge0_on = all_edges_for_edge_tris.contains(tri->edges()[0]);
                        bool edge1_on = all_edges_for_edge_tris.contains(tri->edges()[1]);
                        bool edge2_on = all_edges_for_edge_tris.contains(tri->edges()[2]);

                        if (edge0_on || edge1_on || edge2_on)
                        {
                            //   We have a match - add the triangle and its edges.
                            //
                            //   We also want to search again just in case we have additional
                            //      triangles enter as a result of this add

                            edge_tris.insert(tri);

                            all_edges_for_edge_tris.insert(tri->edges()[0]);
                            all_edges_for_edge_tris.insert(tri->edges()[1]);
                            all_edges_for_edge_tris.insert(tri->edges()[2]);

                            repeat_search = true;
                        }
                    }
                }
            }
        } while (repeat_search);

        //  Finished - return the triangles;

        return edge_tris;
    }

    std::unordered_set<const TopoTri*> MeshTopoCache::tris_inside_boundaries(
        const std::vector<TopoEdgeBoundary>& boundaries, const TopoTri& seed_triangle_inside_boundary,
        uint32_t max_num_tris_before_failure) const
    {
        //  Starting from the seed triangle, grow the surface outward until we run out of triangles to process.
        //      Along the way, keep track of the boundary edges, we use them below to get reliable boundaries.

        std::unordered_set<const TopoEdge*> boundary_edges;
        std::unordered_set<const TopoEdge*> boundary_edges_found;
        std::unordered_set<const TopoEdge*> edges_processed;
        std::unordered_set<const TopoTri*> tris_inside_boundary;
        std::vector<const TopoTri*> tris_to_process;

        for (auto& current_boundary : boundaries)
        {
            boundary_edges.insert(current_boundary.edges().begin(), current_boundary.edges().end());
        }

        tris_to_process.emplace_back(&seed_triangle_inside_boundary);

        while (!tris_to_process.empty())
        {
            const TopoTri* current_tri = tris_to_process.back();
            tris_to_process.pop_back();

            tris_inside_boundary.insert(current_tri);

            if (tris_inside_boundary.size() > max_num_tris_before_failure)
            {
                return std::unordered_set<const TopoTri*>();
            }

            for (auto current_edge : current_tri->edges())
            {
                if (edges_processed.contains(current_edge))
                {
                    continue;
                }

                edges_processed.insert(current_edge);

                if (boundary_edges.contains(current_edge))
                {
                    boundary_edges_found.insert(current_edge);
                    continue;
                }
                else if (current_edge->triangles().size() == 2)
                {
                    if (current_edge->triangles()[0] != current_tri)
                    {
                        tris_to_process.push_back(current_edge->triangles()[0]);
                    }
                    else
                    {
                        tris_to_process.push_back(current_edge->triangles()[1]);
                    }
                }
                else
                {
                    std::cout << "Found edge with three or more incidences" << std::endl;  //   TODO fix with error
                }
            }
        }

        return tris_inside_boundary;
    }

    std::ostream& operator<<(std::ostream& out, const TopoVert& vertex)
    {
        out << "ref(" << vertex.index() << ") "
            << "e(" << vertex.edges().size() << "):";

        for (const TopoEdge* e : vertex.edges())
        {
            out << e << ";";
        }

        out << " "
            << "t(" << vertex.triangles().size() << "):";

        for (auto t : vertex.triangles())
        {
            out << t << ";";
        }

        return (out);
    }

    std::ostream& operator<<(std::ostream& out, const TopoEdge& edge)
    {
        out << "v(2):" << edge.verts()[0] << "(" << edge.verts()[0]->index() << ");" << edge.verts()[1] << "("
            << edge.verts()[1]->index() << ");";
        out << " "
            << "t(" << edge.triangles().size() << "):";

        for (auto t : edge.triangles())
        {
            out << t << ";";
        }

        return (out);
    }

    std::ostream& operator<<(std::ostream& out, const TopoTri& tri)
    {
        out << "ref(" << tri.ref() << ") ";
        out << "v(3):" << tri.verts()[0] << "(" << tri.verts()[0]->index() << ");" << tri.verts()[1] << "("
            << tri.verts()[1]->index() << ");" << tri.verts()[2] << "(" << tri.verts()[2]->index() << ");";
        out << " ";
        out << "e(3):" << tri.edges()[0] << ";" << tri.edges()[1] << ";" << tri.edges()[2] << ";";
        return out;
    }

    void TriangleByIndicesVectorTopoCache::print()
    {
        using std::cout;
        using std::endl;

        cout << "dumping remeshing cache for debug..." << endl;
        cout << "TRIS" << endl;
        int tri_count = 0;

        for (auto& t : triangles())
        {
            cout << " " << &t << ": " << t << endl;
            tri_count++;
        }

        cout << "There were " << tri_count << " TRIS" << endl;
        cout << "EDGES" << endl;
        int edge_count = 0;

        for (auto& e : edges())
        {
            cout << " " << &e << ": " << endl;
            cout << "  v " << e.verts()[0] << "; " << e.verts()[1] << endl;
            cout << "  t (" << e.triangles().size() << ")" << endl;
            for (auto t : e.triangles()) cout << "    " << t << endl;
            edge_count++;
        }

        cout << "There were " << edge_count << " EDGES" << endl;
        cout << "VERTS" << endl;
        int vert_count = 0;

        for (auto& v : vertices())
        {
            cout << " " << &v << ": ref(" << v.index() << ")" << endl;
            cout << "  e (" << v.edges().size() << ")" << endl;
            for (auto e : v.edges()) cout << "    " << &e << endl;
            cout << "  t (" << v.triangles().size() << ")" << endl;
            for (auto t : v.triangles()) cout << "    " << &t << endl;
            vert_count++;
        }

        cout << "There were " << vert_count << " VERTS" << endl;
    }

}  // namespace Cork::Meshes
