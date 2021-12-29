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

#include "mesh/topo_cache.hpp"

#include <tbb/spin_mutex.h>
#include <tbb/task_group.h>

#include "math/gmpext4.hpp"
#include "mesh/mesh_base.hpp"

namespace Cork::Meshes
{

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

        for (auto& vert : m_topoVertexList)
        {
            live_verts[vert.index()] = true;
        }

        // record which triangles are live, and record connectivity

        BooleanVector<TriangleByIndicesIndex> live_tris(mesh_triangles_.size());

        for (auto& tri : m_topoTriList)
        {
            live_tris[tri.ref()] = true;

            for (size_t k = 0; k < 3; k++)
            {
                mesh_triangles_[tri.ref()][k] = tri.verts()[k]->index();
            }
        }

        // compact the vertices and build a remapping function

        std::vector<VertexIndex> vertex_map;

        vertex_map.reserve(mesh_vertices_.size());

        VertexIndex vert_write(0u);

        for (VertexIndex::integer_type read = 0; read < mesh_vertices_.size(); read++)
        {
            if (live_verts[read])
            {
                vertex_map.emplace_back(vert_write);
                mesh_vertices_[vert_write] = mesh_vertices_[VertexIndex(read)];
                vert_write++;
            }
            else
            {
                vertex_map.emplace_back(Primitives::UNINTIALIZED_INDEX);
            }
        }

        mesh_vertices_.resize(VertexIndex::integer_type(vert_write));

        // rewrite the vertex reference ids

        for (auto& vert : m_topoVertexList)
        {
            vert.set_index(vertex_map[VertexIndex::integer_type(vert.index())]);
        }

        std::vector<TriangleByIndicesIndex> tmap;
        tmap.reserve(mesh_triangles_.size());

        TriangleByIndicesIndex tri_write = 0U;

        for (TriangleByIndicesIndex read = 0U; read < mesh_triangles_.size(); read++)
        {
            if (live_tris[read])
            {
                tmap.emplace_back(tri_write);
                mesh_triangles_[tri_write] = mesh_triangles_[read];

                for (uint k = 0; k < 3; k++)
                {
                    mesh_triangles_[tri_write][k] =
                        vertex_map[VertexIndex::integer_type(mesh_triangles_[tri_write][k])];
                }

                tri_write++;
            }
            else
            {
                tmap.emplace_back(Primitives::UNINTIALIZED_INDEX);
            }
        }

        mesh_triangles_.resize(TriangleByIndicesIndex::integer_type(tri_write));

        // rewrite the triangle reference ids

        for (auto& tri : triangles())
        {
            tri.set_ref(tmap[TriangleByIndicesIndex::integer_type(tri.ref())]);
        }
    }

    std::vector<const TopoEdge*> MeshTopoCache::topo_edge_boundary(const BoundaryEdge& boundary) const
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

        return topo_edge_boundary;
    }

    std::set<const TopoTri*> MeshTopoCache::tris_along_edges(const std::vector<const TopoEdge*>& boundary) const
    {
        std::set<const TopoVert*> edge_verts;
        std::set<const TopoTri*> edge_tris;
        std::set<const TopoEdge*> all_edges_for_edge_tris;

        //  First, add all triangles that share an edge.

        for (auto edge : boundary)
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
