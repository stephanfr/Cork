// +-------------------------------------------------------------------------
// | TopoCache.cpp
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



#include "intersection/gmpext4.h"

#include "mesh/TopoCache.h"

#include "intersection/quantization.h"

#include <tbb/spin_mutex.h>
#include <tbb/task_group.h>




namespace Cork
{



	TopoCache::TopoCache( MeshBase&					owner,
						  TopoCacheWorkspace&		workspace )
		: m_mesh(owner),
		  m_workspace(workspace),
		  m_meshTriangles( owner.triangles() ),
		  m_meshVertices( owner.vertices() ),
		  m_topoVertexList( m_workspace ),
		  m_topoEdgeList( m_workspace ),
		  m_topoTriList( m_workspace )
	{
		workspace.reset();

		init();
	}




	void TopoCache::init()
	{
		//	First lay out vertices

		for(uint i=0; i< m_meshVertices.size(); i++)
		{
			m_topoVertexList.emplace_back( i );
		}

		// We need to still do the following
		//  * Generate TopoTris
		//  * Generate TopoEdges
		// ---- Hook up references between
		//  * Triangles and Vertices
		//  * Triangles and Edges
		//  * Vertices and Edges

		// We handle two of these items in a pass over the triangles,
		//  * Generate TopoTris
		//  * Hook up Triangles and Vertices
		// building a structure to handle the edges as we go:

		std::vector<TopoEdgePrototypeVector>			edgeacc( m_meshVertices.size() );

		for( size_t i = 0; i < m_meshTriangles.size(); i++ )
		{
			const CorkTriangle&		ref_tri = m_meshTriangles[i];

			// triangles <--> verts

			size_t		vids0 = ref_tri[0];
			size_t		vids1 = ref_tri[1];
			size_t		vids2 = ref_tri[2];

			TopoTri* tri = m_topoTriList.emplace_back( i, m_topoVertexList.getPool()[vids0], m_topoVertexList.getPool()[vids1], m_topoVertexList.getPool()[vids2] );

			// then, put these in arbitrary but globally consistent order

			if( vids0 > vids1 )
			{
				std::swap( vids0, vids1 );
			}

			if( vids1 > vids2 )
			{
				std::swap( vids1, vids2 );
			}

			if( vids0 > vids1 )
			{
				std::swap( vids0, vids1 );
			}

			// and accrue in structure

			TopoVert*	v0 = &( m_topoVertexList.getPool()[vids0] );
			TopoVert*	v1 = &( m_topoVertexList.getPool()[vids1] );
			TopoVert*	v2 = &( m_topoVertexList.getPool()[vids2] );

			//	Create edges and link them to the triangle

			TopoEdge*				edge01;
			TopoEdge*				edge02;
			TopoEdge*				edge12;

			{
				TopoEdgePrototype&		edge01Proto = edgeacc[vids0].find_or_add( vids1 );

				edge01 = edge01Proto.edge();

				if( edge01 == nullptr )
				{
					edge01 = edge01Proto.setEdge( m_topoEdgeList.emplace_back( v0, v1 ) );
				}

				edge01->triangles().insert( tri );


				TopoEdgePrototype&		edge02Proto = edgeacc[vids0].find_or_add( vids2 );

				edge02 = edge02Proto.edge();

				if( edge02 == nullptr )
				{
					edge02 = edge02Proto.setEdge( m_topoEdgeList.emplace_back( v0, v2 ) );
				}

				edge02->triangles().insert( tri );


				TopoEdgePrototype&		edge12Proto = edgeacc[vids1].find_or_add( vids2 );

				edge12 = edge12Proto.edge();

				if( edge12 == nullptr )
				{
					edge12 = edge12Proto.setEdge( m_topoEdgeList.emplace_back( v1, v2 ) );
				}

				edge12->triangles().insert( tri );
			}
			//	We swapped around indices, so now fix the edge assignments

			tri->AssignEdges( v0, v1, v2, edge01, edge02, edge12 );
		}

	}



	void TopoCache::commit()
	{
		// record which vertices are live
    
		std::vector<bool>		live_verts( m_meshVertices.size(), false );		//	All initialized to zero

		for( auto& vert : m_topoVertexList )
		{
			live_verts[vert.ref()] = true;
		}
    
		// record which triangles are live, and record connectivity
    
		std::vector<bool>		live_tris( m_meshTriangles.size(), false );		//	All initialized to zero

		for( auto& tri : m_topoTriList )
		{
			live_tris[tri.ref()] = true;

			for( size_t k = 0; k < 3; k++ )
			{
				m_meshTriangles[tri.ref()][k] = tri.verts()[k]->ref();
			}
		}
    
		// compact the vertices and build a remapping function
    
		std::vector<size_t>		vmap;
	
		vmap.reserve( m_meshVertices.size() );

		uint write = 0;
    
		for( size_t read = 0; read < m_meshVertices.size(); read++ )
		{
			if(live_verts[read])
			{
				vmap.emplace_back( write );
				m_meshVertices[write] = m_meshVertices[read];
				write++;
			}
			else
			{
				vmap.emplace_back( INVALID_ID );
			}
		}

		m_meshVertices.resize(write);
    
		// rewrite the vertex reference ids
    
		for( auto& vert : m_topoVertexList )
		{
			vert.setRef( vmap[vert.ref()] );
		}
    
		std::vector<size_t>		tmap;
		tmap.reserve( m_meshTriangles.size() );

		write = 0;
    
		for( size_t read = 0; read < m_meshTriangles.size(); read++ )
		{
			if(live_tris[read])
			{
				tmap.emplace_back( write );
				m_meshTriangles[write] = m_meshTriangles[read];
			
				for (uint k = 0; k < 3; k++)
				{
					m_meshTriangles[write][k] = vmap[m_meshTriangles[write][k]];
				}
            
				write++;
			}
			else
			{
				tmap.emplace_back( INVALID_ID );
			}
		}
    
		m_meshTriangles.resize(write);
    
		// rewrite the triangle reference ids
    
		for( auto& tri : triangles() )
		{
			tri.setRef( tmap[tri.ref()] );
		}
	}






	std::ostream& operator<<( std::ostream &out, const TopoVert& vertex )
	{
		out << "ref(" << vertex.ref() << ") " << "e(" << vertex.edges().size() << "):";
    
		for(const TopoEdge* e : vertex.edges())
		{
			out << e << ";";
		}

		out << " " << "t(" << vertex.triangles().size() << "):";

		for(auto t : vertex.triangles())
		{
			out << t << ";";
		}

		return( out );
	}


	std::ostream& operator<<(std::ostream &out, const TopoEdge& edge)
	{
		out << "v(2):" << edge.verts()[0] << "(" << edge.verts()[0]->ref() << ");"
					   << edge.verts()[1] << "(" << edge.verts()[1]->ref() << ");";
		out << " " << "t(" << edge.triangles().size() << "):";
    
		for(auto t : edge.triangles())
		{
			out << t << ";";
		}

		return( out );
	}


	std::ostream& operator<<(std::ostream &out, const TopoTri& tri)
	{
		out << "ref(" << tri.ref() << ") ";
		out << "v(3):" << tri.verts()[0] << "(" << tri.verts()[0]->ref() << ");"
					   << tri.verts()[1] << "(" << tri.verts()[1]->ref() << ");"
					   << tri.verts()[2] << "(" << tri.verts()[2]->ref() << ");";
		out << " ";
		out << "e(3):" << tri.edges()[0] << ";"
					   << tri.edges()[1] << ";"
					   << tri.edges()[2] << ";";
		return out;
	}



	void TopoCache::print()
	{
		using std::cout;
		using std::endl;
    
		cout << "dumping remeshing cache for debug..." << endl;
		cout << "TRIS" << endl;
		int tri_count = 0;
    
	
		for( auto& t : triangles() )
		{
			cout << " " << &t << ": " << t << endl;
			tri_count++;
		}
    
		cout << "There were " << tri_count << " TRIS" << endl;
		cout << "EDGES" << endl;
		int edge_count = 0;
    
		for( auto& e : edges() )
		{
			cout << " " << &e << ": " << endl;
			cout << "  v " << e.verts()[0] << "; "
						   << e.verts()[1] << endl;
			cout << "  t (" << e.triangles().size() << ")" << endl;
			for(auto t : e.triangles())
			cout << "    " << t << endl;
			edge_count++;
		}
    
		cout << "There were " << edge_count << " EDGES" << endl;
		cout << "VERTS" << endl;
		int vert_count = 0;
    
		for( auto& v : vertices() )
		{
			cout << " " << &v << ": ref(" << v.ref() << ")" << endl;
			cout << "  e (" << v.edges().size() << ")" << endl;
			for( auto e : v.edges() )
			cout << "    " << &e << endl;
			cout << "  t (" << v.triangles().size() << ")" << endl;
			for( auto t : v.triangles())
			cout << "    " << &t << endl;
			vert_count++;
		}
    
		cout << "There were " << vert_count << " VERTS" << endl;
	}






	// support functions for validity check

	template<class T, class Container>
	inline
	bool count(const Container &contain, const T &val)
	{
		uint c=0;
		for (const T &t : contain)
		{
			if (t == val)
			{
				c++;
			}
		}

		return( c );
	}


	template<class T>
	inline
	bool count2(const T arr[], const T &val)
	{
		return ((arr[0] == val)? 1 : 0) + ((arr[1] == val)? 1 : 0);
	}


	template<class T>
	inline
	bool count3(const T arr[], const T &val)
	{
		return ((arr[0] == val)? 1 : 0) + ((arr[1] == val)? 1 : 0) + ((arr[2] == val)? 1 : 0);
	}


	/*
	bool TopoCache::isValid()
	{
		//print();
		std::set<Vptr> vaddr;
		std::set<Eptr> eaddr;
		std::set<Tptr> taddr;
		verts.for_each([&vaddr](Vptr v) { vaddr.insert(v); });
		edges.for_each([&eaddr](Eptr e) { eaddr.insert(e); });
		tris.for_each( [&taddr](Tptr t) { taddr.insert(t); });
    
		// check verts
		verts.for_each([&](Vptr v)
		{
			ENSURE(v->ref < mesh->verts.size());
        
			// make sure each edge pointer goes somewhere and that
			// the pointed-to site also points back correctly
        
			for(Eptr e : v->edges)
			{
				ENSURE(eaddr.count(e) > 0); // pointer is good
				ENSURE(count2(e->verts, v) == 1); // back-pointer is good
			}

			for(Tptr t : v->tris)
			{
				ENSURE(taddr.count(t) > 0);
				ENSURE(count3(t->verts, v) == 1);
			}
		});
    
		// check edges
		edges.for_each([&](Eptr e)
		{
			// check for non-degeneracy
			ENSURE(e->verts[0] != e->verts[1]);

			for(uint k=0; k<2; k++)
			{
				Vptr v = e->verts[k];
				ENSURE(vaddr.count(v) > 0);
				ENSURE(count(v->edges, e) == 1);
			}

			for(Tptr t : e->tris)
			{
				ENSURE(taddr.count(t) > 0);
				ENSURE(count3(t->edges, e) == 1);
			}
		});
    
		// check triangles
		tris.for_each([&](Tptr t)
		{
			// check for non-degeneracy
			ENSURE(t->verts[0] != t->verts[1] && t->verts[1] != t->verts[2]
											  && t->verts[0] != t->verts[2]);
			for(uint k=0; k<3; k++)
			{
				Vptr v = t->verts[k];
				ENSURE(vaddr.count(v) > 0);
				ENSURE(count(v->tris, t) == 1);
            
				Eptr e = t->edges[k];
				ENSURE(eaddr.count(e) == 1);
				ENSURE(count(e->tris, t) == 1);
            
				// also need to ensure that the edges are opposite the
				// vertices as expected
				Vptr v0 = e->verts[0];
				Vptr v1 = e->verts[1];
				ENSURE((v0 == t->verts[(k+1)%3] && v1 == t->verts[(k+2)%3])
					|| (v0 == t->verts[(k+2)%3] && v1 == t->verts[(k+1)%3]));
			}
		});
    
		return true;
	}
	*/

}





