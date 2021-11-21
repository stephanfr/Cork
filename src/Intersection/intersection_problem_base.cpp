// +-------------------------------------------------------------------------
// | intersection_problem_base.cpp
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

#include "intersection/intersection_problem_base.hpp"

namespace Cork::Intersection
{

    IntersectionProblemBase::IntersectionProblemBase(MeshBase& owner_mesh, const Math::Quantizer& quantizer)
        : workspace_(std::move(IntersectionWorkspaceFactory::GetInstance())),
          owner_mesh_(owner_mesh),
          topo_cache_(owner_mesh, quantizer),
          quantizer_(quantizer),
          perturbation_(quantizer_),
          m_gluePointMarkerList(*(workspace_.get())),
          m_isctVertTypeList(*(workspace_.get())),
          m_origVertTypeList(*(workspace_.get())),
          m_isctEdgeTypeList(*(workspace_.get())),
          m_origEdgeTypeList(*(workspace_.get())),
          m_splitEdgeTypeList(*(workspace_.get())),
          m_genericTriTypeList(*(workspace_.get()))
    {
        //	Initialize all the triangles to NOT have an associated tprob
        //		and set the boolAlgData value based on the input triangle

        for (TopoTri& t : topo_cache_.triangles())
        {
            t.clear_triangle_problem_association();
            t.setBoolAlgData(owner_mesh_.triangles()[t.ref()].bool_alg_data());
        }

        //	Initialize all of the edge solid IDs

        for (TopoEdge& e : topo_cache_.edges())
        {
            e.set_boolean_algorithm_data(e.triangles().front()->boolAlgData());
        }
    }

    void IntersectionProblemBase::perturbPositions()
    {
        for (auto& vertex : topo_cache_.vertices())
        {
            Primitives::Vector3D perturbation = perturbation_.getPerturbation();

            vertex.perturb(perturbation);
        }
    }

    void IntersectionProblemBase::CreateBoundingVolumeHierarchy()
    {
        std::unique_ptr<AABVH::GeomBlobVector> edge_geoms(new AABVH::GeomBlobVector());

        edge_geoms->reserve(topo_cache_.edges().size());

        for (auto& e : topo_cache_.edges())
        {
            edge_geoms->emplace_back(e);
        }

        edge_bvh_.reset(new AABVH::AxisAlignedBoundingVolumeHierarchy(edge_geoms, workspace(),
                                                                      owner_mesh_.solver_control_block()));
    }

    void IntersectionProblemBase::FindEdgeAndTriangleIntersections(AABVH::IntersectionType selfOrBooleanIntersection,
                                                                   TriangleAndIntersectingEdgesQueue& triangleAndEdges)
    {
        CreateBoundingVolumeHierarchy();

        //	Search for intersections, either in multiple threads or in a single thread
        /*
                        if( ownerMesh().solverControlBlock().useMultipleThreads() )
                        {
                            //	Multithreaded search

                            assert( triangles().isCompact() );

                            tbb::parallel_for( tbb::blocked_range<TopoTriList::PoolType::iterator>(
           triangles().getPool().begin(), triangles().getPool().end(), ( triangles().getPool().size() / 4 ) - 1
           ),
                                [&] ( tbb::blocked_range<TopoTriList::PoolType::iterator> triangles )
                            {
                                TopoEdgePointerVector			edges;

                                for( TopoTri& t : triangles )
                                {
                                    m_edgeBVH->EdgesIntersectingTriangle( t, selfOrBooleanIntersection, edges );

                                    if (!edges.empty())
                                    {
                                        triangleAndEdges.push( new TriangleAndIntersectingEdgesMessage( t, edges
           ));

                                        edges.clear();
                                    }
                                }
                            }, tbb::simple_partitioner() );
                        }
                        else
                        {
                        */
        //	Single threaded search

        for (TopoTri& tri : topo_cache_.triangles())
        {
            TopoEdgeReferenceVector edges( std::move( edge_bvh_->EdgesIntersectingTriangle(tri, selfOrBooleanIntersection)));

            if (!edges.empty())
            {
                triangleAndEdges.push(new TriangleAndIntersectingEdgesMessage(tri, edges));
            }
        }
        //				}

        //	Push the end of queue message

        triangleAndEdges.push(new TriAndEdgeQueueEnd());
    }

    std::unique_ptr<std::vector<Primitives::Vector3D>> IntersectionProblemBase::dumpIsctPoints()
    {
        auto points = std::make_unique<std::vector<Primitives::Vector3D>>();

        points->resize(m_gluePointMarkerList.size());

        uint write = 0;

        for (auto& glue : m_gluePointMarkerList)
        {
            assert(glue.vertices_to_be_glued().size() > 0);
            IsctVertType* iv = glue.vertices_to_be_glued()[0];

            (*points)[write] = iv->coordinate();
            write++;
        }

        return points;
    }

}  // namespace Cork::Intersection