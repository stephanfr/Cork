// +-------------------------------------------------------------------------
// | intersection.cpp
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

#include "intersection/edge_cache.hpp"
#include "intersection/glue_and_generic_types.hpp"
#include "intersection/intersection_problem.hpp"
#include "intersection/intersection_problem_base.hpp"
#include "intersection/triangle_problem.hpp"
#include "util/CachingFactory.h"

namespace Cork::Intersection
{
    using TriangleByIndicesIndex = Primitives::TriangleByIndicesIndex;

    using TopoVert = Meshes::TopoVert;
    using TopoEdge = Meshes::TopoEdge;
    using TopoTri = Meshes::TopoTri;
    using TopoEdgePointerVector = Meshes::TopoEdgePointerVector;

    using CorkTriangle = Meshes::CorkTriangle;

    using MeshBase = Meshes::MeshBase;

    //  The following template classes are needed to insure the correct delete function is associated
    //      with the allocation function used for elements passed to the Triangle library.

    template <typename T>
    struct TriangulateDeleter
    {
        void operator()(T* pointer) { trifree(pointer); }
    };

    template <typename T>
    struct FreeDeleter
    {
        void operator()(T* pointer) { free(pointer); }
    };

    class TriTripleTemp
    {
       public:
        TriTripleTemp(const TopoTri& tp0, const TopoTri& tp1, const TopoTri& tp2) : t0(tp0), t1(tp1), t2(tp2) {}

        const TopoTri& t0;
        const TopoTri& t1;
        const TopoTri& t2;
    };

/*
    class IntersectionProblemWorkspace : public IntersectionWorkspace
    {
       public:
        IntersectionProblemWorkspace()
        {
            m_gluePointMarkerPool.reserve(100000);
            m_isctVertexTypePool.reserve(200000);
            m_isctEdgeTypePool.reserve(300000);
            m_genericTriTypePool.reserve(100000);
            m_triangleProblemList.reserve(100000);
            m_isctVertexPointerPool.reserve(200000);
            m_isctEdgePointerPool.reserve(100000);
            m_genericTriPointerPool.reserve(100000);
        }

        virtual ~IntersectionProblemWorkspace() noexcept {};

        void reset()
        {
            m_gluePointMarkerPool.clear();
            m_isctVertexTypePool.clear();
            m_isctEdgeTypePool.clear();
            m_genericTriTypePool.clear();
            m_triangleProblemList.clear();
            m_isctVertexPointerPool.clear();
            m_isctEdgePointerPool.clear();
            m_genericTriPointerPool.clear();

            m_AABVHWorkspace.reset();
        }

        operator GluePointMarkerList::PoolType &() { return (m_gluePointMarkerPool); }

        operator IsctVertTypeList::PoolType &() { return (m_isctVertexTypePool); }

        operator IsctEdgeTypeList::PoolType &() { return (m_isctEdgeTypePool); }

        operator GenericTriTypeList::PoolType &() { return (m_genericTriTypePool); }

        operator TriangleProblemList&() { return (m_triangleProblemList); }

        operator IntersectionVertexPointerList::PoolType &() { return (m_isctVertexPointerPool); }

        operator IntersectionEdgePointerList::PoolType &() { return (m_isctEdgePointerPool); }

        operator GenericTriPointerList::PoolType &() { return (m_genericTriPointerPool); }

        operator AABVH::Workspace &() { return (m_AABVHWorkspace); }

       private:
        GluePointMarkerList::PoolType m_gluePointMarkerPool;
        IsctVertTypeList::PoolType m_isctVertexTypePool;  //	Covers both the intersection and original vertex
                                                          // types
        IsctEdgeTypeList::PoolType m_isctEdgeTypePool;
        GenericTriTypeList::PoolType m_genericTriTypePool;
        TriangleProblemList m_triangleProblemList;

        IntersectionVertexPointerList::PoolType m_isctVertexPointerPool;
        IntersectionEdgePointerList::PoolType m_isctEdgePointerPool;
        GenericTriPointerList::PoolType m_genericTriPointerPool;

        AABVH::Workspace m_AABVHWorkspace;
    };

    typedef SEFUtility::CachingFactory<IntersectionProblemWorkspace> IntersectionWorkspaceFactory;
*/


    class IntersectionSolverImpl : public IntersectionProblemBase, public IntersectionSolver
    {
       public:
        IntersectionSolverImpl(
            MeshBase& owner, const Math::Quantizer& quantizer, const Primitives::BBox3D& intersectionBBox);

        virtual ~IntersectionSolverImpl() { reset(); }

        //	Implementation of IntersectionProblemIfx

        IntersectionProblemResult FindIntersections() final;
        IntersectionProblemResult ResolveAllIntersections() final;

        void commit() { TopoCache::commit(); }

        //	Other IntersectionProblem methods

        TriangleProblem* getTprob(const TopoTri& t)
        {
            TriangleProblem* prob = reinterpret_cast<TriangleProblem*>(const_cast<void*>(t.data()));

            if (!prob)
            {
                m_triangleProblemList.emplace_back(*this, t);
                prob = &(m_triangleProblemList.back());
            }

            return (prob);
        }

        void dumpIsctEdges(std::vector<std::pair<Primitives::Vector3D, Primitives::Vector3D>>* edges)
        {
            edges->clear();

            for (auto& tprob : m_triangleProblemList)
            {
                for (IsctEdgeType* ie : tprob.iedges())
                {
                    GenericVertType* gv0 = ie->ends()[0];
                    GenericVertType* gv1 = ie->ends()[1];

                    edges->push_back(std::make_pair(gv0->coordinate(), gv1->coordinate()));
                }
            }
        }

       private:

        TriangleProblemList m_triangleProblemList;

        // if we encounter ambiguous degeneracies, then this
        // routine returns false, indicating that the computation aborted.

        enum TryToFindIntersectionsResultCodes
        {
            SUCCESS = 0,
            OUT_OF_MEMORY,
            TRI_EGDE_DEGENERACIES,
            TRI_TRI_TRI_INTERSECTIONS_FAILED
        };

        typedef SEFUtility::Result<TryToFindIntersectionsResultCodes> TryToFindIntersectionsResult;

        TryToFindIntersectionsResult tryToFindIntersections();
        bool findTriTriTriIntersections();

        void reset();

        void createRealTriangles(TriangleProblem& tprob, EdgeCache& ecache)
        {
            for (auto& gt : tprob.gtris())
            {
                TopoTri* t = TopoCache::newTri();

                std::array<TopoVert*, 3> vertices;
                std::array<TopoEdge*, 3> edges;

                GenericTriType* genericTri = gt.pointer();

                genericTri->set_concrete_triangle(t);

                CorkTriangle& tri = TopoCache::ownerMesh().triangles()[t->ref()];

                for (uint k = 0; k < 3; k++)
                {
                    TopoVert& v = genericTri->vertices()[k]->concrete_vertex();
                    vertices[k] = &v;
                    tri[k] = v.ref();

                    edges[k] = ecache.getTriangleEdge(genericTri, k, tprob.triangle());
                }

                t->setVertices(vertices);
                t->setEdges(edges);

                fillOutTriData(*t, tprob.triangle());
            }

            //	Once all the pieces are hooked up, let's kill the old triangle!
            //		We need to cast away the const here as well...

            TopoCache::deleteTri(&(const_cast<TopoTri&>(tprob.triangle())));
        }
    };

    IntersectionProblemBase::IntersectionProblemBase(MeshBase& owner, const Math::Quantizer& quantizer,
                                                     const Primitives::BBox3D& intersectionBBox)
        : TopoCache(owner, quantizer),
          m_intersection_problem_workspace(std::move(IntersectionWorkspaceFactory::GetInstance())),
          m_quantizer(quantizer),
          m_perturbation(quantizer),
          m_intersectionBBox(std::make_unique<Primitives::BBox3D>(intersectionBBox)),
          m_gluePointMarkerList(*(m_intersection_problem_workspace.get())),
          m_isctVertTypeList(*(m_intersection_problem_workspace.get())),
          m_origVertTypeList(*(m_intersection_problem_workspace.get())),
          m_isctEdgeTypeList(*(m_intersection_problem_workspace.get())),
          m_origEdgeTypeList(*(m_intersection_problem_workspace.get())),
          m_splitEdgeTypeList(*(m_intersection_problem_workspace.get())),
          m_genericTriTypeList(*(m_intersection_problem_workspace.get()))
    {
        //	Initialize all the triangles to NOT have an associated tprob
        //		and set the boolAlgData value based on the input triangle

        for (TopoTri& t : triangles())
        {
            t.setData(nullptr);
            t.setBoolAlgData(ownerMesh().triangles()[t.ref()].boolAlgData());
        }

        //	Initialize all of the edge solid IDs

        for (TopoEdge& e : edges())
        {
            e.setBoolAlgData(e.triangles().front()->boolAlgData());
        }
    }

    IntersectionSolverImpl::IntersectionSolverImpl(
        MeshBase& owner, const Math::Quantizer& quantizer, const Primitives::BBox3D& intersectionBBox)
        : IntersectionProblemBase(owner, quantizer, intersectionBBox)
//          m_triangleProblemList(*(m_intersection_problem_workspace.get()))
    {
    }

    IntersectionSolverImpl::TryToFindIntersectionsResult IntersectionSolverImpl::tryToFindIntersections()
    {
        m_exactArithmeticContext.degeneracy_count = 0;

        TriangleAndIntersectingEdgesQueue trianglesAndEdges;

        tbb::task_group taskGroup;

        taskGroup.run([&] {
            FindEdgeAndTriangleIntersections(AABVH::IntersectionType::BOOLEAN_INTERSECTION, trianglesAndEdges);
        });

        while (true)
        {
            TriAndEdgeQueueMessage* msg;

            trianglesAndEdges.pop(msg);

            const std::unique_ptr<TriAndEdgeQueueMessage> currentMessage(msg);

            if (currentMessage->type() == TriAndEdgeQueueMessage::MessageType::END_OF_MESSAGES)
            {
                break;
            }

            TopoTri& triangle = ((TriangleAndIntersectingEdgesMessage&)(*currentMessage)).triangle();

            for (const TopoEdge* edge : ((TriangleAndIntersectingEdgesMessage&)(*currentMessage)).edges())
            {
                if (triangle.intersectsEdge(*edge, m_quantizer, m_exactArithmeticContext))
                {
                    GluePointMarker* glue = m_gluePointMarkerList.emplace_back(
                        GluePointMarker::IntersectionType::EDGE_TRIANGLE, *edge, triangle);

                    // first add point and edges to the pierced triangle

                    IsctVertType* iv = getTprob(triangle)->addInteriorEndpoint(*edge, *glue);

                    for (auto tri : edge->triangles())
                    {
                        getTprob(*tri)->addBoundaryEndpoint(triangle, *edge, iv);
                    }
                }

                if (m_exactArithmeticContext.degeneracy_count != 0)
                {
                    break;
                }
            }

            if (m_exactArithmeticContext.degeneracy_count != 0)
            {
                break;
            }
        };

        taskGroup.wait();

        if (m_exactArithmeticContext.degeneracy_count > 0)
        {
            return (TryToFindIntersectionsResult::failure(
                TryToFindIntersectionsResultCodes::TRI_EGDE_DEGENERACIES,
                "Degeneracies Detected during Triangle Edge instersection computations."));
        }

        if (!findTriTriTriIntersections())
        {
            return (TryToFindIntersectionsResult::failure(
                TryToFindIntersectionsResultCodes::TRI_TRI_TRI_INTERSECTIONS_FAILED,
                "Three Triangle Intersection computation failed."));
        }

        return (TryToFindIntersectionsResult::success());
    }

    bool IntersectionSolverImpl::findTriTriTriIntersections()
    {
        // we're going to peek into the triangle problems in order to
        // identify potential candidates for Tri-Tri-Tri intersections

        std::vector<TriTripleTemp> triples;

        for (auto& tprob : m_triangleProblemList)
        {
            const TopoTri& t0 = tprob.triangle();

            // Scan pairs of existing edges to create candidate triples

            for (IntersectionEdgePointerList::iterator ie1 = tprob.iedges().begin(); ie1 != tprob.iedges().end(); ie1++)
            {
                IntersectionEdgePointerList::iterator itrNext = ie1;

                for (IntersectionEdgePointerList::iterator ie2 = ++itrNext; ie2 != tprob.iedges().end(); ie2++)
                {
                    const TopoTri& t1 = ie1->pointer()->otherTriKey();
                    const TopoTri& t2 = ie2->pointer()->otherTriKey();

                    // This triple might be considered three times,
                    // one for each triangle it contains.
                    // To prevent duplication, only proceed if this is
                    // the least triangle according to an arbitrary ordering

                    if ((&t0 < &t1) && (&t0 < &t2))
                    {
                        // now look for the third edge.  We're not
                        // sure if it exists...

                        TriangleProblem* prob1 = reinterpret_cast<TriangleProblem*>(t1.data());

                        for (IsctEdgeType* ie : prob1->iedges())
                        {
                            if (&(ie->otherTriKey()) == &t2)
                            {
                                // ADD THE TRIPLE
                                triples.emplace_back(TriTripleTemp(t0, t1, t2));
                            }
                        }
                    }
                }
            }
        }

        // Now, we've collected a list of Tri-Tri-Tri intersection candidates.
        // Check to see if the intersections actually exist.

        for (TriTripleTemp& t : triples)
        {
            if (!checkIsct(t.t0, t.t1, t.t2))
            {
                continue;
            }

            // Abort if we encounter a degeneracy

            if (m_exactArithmeticContext.degeneracy_count > 0)
            {
                break;
            }

            GluePointMarker* glue = m_gluePointMarkerList.emplace_back(
                GluePointMarker::IntersectionType::TRIANGLE_TRIANGLE_TRIANGLE, t.t0, t.t1, t.t2);

            getTprob(t.t0)->addInteriorPoint(t.t1, t.t2, *glue);
            getTprob(t.t1)->addInteriorPoint(t.t0, t.t2, *glue);
            getTprob(t.t2)->addInteriorPoint(t.t0, t.t1, *glue);
        }

        if (m_exactArithmeticContext.degeneracy_count > 0)
        {
            return (false);  // restart / abort
        }

        return (true);
    }

    void IntersectionSolverImpl::reset()
    {
        // the data pointer in the triangles points to tproblems
        // that we're about to destroy,
        // so zero out all those pointers first!

        for (auto& tprob : m_triangleProblemList)
        {
            tprob.ResetTopoTriLink();
        }

        m_gluePointMarkerList.clear();

        m_isctVertTypeList.clear();
        m_origVertTypeList.clear();

        m_isctEdgeTypeList.clear();
        m_origEdgeTypeList.clear();
        m_splitEdgeTypeList.clear();

        m_genericTriTypeList.clear();

        m_triangleProblemList.clear();

        m_intersection_problem_workspace.get()->reset();
    }

    IntersectionProblemResult IntersectionSolverImpl::FindIntersections()
    {
        perturbPositions();  // always perturb for safety...

        bool foundIntersections = false;

        do
        {
            TryToFindIntersectionsResult result = tryToFindIntersections();

            foundIntersections = result.succeeded();

            if (!foundIntersections)
            {
                if (result.error_code() == TryToFindIntersectionsResultCodes::OUT_OF_MEMORY)
                {
                    return (IntersectionProblemResult::failure(result, IntersectionProblemResultCodes::OUT_OF_MEMORY,
                                                               "Out of Memory"));
                }

                reset();

                auto perturbAdjustResult = m_perturbation.adjust();

                if (!perturbAdjustResult.succeeded())
                {
                    return (IntersectionProblemResult::failure(
                        perturbAdjustResult, IntersectionProblemResultCodes::EXHAUSTED_PURTURBATION_RETRIES,
                        "Perturbation adjustment failed"));
                }

                perturbPositions();
            }
        } while (!foundIntersections);

        // ok all points put together,
        // all triangle problems assembled.
        // Some intersection edges may have original vertices as endpoints
        // we consolidate the problems to check for cases like these.

        for (auto& tprob : m_triangleProblemList)
        {
            if (!tprob.Consolidate().succeeded())
            {
                return (IntersectionProblemResult::failure(IntersectionProblemResultCodes::CONSOLIDATE_FAILED,
                                                           "Consolidate failed"));
            }
        }

        return (IntersectionProblemResult::success());
    }

    IntersectionProblemResult IntersectionSolverImpl::ResolveAllIntersections()
    {
        //	Subdivide the mesh in each triangle problem.

        for (auto& tprob : m_triangleProblemList)
        {
            auto result = tprob.Subdivide();

            if (!result.succeeded())
            {
                //	Usually, we fail here as a result of a self-intersecting mesh.  Check for that condition now
                //		but we only have to check the current collection of triangles associated with this problem.
                //		We can do that directly using the bounding volume hierarchy.

                Empty3d::ExactArithmeticContext localArithmeticContext;
                unsigned int numIntersections = 0;

                std::set<const TopoTri*> allTris;

                allTris.insert(&tprob.triangle());

                for (auto edge : tprob.iedges())
                {
                    allTris.insert(&edge->otherTriKey());
                }

                for (auto tri : allTris)
                {
                    std::cout << tri->boolAlgData() << "    (" << tri->verts()[0]->quantizedValue().x() << ", "
                              << tri->verts()[0]->quantizedValue().y() << ", " << tri->verts()[0]->quantizedValue().z()
                              << ")    (" << tri->verts()[1]->quantizedValue().x() << ", "
                              << tri->verts()[1]->quantizedValue().y() << ", " << tri->verts()[1]->quantizedValue().z()
                              << ")    (" << tri->verts()[2]->quantizedValue().x() << ", "
                              << tri->verts()[2]->quantizedValue().y() << ", " << tri->verts()[2]->quantizedValue().z()
                              << ")" << std::endl;
                    std::cout.flush();

                    TopoEdgePointerVector edges;

                    m_edgeBVH->EdgesIntersectingTriangle(*tri, AABVH::IntersectionType::SELF_INTERSECTION, edges);

                    for (const TopoEdge* edge : edges)
                    {
                        if (tri->intersectsEdge(*edge, m_quantizer, localArithmeticContext))
                        {
                            numIntersections++;
                        }
                    }

                    edges.clear();
                }

                if (numIntersections > 0)
                {
                    return (IntersectionProblemResult::failure(result,
                                                               IntersectionProblemResultCodes::SELF_INTERSECTING_MESH,
                                                               "Self Intersections found in Mesh"));
                }

                return (IntersectionProblemResult::failure(result, IntersectionProblemResultCodes::SUBDIVIDE_FAILED,
                                                           "Subdivide failed"));
            }
        }

        // now we have diced up triangles inside each triangle problem

        // Let's go through the glue points and create a new concrete
        // vertex object for each of these.

        for (auto& glue : m_gluePointMarkerList)
        {
            createRealPtFromGluePt(glue);
        }

        EdgeCache ecache(*this);

        // Now that we have concrete vertices plugged in, we can
        // go through the diced triangle pieces and create concrete triangles
        // for each of those.
        // Along the way, let's go ahead and hook up edges as appropriate

        for (auto& tprob : m_triangleProblemList)
        {
            createRealTriangles(tprob, ecache);
        }

        // mark all edges as normal by zero-ing out the data

        for (auto& e : TopoCache::edges())
        {
            e.setData(0);
        }

        // then iterate over the edges formed by intersections
        // (i.e. those edges without the boundary flag set in each triangle)
        // and mark those by setting the data pointer

        for (IsctEdgeType& ie : m_isctEdgeTypeList)
        {
            // every ie must be non-boundary
            TopoEdge* e = ecache.maybeEdge(&ie);
            e->setData((void*)1);
        }

        for (auto& se : m_splitEdgeTypeList)
        {
            TopoEdge* e = ecache.maybeEdge(&se);
            e->setData((void*)1);
        }

        // This basically takes care of everything EXCEPT one detail *) The base mesh data structures still need to
        // be compacted This detail should be handled by the calling code...

        return (IntersectionProblemResult::success());
    }

    class SelfIntersectionFinderImpl : public SelfIntersectionFinder
    {
       public:
        SelfIntersectionFinderImpl(
            Primitives::TriangleByIndicesVector& triangles, Primitives::Vertex3DVector& vertices, uint32_t num_edges,
            const Math::Quantizer& quantizer);

        virtual ~SelfIntersectionFinderImpl() { reset(); }

        const std::vector<IntersectionInfo> CheckSelfIntersection() final;

       private:
        SEFUtility::CachingFactory<IntersectionWorkspace>::UniquePtr m_intersection_workspace;

        Math::Quantizer quantizer_;

        Meshes::TopoCacheBase<Primitives::TriangleByIndicesVector> topo_cache_;

        std::unique_ptr<AABVH::AxisAlignedBoundingVolumeHierarchy> m_edgeBVH;

        void reset();

        void CreateBoundingVolumeHierarchy()
        {
            std::unique_ptr<AABVH::GeomBlobVector> edge_geoms(new AABVH::GeomBlobVector());

            edge_geoms->reserve(topo_cache_.edges().size());

            for (auto& e : topo_cache_.edges())
            {
                edge_geoms->emplace_back(e);
            }

            m_edgeBVH.reset(new AABVH::AxisAlignedBoundingVolumeHierarchy(
                edge_geoms, *m_intersection_workspace, Cork::CorkService::get_default_control_block()));
        }
    };

    SelfIntersectionFinderImpl::SelfIntersectionFinderImpl(
        Primitives::TriangleByIndicesVector& triangles, Primitives::Vertex3DVector& vertices, uint32_t num_edges,
        const Math::Quantizer& quantizer)
        : m_intersection_workspace(std::move(SEFUtility::CachingFactory<IntersectionWorkspace>::GetInstance() )),
          quantizer_(quantizer),
          topo_cache_(triangles, vertices, num_edges, quantizer)
    {
        CreateBoundingVolumeHierarchy();
    }

    void SelfIntersectionFinderImpl::reset()
    {
        m_intersection_workspace.get()->reset();
    }

    const std::vector<IntersectionInfo> SelfIntersectionFinderImpl::CheckSelfIntersection()
    {
        Empty3d::ExactArithmeticContext localArithmeticContext;
        std::vector<IntersectionInfo> self_intersecting_edges;

        TopoEdgePointerVector edges;

        for (TopoTri& t : topo_cache_.triangles())
        {
            m_edgeBVH->EdgesIntersectingTriangle(t, AABVH::IntersectionType::SELF_INTERSECTION, edges);

            for (const TopoEdge* edge : edges)
            {
                if (t.intersectsEdge(*edge, quantizer_, localArithmeticContext))
                {
                    std::set<TriangleByIndicesIndex> triangles_sharing_edge;
                    std::set<TriangleByIndicesIndex> triangles_touching_triangles_sharing_edge;

                    for (auto triangle_sharing_edge : edge->triangles())
                    {
                        triangles_sharing_edge.insert(triangle_sharing_edge->source_triangle_id());

                        for (auto touching_edge : triangle_sharing_edge->edges())
                        {
                            for (auto triangle_touching : touching_edge->triangles())
                            {
                                triangles_touching_triangles_sharing_edge.insert(
                                    triangle_touching->source_triangle_id());
                            }
                        }
                    }

                    self_intersecting_edges.emplace_back(Intersection::IntersectionInfo(
                        edge->source_triangle_id(), edge->edge_index(), t.source_triangle_id(), triangles_sharing_edge,
                        triangles_touching_triangles_sharing_edge));
                }
            }

            edges.clear();
        }

        return self_intersecting_edges;
    }

    std::unique_ptr<SelfIntersectionFinder> SelfIntersectionFinder::GetFinder(
        Primitives::TriangleByIndicesVector& triangles, Primitives::Vertex3DVector& vertices, uint32_t num_edges,
        const Math::Quantizer& quantizer)
    {
//        IntersectionWorkspaceFactory::UniquePtr workspace(IntersectionWorkspaceFactory::GetInstance());

        std::unique_ptr<SelfIntersectionFinder> finder(
            new SelfIntersectionFinderImpl(triangles, vertices, num_edges, quantizer));

        return finder;
    }

    std::unique_ptr<IntersectionSolver> IntersectionSolver::GetSolver(MeshBase& owner, const Math::Quantizer& quantizer,
                                                                      const Primitives::BBox3D& intersectionBBox)
    {
//        IntersectionWorkspaceFactory::UniquePtr workspace(IntersectionWorkspaceFactory::GetInstance());

        return (std::unique_ptr<IntersectionSolver>(
            new IntersectionSolverImpl(owner, quantizer, intersectionBBox /*, workspace */)));
    }
}  // namespace Cork::Intersection
