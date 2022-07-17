// +-------------------------------------------------------------------------
// | triangle_problem.hpp
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
#pragma once

#include "intersection_problem_base.hpp"
#include "triangulator.hpp"


namespace Cork::Intersection
{
    class TriangleProblem : public IntrusiveListHook
    {
        using TopoVert = Meshes::TopoVert;
        using TopoEdge = Meshes::TopoEdge;
        using TopoTri = Meshes::TopoTri;

       public:

        TriangleProblem() = delete;

        TriangleProblem( const TriangleProblem& ) = delete;
        TriangleProblem( TriangleProblem&& ) = delete;

        TriangleProblem(IntersectionProblemBase& iprob, const TopoTri& triangle)
            : m_iprob(iprob),
              m_triangle(triangle),
              m_iverts(iprob.workspace().getIsctVertexPointerListPool()),
              m_iedges(iprob.workspace().getIsctEdgePointerListPool()),
              m_gtris(iprob.workspace().getGenericTriPointerListPool())
        {
            //	m_triangle can be const... just about everywhere.  This link is the only non-const operation,
            //		so let's explicitly cast away the const here but leave it everytwhere else.

            const_cast<TopoTri&>(m_triangle).associate_triangle_problem(*this);

            // extract original edges/verts

            for (uint k = 0; k < 3; k++)
            {
                overts[k] = m_iprob.newOrigVert(m_triangle.verts()[k]);
            }

            for (uint k = 0; k < 3; k++)
            {
                oedges[k] = iprob.newOrigEdge(*(m_triangle.edges()[k]), overts[(k + 1) % 3], overts[(k + 2) % 3]);
            }
        }

        ~TriangleProblem() {}

        TriangleProblem&    operator=( const TriangleProblem& ) = delete;
        TriangleProblem&    operator=( TriangleProblem&& ) = delete;

        //	Accessors and Mutators

        const TopoTri& triangle() const { return (m_triangle); }

        void ResetTopoTriLink() { const_cast<TopoTri&>(m_triangle).clear_triangle_problem_association(); }

        const IntersectionEdgePointerList& iedges() const { return (m_iedges); }

        IntersectionEdgePointerList& iedges() { return (m_iedges); }

        const GenericTriPointerList& gtris() const { return (m_gtris); }

        GenericTriPointerList& gtris() { return (m_gtris); }

        // specify reference glue point and edge piercing this triangle.
        IsctVertType* addInteriorEndpoint(const TopoEdge& edge, GluePointMarker& glue)
        {
            IsctVertType* iv = m_iprob.newIsctVert(edge, m_triangle, false, glue);
            m_iverts.push_back(iv);

            for (auto tri_key : edge.triangles())
            {
                addEdge(iv, *tri_key);
            }

            return (iv);
        }

        // specify the other triangle cutting this one, the edge cut,
        // and the resulting point of intersection

        void addBoundaryEndpoint(const TopoTri& tri_key, const TopoEdge& edge, IsctVertType* iv)
        {
            iv = m_iprob.copyIsctVert(iv);
            addBoundaryHelper(edge, iv);

            // handle edge extending into interior

            addEdge(iv, tri_key);
        }

        IsctVertType* addBoundaryEndpoint(const TopoTri& tri_key, const TopoEdge& edge,
                                          const Primitives::Vector3D& coord, GluePointMarker& glue)
        {
            IsctVertType* iv = m_iprob.newSplitIsctVert(coord, glue);
            addBoundaryHelper(edge, iv);

            // handle edge extending into interior

            addEdge(iv, tri_key);

            return (iv);
        }

        // Should only happen for manually inserted split points on
        // edges, not for points computed via intersection...

        IsctVertType* addBoundaryPointAlone(const TopoEdge& edge, const Primitives::Vector3D& coord,
                                            GluePointMarker& glue)
        {
            IsctVertType* iv = m_iprob.newSplitIsctVert(coord, glue);
            addBoundaryHelper(edge, iv);

            return (iv);
        }

        void addInteriorPoint(const TopoTri& t0, const TopoTri& t1, GluePointMarker& glue)
        {
            // note this generates wasted re-computation of coordinates 3X

            IsctVertType* iv = m_iprob.newIsctVert(m_triangle, t0, t1, false, glue);
            m_iverts.push_back(iv);

            // find the 2 interior edges

            for (IsctEdgeType* ie : m_iedges)
            {
                if ((&(ie->otherTriKey()) == &t0) || (&(ie->otherTriKey()) == &t1))
                {
                    ie->interior().push_back(iv);
                    iv->edges().push_back(ie);
                }
            }
        }

        // run after we've accumulated all the elements

        typedef SEFUtility::Result<ConsolidateResultCodes> ConsolidateResult;

        ConsolidateResult Consolidate()
        {
            // identify all intersection edges missing endpoints
            // and check to see if we can assign an original vertex
            // as the appropriate endpoint.

            for (IsctEdgeType* ie : m_iedges)
            {
                if (ie->ends()[1] == nullptr)
                {
                    // try to figure out which vertex must be the endpoint...

                    TopoVert* vert;

                    if (!m_triangle.find_common_vertex(ie->otherTriKey(), vert))
                    {
#ifdef _DEBUG

                        std::cout << "the  edge is " << ie->ends()[0] << ",  " << ie->ends()[1] << std::endl;

                        IsctVertType* iv = dynamic_cast<IsctVertType*>(ie->ends()[0]);

                        std::cout << "   " << iv->glueMarker().edgeTriType() << std::endl;
                        std::cout << "the   tri is " << m_triangle << ": " << m_triangle << std::endl;
                        std::cout << "other tri is " << &(ie->otherTriKey().value()) << ": "
                                  << ie->otherTriKey().value() << std::endl;
                        std::cout << "coordinates for triangles" << std::endl;
                        std::cout << "the tri" << std::endl;

                        for (uint k = 0; k < 3; k++)
                        {
                            std::cout << *(m_triangle.verts()[k]->quantizedValue()) << std::endl;
                        }

                        for (uint k = 0; k < 3; k++)
                        {
                            std::cout << *(ie->otherTriKey().value().verts()[k]->quantizedValue()) << std::endl;
                        }

                        std::cout << "degen count:" << m_iprob.ExactArithmeticContext().degeneracy_count << std::endl;
                        std::cout << "exact count: " << m_iprob.ExactArithmeticContext().exact_count << std::endl;
#endif

                        return (
                            ConsolidateResult::failure(ConsolidateResultCodes::COULD_NOT_FIND_COMMON_VERTEX,
                                                       "Could not find common vertex in Triangle Problem Consolidate"));
                    }

                    // then, find the corresponding OrigVertType*, and connect

                    for (uint k = 0; k < 3; k++)
                    {
                        if (&(overts[k]->concrete_vertex()) == vert)
                        {
                            ie->ends()[1] = overts[k];
                            overts[k]->edges().push_back(ie);
                            break;
                        }
                    }
                }
            }

            return (ConsolidateResult::success());
        }

        typedef SEFUtility::Result<SubdivideResultCodes> SubdivideResult;

        SubdivideResult Subdivide()
        {
            // collect all the points, and create more points as necessary

            std::vector<GenericVertType*> points;
            points.reserve(1024);

            for (uint k = 0; k < 3; k++)
            {
                points.push_back(overts[k]);
            }

            for (IsctVertType* iv : m_iverts)
            {
                points.push_back(iv);
            }

            for (uint i = 0; i < points.size(); i++)
            {
                points[i]->setIndex(i);
            }

            // split edges and marshall data
            // for safety, we zero out references to pre-subdivided edges,
            // which may have been destroyed

            VectorOfGenericEdgePointers edges;

            for (uint k = 0; k < 3; k++)
            {
                subdivideEdge(oedges[k], edges);
                oedges[k] = nullptr;
            }

            for (IsctEdgeType* ie : m_iedges)
            {
                subdivideEdge(ie, edges);
                ie = nullptr;
            }

            Triangulator::Triangulator triangulator;

            if (auto result = triangulator.will_problem_fit(points.size(), edges.size());
                result != TriangulationResultCodes::SUCCESS)
            {
                return (SubdivideResult::failure(
                    Triangulator::TriangulateResult::failure(result, "Too many points or segments for triangulation"),
                    SubdivideResultCodes::FAILED_TRIANGULATION, "Failed Triangulation"));
            }

            Math::NormalProjector normal_projector(overts[0]->coordinate(), overts[1]->coordinate(),
                                                           overts[2]->coordinate());

            for (auto& point : points)
            {
                triangulator.add_point(point->coordinate(), point->is_boundary(), normal_projector);
            }

            for (auto& edge : edges)
            {
                triangulator.add_segment(edge->ends()[0]->index(), edge->ends()[1]->index(), edge->is_boundary());
            }

            auto result = triangulator.compute_triangulation();

            if (result.failed())
            {
                //	When we end up here, it is usually because we have hit some self-intersections.

                return (SubdivideResult::failure(result, SubdivideResultCodes::FAILED_TRIANGULATION,
                                                 "Failed Triangulation"));
            }

            m_gtris.clear();

            for (auto triangle : *(result.return_ptr()))
            {
                GenericVertType* gv0 = points[triangle.v0()];
                GenericVertType* gv1 = points[triangle.v1()];
                GenericVertType* gv2 = points[triangle.v2()];

                m_gtris.push_back(m_iprob.newGenericTri(gv0, gv1, gv2));
            }

            return (SubdivideResult::success());
        }

       private:
        typedef std::vector<GenericEdgeType*> VectorOfGenericEdgePointers;

        IntersectionProblemBase& m_iprob;
        const TopoTri& m_triangle;

        IntersectionVertexPointerList m_iverts;
        IntersectionEdgePointerList m_iedges;

        //	Original triangle elements

        std::array<OrigVertType*, 3> overts;
        std::array<OrigEdgeType*, 3> oedges;

        GenericTriPointerList m_gtris;

        // may actually not add edge, but instead just hook up endpoint

        void addEdge(IsctVertType* iv, const TopoTri& tri_key)
        {
            IsctEdgeType* ie = nullptr;

            for (IsctEdgeType* currentIntersectionEdge : m_iedges)
            {
                if (&(currentIntersectionEdge->otherTriKey()) == &tri_key)
                {
                    ie = currentIntersectionEdge;
                    break;
                }
            }

            if (ie)
            {  // if the edge is already present
                ie->ends()[1] = iv;
                iv->edges().push_back(ie);
            }
            else
            {  // if the edge is being added
                ie = m_iprob.newIsctEdge(iv, tri_key);
                m_iedges.push_back(ie);
            }
        }

        void addBoundaryHelper(const TopoEdge& edge, IsctVertType* iv)
        {
            iv->set_boundary(true);
            m_iverts.push_back(iv);

            // hook up point to boundary edge interior!

            for (uint k = 0; k < 3; k++)
            {
                OrigEdgeType* oe = oedges[k];

                if (&(oe->concrete()) == &edge)
                {
                    oe->interior().push_back(iv);
                    iv->edges().push_back(oe);
                    break;
                }
            }
        }

        void subdivideEdge(GenericEdgeType* ge, VectorOfGenericEdgePointers& edges)
        {
            if (ge->interior().size() == 0)
            {
                edges.push_back(ge);
            }
            else if (ge->interior().size() == 1)
            {  // common case
                SplitEdgeType* se0 = m_iprob.newSplitEdge(ge->ends()[0], ge->interior()[0], ge->is_boundary());

                SplitEdgeType* se1 = m_iprob.newSplitEdge(ge->interior()[0], ge->ends()[1], ge->is_boundary());

                edges.push_back(se0);
                edges.push_back(se1);

                // get rid of old edge

                m_iprob.releaseEdge(ge);
            }
            else
            {
                // sorting is the uncommon case
                // determine the primary dimension and direction of the edge

                Primitives::Vector3D dir = ge->ends()[1]->coordinate() - ge->ends()[0]->coordinate();
                uint dim = (fabs(dir.x()) > fabs(dir.y())) ? ((fabs(dir.x()) > fabs(dir.z())) ? 0 : 2)
                                                           : ((fabs(dir.y()) > fabs(dir.z())) ? 1 : 2);
                double sign = (dir[dim] > 0.0) ? 1.0 : -1.0;

                // pack the interior vertices into a vector for sorting

                std::vector<std::pair<double, IsctVertType*>> verts;
                verts.reserve(ge->interior().size());

                for (IsctVertType* iv : ge->interior())
                {
                    // if the sort is ascending, then we're good...

                    verts.emplace_back(std::make_pair(sign * iv->coordinate()[dim], iv));
                }

                // ... and sort the vector

                std::sort(verts.begin(), verts.end());

                // then, write the verts into a new container with the endpoints

                std::vector<GenericVertType*> allv(verts.size() + 2);

                allv[0] = ge->ends()[0];
                allv[allv.size() - 1] = ge->ends()[1];

                for (uint k = 0; k < verts.size(); k++)
                {
                    allv[k + 1] = verts[k].second;
                }

                // now create and accumulate new split edges

                for (uint i = 1; i < allv.size(); i++)
                {
                    SplitEdgeType* se = m_iprob.newSplitEdge(allv[i - 1], allv[i], ge->is_boundary());
                    edges.push_back(se);
                }

                // get rid of old edge

                m_iprob.releaseEdge(ge);
            }
        }
    };

    typedef tbb::concurrent_vector<TriangleProblem> TriangleProblemList;
}  // namespace Cork::Intersection