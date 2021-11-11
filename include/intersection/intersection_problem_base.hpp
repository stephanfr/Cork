// +-------------------------------------------------------------------------
// | intersection_workspace_base.hpp
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

#include "tbb/tbb.h"

#include "intersection/intersection_workspace.hpp"
#include "perturbation_epsilon.hpp"

namespace Cork::Intersection
{
    
    inline Primitives::Vector3D computeCoords(const Meshes::TopoEdge& e, const Meshes::TopoTri& t, const Math::Quantizer& quantizer)
    {
        Math::ExteriorCalculusR4::GMPExt4_2 edgeCoordinates(e.edgeExactCoordinates(quantizer));
        Math::ExteriorCalculusR4::GMPExt4_3 triangleCoordinates(t.triangleExactCoordinates(quantizer));

        return (Empty3d::coordsExact(edgeCoordinates, triangleCoordinates, quantizer));
    }

    inline Primitives::Vector3D computeCoords(const Meshes::TopoTri& t0, const Meshes::TopoTri& t1, const Meshes::TopoTri& t2,
                                              const Math::Quantizer& quantizer)
    {
        return (Empty3d::coordsExact(t0.triangleExactCoordinates(quantizer), t1.triangleExactCoordinates(quantizer),
                                     t2.triangleExactCoordinates(quantizer), quantizer));
    }

    class IntersectionProblemBase
    {
        protected :

        using TopoCache = Meshes::TopoCache;
        using TopoVert = Meshes::TopoVert;
        using TopoEdge = Meshes::TopoEdge;
        using TopoTri = Meshes::TopoTri;
        using TopoEdgeReferenceVector = Meshes::TopoEdgeReferenceVector;
        using MeshBase = Meshes::MeshBase;

       public:
        class TriAndEdgeQueueMessage
        {
           public:
            enum class MessageType
            {
                TRI_AND_INTERSECTING_EDGES,
                END_OF_MESSAGES
            };

            virtual ~TriAndEdgeQueueMessage() = default;

            virtual MessageType type() const = 0;
        };

        class TriAndEdgeQueueEnd : public TriAndEdgeQueueMessage
        {
           public:
            MessageType type() const final { return (MessageType::END_OF_MESSAGES); }
        };

        class TriangleAndIntersectingEdgesMessage : public TriAndEdgeQueueMessage
        {
           public:
            TriangleAndIntersectingEdgesMessage() = delete;

            TriangleAndIntersectingEdgesMessage(const TriangleAndIntersectingEdgesMessage&) = delete;

            TriangleAndIntersectingEdgesMessage(TopoTri& tri, TopoEdgeReferenceVector& edges)
                : m_triangle(tri), m_edges(edges)
            {
            }

            ~TriangleAndIntersectingEdgesMessage() = default;

            TriangleAndIntersectingEdgesMessage& operator=(const TriangleAndIntersectingEdgesMessage&) = delete;

            MessageType type() const final { return (MessageType::TRI_AND_INTERSECTING_EDGES); }

            TopoTri& triangle() { return (m_triangle); }

            TopoEdgeReferenceVector& edges() { return (m_edges); }

           private:
            TopoTri& m_triangle;
            TopoEdgeReferenceVector m_edges;
        };

        using TriangleAndIntersectingEdgesQueue = tbb::concurrent_bounded_queue<TriAndEdgeQueueMessage*>;

        IntersectionProblemBase(MeshBase& owner_mesh, const Math::Quantizer& quantizer);

        IntersectionProblemBase(const IntersectionProblemBase& isctProblemToCopy) = delete;

        IntersectionProblemBase& operator=(const IntersectionProblemBase&) = delete;

        virtual ~IntersectionProblemBase() {}

        IntersectionWorkspace& workspace() { return *(workspace_.get()); }

        MeshBase& owner_mesh() { return owner_mesh_; }

        const MeshBase& owner_mesh() const { return owner_mesh_; }

        TopoCache&      topo_cache() { return topo_cache_; }

        IsctVertType* newIsctVert(const TopoEdge& e, const TopoTri& t, bool boundary, GluePointMarker& glue)
        {
            return (m_isctVertTypeList.emplace_back(GenericVertType::VertexType::INTERSECTION,
                                                    computeCoords(e, t, quantizer_), boundary, glue));
        }

        IsctVertType* newIsctVert(const TopoTri& t0, const TopoTri& t1, const TopoTri& t2, bool boundary,
                                  GluePointMarker& glue)
        {
            return (m_isctVertTypeList.emplace_back(GenericVertType::VertexType::INTERSECTION,
                                                    computeCoords(t0, t1, t2, quantizer_), boundary, glue));
        }

        IsctVertType* newSplitIsctVert(const Primitives::Vector3D& coords, GluePointMarker& glue)
        {
            return (m_isctVertTypeList.emplace_back(GenericVertType::VertexType::INTERSECTION, coords, false, glue));
        }

        IsctVertType* copyIsctVert(IsctVertType* orig)
        {
            return (m_isctVertTypeList.emplace_back(GenericVertType::VertexType::INTERSECTION, orig->coordinate(),
                                                    orig->is_boundary(), orig->glueMarker()));
        }

        IsctEdgeType* newIsctEdge(IsctVertType* endpoint, const TopoTri& tri_key)
        {
            return (m_isctEdgeTypeList.emplace_back(GenericEdgeType::EdgeType::INTERSECTION, false, endpoint, tri_key));
        }

        OrigVertType* newOrigVert(TopoVert* v)
        {
            return (
                m_origVertTypeList.emplace_back(GenericVertType::VertexType::ORIGINAL, *v, v->quantizedValue(), true));
        }

        OrigEdgeType* newOrigEdge(const TopoEdge& e, OrigVertType* v0, OrigVertType* v1)
        {
            return (m_origEdgeTypeList.emplace_back(GenericEdgeType::EdgeType::ORIGINAL, e, true, v0, v1));
        }

        SplitEdgeType* newSplitEdge(GenericVertType* v0, GenericVertType* v1, bool boundary)
        {
            return (m_splitEdgeTypeList.emplace_back(GenericEdgeType::EdgeType::SPLIT, boundary, v0, v1));
        }

        GenericTriType* newGenericTri(GenericVertType* v0, GenericVertType* v1, GenericVertType* v2)
        {
            return (m_genericTriTypeList.emplace_back(v0, v1, v2));
        }

        void perturbPositions();

        void CreateBoundingVolumeHierarchy();

        void FindEdgeAndTriangleIntersections(AABVH::IntersectionType selfOrBooleanIntersection,
                                              TriangleAndIntersectingEdgesQueue& triangleAndEdges);

        void createRealPtFromGluePt(GluePointMarker& glue)
        {
            assert(glue.vertices_to_be_glued().size() > 0);

            TopoVert* v = topo_cache_.newVert();

            topo_cache_.ownerMesh().vertices()[v->index()] = glue.vertices_to_be_glued()[0]->coordinate();

            for (IsctVertType* iv : glue.vertices_to_be_glued())
            {
                iv->set_concrete_vertex(*v);
            }
        }

        void releaseEdge(GenericEdgeType* ge)
        {
            ge->disconnect();

            IsctEdgeType* ie = dynamic_cast<IsctEdgeType*>(ge);

            switch (ge->edgeType())
            {
                case GenericEdgeType::EdgeType::INTERSECTION:
                    m_isctEdgeTypeList.free(ie);
                    break;

                case GenericEdgeType::EdgeType::ORIGINAL:
                    m_origEdgeTypeList.free(ie);
                    break;

                case GenericEdgeType::EdgeType::SPLIT:
                    m_splitEdgeTypeList.free(ie);
                    break;
            }
        }

        void killIsctVert(IsctVertType* iv)
        {
            iv->removeFromGlueMarkerCopies();

            if (iv->glueMarker().vertices_to_be_glued().size() == 0)
            {
                m_gluePointMarkerList.free(iv->glueMarker());
            }

            for (GenericEdgeType* ge : iv->edges())
            {
                // disconnect
                ge->interior().erase(std::find(ge->interior().begin(), ge->interior().end(), iv));

                if (ge->ends()[0] == iv)
                {
                    ge->ends()[0] = nullptr;
                }

                if (ge->ends()[1] == iv)
                {
                    ge->ends()[1] = nullptr;
                }
            }

            m_isctVertTypeList.free(iv);
        }

        void killIsctEdge(IsctEdgeType* ie)
        {
            // an endpoint may be an original vertex

            if (ie->ends()[1])
            {
                ie->ends()[1]->edges().erase(
                    std::find(ie->ends()[1]->edges().begin(), ie->ends()[1]->edges().end(), ie));
            }

            m_isctEdgeTypeList.free(ie);
        }

        void killOrigVert(OrigVertType* ov) { m_origVertTypeList.free(ov); }

        void killOrigEdge(OrigEdgeType* oe) { m_origEdgeTypeList.free(oe); }

        bool checkIsct(const TopoTri& t0, const TopoTri& t1, const TopoTri& t2)
        {
            // This function should only be called if we've already
            // identified that the intersection edges
            //      (t0,t1), (t0,t2), (t1,t2)
            // exist.
            //
            // From this, we can conclude that each pair of triangles
            // shares no more than a single vertex in common.
            //
            // If each of these shared vertices is different from each other,
            // then we could legitimately have a triple intersection point,
            // but if all three pairs share the same vertex in common, then
            // the intersection of the three triangles must be that vertex.
            // So, we must check for such a single vertex in common amongst
            // the three triangles

            TopoVert* common;

            if (t0.findCommonVertex(t1, common))
            {
                for (uint i = 0; i < 3; i++)
                {
                    if (common == t2.verts()[i])
                    {
                        return (false);
                    }
                }
            }

            //	Visual Studio's IDE gripes about the following when the explicit operator call is omitted

            Empty3d::TriangleTriangleTriangleIntersection input(t0.operator Empty3d::IntersectingTriangle(), t1.operator Empty3d::IntersectingTriangle(),
                                       t2.operator Empty3d::IntersectingTriangle());

            return input.emptyExact(quantizer_, exact_arithmetic_context_) != Empty3d::HasIntersection::YES;
        }

        void fillOutTriData(const TopoTri& piece, const TopoTri& parent)
        {
            topo_cache_.ownerMesh().triangles()[piece.ref()].boolAlgData() =
                topo_cache_.ownerMesh().triangles()[parent.ref()].boolAlgData();
        }

        std::unique_ptr<std::vector<Primitives::Vector3D>> dumpIsctPoints();

       protected:
        SEFUtility::CachingFactory<IntersectionWorkspace>::UniquePtr workspace_;

        MeshBase& owner_mesh_;

        Meshes::TopoCache   topo_cache_;

        std::unique_ptr<AABVH::AxisAlignedBoundingVolumeHierarchy> edge_bvh_;

        Empty3d::ExactArithmeticContext exact_arithmetic_context_;

        //	Quantizer must be in front of the Perturbation as the perturbation initialization depends on the quantizer

        Math::Quantizer quantizer_;
        PerturbationEpsilon perturbation_;

        GluePointMarkerList m_gluePointMarkerList;
        IsctVertTypeList m_isctVertTypeList;
        OrigVertTypeList m_origVertTypeList;
        IsctEdgeTypeList m_isctEdgeTypeList;
        OrigEdgeTypeList m_origEdgeTypeList;
        SplitEdgeTypeList m_splitEdgeTypeList;
        GenericTriTypeList m_genericTriTypeList;
    };
}  // namespace Cork::Intersection