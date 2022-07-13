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

#include "intersection_workspace.hpp"
#include "perturbation_epsilon.hpp"
#include "mesh/mesh_base.hpp"

namespace Cork::Intersection
{
    
    inline Primitives::Vector3D computeCoords(const Meshes::TopoEdge& e, const Meshes::TopoTri& t, const Math::Quantizer& quantizer)
    {
        Math::ExteriorCalculusR4::GMPExt4_2 edgeCoordinates(e.edge_exact_coordinates(quantizer));
        Math::ExteriorCalculusR4::GMPExt4_3 triangleCoordinates(t.triangle_exact_coordinates(quantizer));

        return (Empty3d::coordsExact(edgeCoordinates, triangleCoordinates, quantizer));
    }

    inline Primitives::Vector3D computeCoords(const Meshes::TopoTri& t0, const Meshes::TopoTri& t1, const Meshes::TopoTri& t2,
                                              const Math::Quantizer& quantizer)
    {
        return (Empty3d::coordsExact(t0.triangle_exact_coordinates(quantizer), t1.triangle_exact_coordinates(quantizer),
                                     t2.triangle_exact_coordinates(quantizer), quantizer));
    }


    class TriTripleTemp
    {
       public:

       TriTripleTemp() = delete;
       TriTripleTemp( const TriTripleTemp& ) = delete;
       TriTripleTemp( TriTripleTemp&& ) = default;

        TriTripleTemp(const Meshes::TopoTri& tp0, const Meshes::TopoTri& tp1, const Meshes::TopoTri& tp2) : t0_(tp0), t1_(tp1), t2_(tp2) {}

        TriTripleTemp&      operator=( const TriTripleTemp& ) = delete;
        TriTripleTemp&      operator=( TriTripleTemp&& ) = delete;

        const Meshes::TopoTri& t0() const { return t0_; }
        const Meshes::TopoTri& t1() const { return t0_; };
        const Meshes::TopoTri& t2() const { return t0_; };

        private :

        const Meshes::TopoTri& t0_;
        const Meshes::TopoTri& t1_;
        const Meshes::TopoTri& t2_;
    };


    class IntersectionProblemBase
    {
        protected :

        using TopoVert = Meshes::TopoVert;
        using TopoEdge = Meshes::TopoEdge;
        using TopoTri = Meshes::TopoTri;
        using TopoEdgeReferenceVector = Meshes::TopoEdgeReferenceVector;
        using MeshBaseImpl = Meshes::MeshBase;
        using MeshTopoCache = Meshes::MeshTopoCache;

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

        IntersectionProblemBase(MeshBaseImpl& owner_mesh, const Math::Quantizer& quantizer, const SolverControlBlock& solver_control_block);

        IntersectionProblemBase(const IntersectionProblemBase& isctProblemToCopy) = delete;

        IntersectionProblemBase& operator=(const IntersectionProblemBase&) = delete;

        virtual ~IntersectionProblemBase() {}

        IntersectionWorkspace& workspace() { return *(workspace_.get()); }

        MeshBaseImpl& owner_mesh() { return owner_mesh_; }

        const MeshBaseImpl& owner_mesh() const { return owner_mesh_; }

        MeshTopoCache&      topo_cache() { return owner_mesh_.topo_cache(); }

        IsctVertType* newIsctVert(const TopoEdge& e, const TopoTri& t, bool boundary, GluePointMarker& glue)
        {
            return (isct_vert_type_list_.emplace_back(GenericVertType::VertexType::INTERSECTION,
                                                    computeCoords(e, t, quantizer_), boundary, glue));
        }

        IsctVertType* newIsctVert(const TopoTri& t0, const TopoTri& t1, const TopoTri& t2, bool boundary,
                                  GluePointMarker& glue)
        {
            return (isct_vert_type_list_.emplace_back(GenericVertType::VertexType::INTERSECTION,
                                                    computeCoords(t0, t1, t2, quantizer_), boundary, glue));
        }

        IsctVertType* newSplitIsctVert(const Primitives::Vector3D& coords, GluePointMarker& glue)
        {
            return (isct_vert_type_list_.emplace_back(GenericVertType::VertexType::INTERSECTION, coords, false, glue));
        }

        IsctVertType* copyIsctVert(IsctVertType* orig)
        {
            return (isct_vert_type_list_.emplace_back(GenericVertType::VertexType::INTERSECTION, orig->coordinate(),
                                                    orig->is_boundary(), orig->glueMarker()));
        }

        IsctEdgeType* newIsctEdge(IsctVertType* endpoint, const TopoTri& tri_key)
        {
            return (isct_edge_type_list_.emplace_back(GenericEdgeType::EdgeType::INTERSECTION, false, endpoint, tri_key));
        }

        OrigVertType* newOrigVert(TopoVert* v)
        {
            return (
                orig_vert_type_list_.emplace_back(GenericVertType::VertexType::ORIGINAL, *v, v->quantized_value(), true));
        }

        OrigEdgeType* newOrigEdge(const TopoEdge& e, OrigVertType* v0, OrigVertType* v1)
        {
            return (orig_edge_type_list_.emplace_back(GenericEdgeType::EdgeType::ORIGINAL, e, true, v0, v1));
        }

        SplitEdgeType* newSplitEdge(GenericVertType* v0, GenericVertType* v1, bool boundary)
        {
            return (split_edge_type_list_.emplace_back(GenericEdgeType::EdgeType::SPLIT, boundary, v0, v1));
        }

        GenericTriType* newGenericTri(GenericVertType* v0, GenericVertType* v1, GenericVertType* v2)
        {
            return (generic_tri_type_list_.emplace_back(v0, v1, v2));
        }

        void perturbPositions();

        void CreateBoundingVolumeHierarchy();

        void FindEdgeAndTriangleIntersections(AABVH::IntersectionType selfOrBooleanIntersection,
                                              TriangleAndIntersectingEdgesQueue& triangleAndEdges);

        void createRealPtFromGluePt(GluePointMarker& glue)
        {
            assert(glue.vertices_to_be_glued().size() > 0);

            TopoVert* v = topo_cache().new_vertex();

            topo_cache().owner_mesh().vertices()[v->index()] = glue.vertices_to_be_glued()[0]->coordinate();

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
                    isct_edge_type_list_.free(ie);
                    break;

                case GenericEdgeType::EdgeType::ORIGINAL:
                    orig_edge_type_list_.free(ie);
                    break;

                case GenericEdgeType::EdgeType::SPLIT:
                    split_edge_type_list_.free(ie);
                    break;
            }
        }

        void killIsctVert(IsctVertType* iv)
        {
            iv->removeFromGlueMarkerCopies();

            if (iv->glueMarker().vertices_to_be_glued().size() == 0)
            {
                glue_point_marker_list_.free(iv->glueMarker());
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

            isct_vert_type_list_.free(iv);
        }

        void killIsctEdge(IsctEdgeType* ie)
        {
            // an endpoint may be an original vertex

            if (ie->ends()[1])
            {
                ie->ends()[1]->edges().erase(
                    std::find(ie->ends()[1]->edges().begin(), ie->ends()[1]->edges().end(), ie));
            }

            isct_edge_type_list_.free(ie);
        }

        void killOrigVert(OrigVertType* ov) { orig_vert_type_list_.free(ov); }

        void killOrigEdge(OrigEdgeType* oe) { orig_edge_type_list_.free(oe); }

        bool checkIsct(const TriTripleTemp& triangles)
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

            if (triangles.t0().find_common_vertex(triangles.t1(), common))
            {
                for (uint i = 0; i < 3; i++)
                {
                    if (common == triangles.t2().verts()[i])
                    {
                        return (false);
                    }
                }
            }

            //	Visual Studio's IDE gripes about the following when the explicit operator call is omitted

            Empty3d::TriangleTriangleTriangleIntersection input(triangles.t0().operator Empty3d::IntersectingTriangle(), triangles.t1().operator Empty3d::IntersectingTriangle(),
                                       triangles.t2().operator Empty3d::IntersectingTriangle());

            return input.emptyExact(quantizer_, exact_arithmetic_context_) != Empty3d::HasIntersection::YES;
        }

        void fillOutTriData(const TopoTri& piece, const TopoTri& parent)
        {
            owner_mesh_.triangles()[piece.ref()].set_bool_alg_data(
                owner_mesh_.triangles()[parent.ref()].bool_alg_data() );
        }

        std::unique_ptr<std::vector<Primitives::Vector3D>> dumpIsctPoints();

       protected:
        SEFUtility::CachingFactory<IntersectionWorkspace>::UniquePtr workspace_;

        const SolverControlBlock& solver_control_block_;

        MeshBaseImpl& owner_mesh_;

        std::unique_ptr<AABVH::AxisAlignedBoundingVolumeHierarchy> edge_bvh_;

        Empty3d::ExactArithmeticContext exact_arithmetic_context_;

        //	Quantizer must be in front of the Perturbation as the perturbation initialization depends on the quantizer

        Math::Quantizer quantizer_;
        PerturbationEpsilon perturbation_;

        GluePointMarkerList glue_point_marker_list_;
        IsctVertTypeList isct_vert_type_list_;
        OrigVertTypeList orig_vert_type_list_;
        IsctEdgeTypeList isct_edge_type_list_;
        OrigEdgeTypeList orig_edge_type_list_;
        SplitEdgeTypeList split_edge_type_list_;
        GenericTriTypeList generic_tri_type_list_;
    };
}  // namespace Cork::Intersection