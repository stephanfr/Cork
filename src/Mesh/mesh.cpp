
// +-------------------------------------------------------------------------
// | Mesh.cpp
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

#include "mesh/mesh.hpp"

#include <boost/timer/timer.hpp>
#include <sstream>

#include "intersection/intersection_problem.hpp"
#include "intersection/unsafe_ray_triangle_intersection.hpp"
#include "mesh/topo_cache.hpp"
#include "mesh/triangle_mesh.h"
#include "tbb/parallel_for.h"
#include "util/thread_pool.hpp"
#include "util/union_find.hpp"

namespace Cork::Meshes
{
    using IntersectionSolver = Intersection::IntersectionSolver;
    using IntersectionProblemResult = Intersection::IntersectionProblemResult;
    using IntersectionProblemResultCodes = Intersection::IntersectionProblemResultCodes;

    using IncrementalVertexIndexTriangleMeshBuilder = Meshes::IncrementalVertexIndexTriangleMeshBuilder;

    inline double tri_area(const Vector3D& a, const Vector3D& b, const Vector3D& c)
    {
        return ((b - a).cross(c - a).len());
    }

    inline double tri_area_squared(const Vector3D& a, const Vector3D& b, const Vector3D& c)
    {
        return ((b - a).cross(c - a).len_squared());
    }

    inline void Mesh::for_ecache(EGraphCache& ecache, std::function<void(const EGraphEntryTIDVector& tids)> action,
                                 int numThreads) const
    {
        //		ThreadPool::getPool().parallel_for(numThreads, ecache.columns().begin(), ecache.columns().end(),
        //[&](BlockRange<EGraphCache::SkeletonColumnVector::iterator>	partitionedCloumns )
        //		tbb::parallel_for(tbb::blocked_range<EGraphCache::SkeletonColumnVector::iterator>(ecache.columns().begin(),
        // ecache.columns().end(), ecache.columns().size() / 4 ),
        //			[&](tbb::blocked_range<EGraphCache::SkeletonColumnVector::iterator> partitionedColumns)
        {
            for (auto& column : ecache.columns())
            {
                column.for_each([this, &action](EGraphEntry& entry) {
                    if (entry.intersects())
                    {
                        EGraphEntryTIDVector tid0s;
                        EGraphEntryTIDVector tid1s;

                        for (TriangleByIndicesIndex tid : entry.tids())
                        {
                            if (tris_[tid].bool_alg_data() & 1)
                            {
                                tid1s.push_back(tid);
                            }
                            else
                            {
                                tid0s.push_back(tid);
                            }
                        }

                        action(tid1s);
                        action(tid0s);
                    }
                    else
                    {
                        action(entry.tids());
                    }
                });
            }
        }
    }

    inline bool Mesh::isInside(TriangleByIndicesIndex tid, uint32_t operand)
    {
        // find the point to trace outward from...

        Vector3D p(verts_[tris_[tid].a()]);

        p += verts_[tris_[tid].b()];
        p += verts_[tris_[tid].c()];
        p /= 3.0;  //  NOLINT(cppcoreguidelines-avoid-magic-numbers)

        // ok, we've got the point, now let's pick a direction

        Ray3DWithInverseDirection directionRay(
            p, Vector3D::randomVector(0.5, 1.5));  //  NOLINT(cppcoreguidelines-avoid-magic-numbers)

        long winding = 0;

        //	Pass all triangles over ray
        //		Check for intersections with the triangle's bounding box to cull before the more
        //		expensive ray triangle intersection test.

        for (auto& tri : tris_)
        {
            // ignore triangles from the same operand surface

            if ((tri.bool_alg_data() & 1) == operand)
            {
                continue;
            }

            //	Check the bounding box intersection first

            BBox3D boundingBox(verts_[tri.a()].min(verts_[tri.b()], verts_[tri.c()]),
                               verts_[tri.a()].max(verts_[tri.b()], verts_[tri.c()]));

            if (!boundingBox.intersects(directionRay))
            {
                continue;
            }

            //	OK, we may have a hit so check for ray triangle intersection

            RayTriangleIntersection(tri, directionRay, winding);
        }

        // now, we've got a winding number to work with...
        return (winding > 0);
    }

    inline void Mesh::RayTriangleIntersection(const TriangleByIndices& tri, Ray3D& r, long& winding)
    {
        NUMERIC_PRECISION flip = 1.0;

        VertexIndex a = tri.a();
        VertexIndex b = tri.b();
        VertexIndex c = tri.c();

        Vector3D va = verts_[a];
        Vector3D vb = verts_[b];
        Vector3D vc = verts_[c];

        // normalize vertex order (to prevent leaks)

        if (a > b)
        {
            std::swap(a, b);
            std::swap(va, vb);
            flip = -flip;
        }

        if (b > c)
        {
            std::swap(b, c);
            std::swap(vb, vc);
            flip = -flip;
        }

        if (a > b)
        {
            std::swap(a, b);
            std::swap(va, vb);
            flip = -flip;
        }

        if (Intersection::CheckForRayTriangleIntersection(r, va, vb, vc))
        {
            Vector3D normal = flip * (vb - va).cross(vc - va);

            if (normal.dot(r.direction()) > 0.0)
            {
                winding++;
            }
            else
            {
                winding--;
            }
        }
    }

    //
    //	Constructors and assignment operators
    //

    Mesh::Mesh(const TriangleMesh& inputMesh, const SolverControlBlock& controlBlock) : MeshBase(controlBlock)
    {
        min_and_max_edge_lengths_ = inputMesh.min_and_max_edge_lengths();
        max_vertex_magnitude_ = inputMesh.max_vertex_magnitude();

        tris_.reserve(inputMesh.numTriangles());
        verts_.reserve(inputMesh.numVertices());

        //	Start by copying the vertices.

        for (auto currentVertex : inputMesh.vertices())
        {
            verts_.emplace_back(currentVertex.x(), currentVertex.y(), currentVertex.z());
        }

        //	Fill the triangles

        for (TriangleByIndicesIndex i = 0u; i < inputMesh.triangles().size(); i++)
        {
            tris_.emplace_back(inputMesh.triangles()[i], 0);
        }

        bounding_box_ = inputMesh.boundingBox();
    }

    Mesh::~Mesh() {}

    void Mesh::operator=(Mesh&& src)
    {
        tris_ = src.tris_;
        verts_ = src.verts_;
    }

    bool Mesh::valid() const
    {
        for (VertexIndex i = 0u; i < verts_.size(); i++)
        {
            if (!std::isfinite(verts_[i].x()) || !std::isfinite(verts_[i].y()) || !std::isfinite(verts_[i].z()))
            {
                std::ostringstream message;
                message << "vertex #" << i << " has non-finite coordinates: " << verts_[i];
                std::cerr << message.str();
                return false;
            }
        }

        for (TriangleByIndicesIndex i = 0U; i < tris_.size(); i++)
        {
            if (tris_[i].a() >= verts_.size() || tris_[i].b() >= verts_.size() || tris_[i].c() >= verts_.size())
            {
                std::ostringstream message;
                message << "triangle #" << i << " should have indices in "
                        << "the range 0 to " << (verts_.size() - 1) << ", but it has invalid indices: " << tris_[i].a()
                        << ", " << tris_[i].b() << ", " << tris_[i].c();
                std::cerr << message.str();
                return false;
            }
        }

        return (true);
    }

    void Mesh::DisjointUnion(const Mesh& meshToMerge)
    {
        //	Reset the labels on this mesh's collection of triangles

        for (auto& t : tris_)
        {
            t.set_bool_alg_data(0);
        }

        uint32_t oldVsize = verts_.size();
        uint32_t oldTsize = tris_.size();
        uint32_t cpVsize = meshToMerge.verts_.size();
        uint32_t cpTsize = meshToMerge.tris_.size();
        uint32_t newVsize = oldVsize + cpVsize;
        uint32_t newTsize = oldTsize + cpTsize;

        verts_.resize(newVsize);
        tris_.resize(newTsize);

        for (VertexIndex i = 0u; i < cpVsize; i++)
        {
            verts_[oldVsize + i] = meshToMerge.verts_[i];
        }

        for (TriangleByIndicesIndex i = 0U; i < cpTsize; i++)
        {
            auto& tri = tris_[oldTsize + i];

            tri = meshToMerge.tris_[i];
            tri.set_bool_alg_data(1);  //	These triangles are part of the RHS so label them as such
            tri.offset_indices(oldVsize);
        }
    }

    Mesh::SetupBooleanProblemResult Mesh::SetupBooleanProblem(const Mesh& rhs)
    {
        auto intersectionBBox = bounding_box_.intersection(rhs.bounding_box());

        //	Form the disjoint union of this mesh and the second operand mesh

        DisjointUnion(rhs);

        performance_stats_.set_number_of_triangles_in_disjoint_union((unsigned long)this->tris_.size());
        control_block_->set_num_triangles((unsigned long)this->tris_.size());

        if (this->tris_.size() >= MAX_TRIANGLES_IN_DISJOINT_UNION)
        {
            return (
                SetupBooleanProblemResult::failure(SetupBooleanProblemResultCodes::TOO_MANY_TRIANGLES_IN_DISJOINT_UNION,
                                                   "Too many triangles in disjoint union, possible out of memory "
                                                   "exception if the operation is attenmpted."));
        }

        //	Start by finding the intersections

        Math::Quantizer::GetQuantizerResult get_quantizer_result = quantizer();

        if (!get_quantizer_result.succeeded())
        {
            return (SetupBooleanProblemResult::failure(get_quantizer_result,
                                                       SetupBooleanProblemResultCodes::QUANTIZER_CREATION_FAILED,
                                                       "Failed to create quantizer"));
        }

        Math::Quantizer quantizer(get_quantizer_result.return_value());

        //	Find intersections and then resolve them.  We might have to repurturb if finding and resolving fails.
        //		We can repurturb until we run out of perturbation resolution.

        std::unique_ptr<IntersectionSolver> iproblem(IntersectionSolver::GetSolver(*this, quantizer));

        while (true)
        {
            IntersectionProblemResult findResult = iproblem->FindIntersections();

            if (!findResult.succeeded())
            {
                //	If we failed here - not mush to do but return a failed result

                return (SetupBooleanProblemResult::failure(findResult,
                                                           SetupBooleanProblemResultCodes::FIND_INTERSECTIONS_FAILED,
                                                           "FindIntersections failed."));
            }

            //	Next, resolve them

            IntersectionProblemResult resolveResult = iproblem->ResolveAllIntersections();

            if (!resolveResult.succeeded())
            {
                //	Resolve failed, check the error code to see if this is a recoverable error or not

                //	If we failed due to a self-intersection, then one of the meshes is bad so no amount of
                // repurturbation will work.

                if (resolveResult.error_code() == IntersectionProblemResultCodes::SELF_INTERSECTING_MESH)
                {
                    return SetupBooleanProblemResult::failure(resolveResult,
                                                              SetupBooleanProblemResultCodes::SELF_INTERSECTING_MESH,
                                                              "One of the two meshes self intersects");
                }

                //	Resolve failed for some other reason.

                return SetupBooleanProblemResult::failure(resolveResult,
                                                          SetupBooleanProblemResultCodes::RESOLVE_INTERSECTIONS_FAILED,
                                                          "ResolveIntersections failed and exhuasted perturbations.");
            }

            iproblem->commit();
            break;
        }

        //	Create and populate the EGraph Cache

        BuildEGraphCacheResult buildEGraphResult = BuildEdgeGraphCache();

        if (!buildEGraphResult.succeeded())
        {
            return (SetupBooleanProblemResult::failure(buildEGraphResult,
                                                       SetupBooleanProblemResultCodes::POPULATE_EDGE_GRAPH_CACHE_FAILED,
                                                       "Building Edge Graph Cache Failed"));
        }

        std::unique_ptr<EGraphCache> ecache(std::move(buildEGraphResult.return_ptr()));

        // form connected components;
        // we get one component for each connected component in one
        // of the two input meshes.
        // These components are not necessarily uniformly inside or outside
        // of the other operand mesh.

        std::unique_ptr<ComponentList> components(std::move(FindComponents(*ecache)));

        if (solver_control_block().use_multiple_threads() && (components->size() > 1))
        {
            size_t partitionSize = 1;

            if (components->size() > 8)
            {
                partitionSize = components->size() / 8;
            }

            tbb::parallel_for(
                tbb::blocked_range<ComponentList::iterator>(components->begin(), components->end(), partitionSize),
                [&](tbb::blocked_range<ComponentList::iterator> partitionedComponents) {
                    for (auto& comp : partitionedComponents)
                    {
                        ProcessComponent(*ecache, comp);
                    }
                },
                tbb::simple_partitioner());
        }
        else
        {
            for (auto& comp : *components)
            {
                ProcessComponent(*ecache, comp);
            }
        }

        //	Finished with Success

        return (SetupBooleanProblemResult::success());
    }

    Mesh::BuildEGraphCacheResult Mesh::BuildEdgeGraphCache() const
    {
        std::unique_ptr<EGraphCache> ecachePtr(new EGraphCache);

        try
        {
            ecachePtr->resize(verts_.size());
        }
        catch (std::bad_alloc& ex)
        {
            return (BuildEGraphCacheResult::failure(BuildEGraphCacheResultCodes::OUT_OF_MEMORY,
                                                    "Out of Memory resizing the edge cache"));
        }

        EGraphCache& ecache = *ecachePtr;

        for (TriangleByIndicesIndex tid = 0U; tid < tris_.size(); tid++)
        {
            const TriangleByIndices& tri = tris_[tid];

            ecache[VertexIndex::integer_type(tri.a())]
                .find_or_add(VertexIndex::integer_type(tri.b()))
                .tids()
                .push_back(tid);
            ecache[VertexIndex::integer_type(tri.a())]
                .find_or_add(VertexIndex::integer_type(tri.c()))
                .tids()
                .push_back(tid);

            ecache[VertexIndex::integer_type(tri.b())]
                .find_or_add(VertexIndex::integer_type(tri.a()))
                .tids()
                .push_back(tid);
            ecache[VertexIndex::integer_type(tri.b())]
                .find_or_add(VertexIndex::integer_type(tri.c()))
                .tids()
                .push_back(tid);

            ecache[VertexIndex::integer_type(tri.c())]
                .find_or_add(VertexIndex::integer_type(tri.a()))
                .tids()
                .push_back(tid);
            ecache[VertexIndex::integer_type(tri.c())]
                .find_or_add(VertexIndex::integer_type(tri.b()))
                .tids()
                .push_back(tid);
        }

        //	Label some of the edges as intersection edges and others as not

        for (auto& column : ecache.columns())
        {
            column.for_each([this](EGraphEntry& entry) {
                entry.set_intersects(false);
                uint32_t operand = tris_[entry.tids()[0]].bool_alg_data();

                for (uint k = 1; k < entry.tids().size(); k++)
                {
                    if (tris_[entry.tids()[k]].bool_alg_data() != operand)
                    {
                        entry.set_intersects(true);
                        break;
                    }
                }
            });
        }

        //	Finished with success

        return BuildEGraphCacheResult::success(std::move(ecachePtr));
    }

    std::unique_ptr<Mesh::ComponentList> Mesh::FindComponents(EGraphCache& ecache) const
    {
        // form connected components;
        // we get one component for each connected component in one
        // of the two input meshes.
        // These components are not necessarily uniformly inside or outside
        // of the other operand mesh.

        RandomWeightedParallelUnionFind uf(tris_.size());

        for_ecache(ecache, [&uf](const EGraphEntryTIDVector& tids) {
            TriangleByIndicesIndex tid0 = tids[0];

            for (size_t k = 1U; k < tids.size(); k++)
            {
                uf.unite(TriangleByIndicesIndex::integer_type(tid0), TriangleByIndicesIndex::integer_type(tids[k]));
            }
        });

        // we re-organize the results of the union find as follows:

        std::vector<long> uq_ids(tris_.size(), long(-1));
        std::unique_ptr<ComponentList> components(new ComponentList);

        components->reserve(256);

        std::mutex vectorLock;

        ThreadPool::getPool().parallel_for(4, (size_t)0, tris_.size(), [&](size_t blockBegin, size_t blockEnd) {
            size_t ufid;

            for (size_t i = blockBegin; i < blockEnd; i++)
            {
                ufid = uf.find(i);

            retry:

                if (uq_ids[ufid] == long(-1))
                {
                    std::lock_guard<std::mutex> lock(vectorLock);

                    if (uq_ids[ufid] != long(-1)) goto retry;

                    size_t N = components->size();
                    components->emplace_back();
                    //					(*components)[N].reserve(512);

                    uq_ids[ufid] = uq_ids[i] = (long)N;
                    (*components)[N].push_back(i);
                }
                else
                {
                    uq_ids[i] = uq_ids[ufid];
                    (*components)[uq_ids[i]].push_back(i);
                }
            }
        });

        return (components);
    }

    size_t Mesh::CountComponents() const
    {
        BuildEGraphCacheResult ecacheResult = BuildEdgeGraphCache();

        std::unique_ptr<EGraphCache> ecache(std::move(ecacheResult.return_ptr()));

        std::unique_ptr<ComponentList> components(std::move(FindComponents(*ecache)));

        std::vector<std::set<VertexIndex>> bodies;

        bodies.reserve(components->size());

        for (int i = 0; i < components->size(); i++)
        {
            ComponentType& component = (*components)[i];

            bodies.emplace_back();

            std::set<VertexIndex>& bodyByVerts = bodies.back();

            for (auto triIndex : component)
            {
                bodyByVerts.insert(tris_[TriangleByIndicesIndex(uint32_t(triIndex))].a());
                bodyByVerts.insert(tris_[TriangleByIndicesIndex(uint32_t(triIndex))].b());
                bodyByVerts.insert(tris_[TriangleByIndicesIndex(uint32_t(triIndex))].c());
            }
        }

        for (int i = 0; i < bodies.size(); i++)
        {
            for (int j = i + 1; j < bodies.size(); j++)
            {
                bool merged = false;

                std::set<VertexIndex>& body1 = bodies[i];
                std::set<VertexIndex>& body2 = bodies[j];

                for (auto element2 : body2)
                {
                    if (body1.find(element2) != body1.end())
                    {
                        body1.insert(body2.begin(), body2.end());

                        bodies.erase(bodies.begin() + j);

                        j--;

                        merged = true;
                        break;
                    }
                }

                if (merged)
                {
                    i--;
                    break;
                }
            }
        }

        return (bodies.size());
    }

    void Mesh::ProcessComponent(const EGraphCache& ecache, const ComponentType& trisInComponent)
    {
        // find the "best" triangle in each component,
        // and ray cast to determine inside-ness vs. outside-ness

        TriangleByIndicesIndex best_tid = FindTriForInsideTest(trisInComponent);

        //	Do the 'inside' test

        uint32_t operand = tris_[best_tid].bool_alg_data();
        bool inside = isInside(best_tid, operand);

        //	Do a breadth first propagation of classification throughout the component.

        std::vector<TriangleByIndicesIndex> work;
        work.reserve(trisInComponent.size());

        Primitives::BooleanVector<TriangleByIndicesIndex>      visited(tris_.size());

        // begin by tagging the first triangle

        tris_[best_tid].set_bool_alg_data(tris_[best_tid].bool_alg_data() | (inside) ? 2 : 0);
        visited[best_tid] = true;
        work.push_back(best_tid);

        while (!work.empty())
        {
            TriangleByIndicesIndex curr_tid = work.back();
            work.pop_back();

            for (size_t k = 0; k < 3; k++)
            {
                VertexIndex a = tris_[curr_tid][k];
                VertexIndex b = tris_[curr_tid][(k + 1) % 3];

                auto& entry = ecache[VertexIndex::integer_type(a)][VertexIndex::integer_type(b)];

                uint32_t inside_sig = tris_[curr_tid].bool_alg_data() & 2;

                if (entry.intersects())
                {
                    inside_sig ^= 2;
                }

                for (TriangleByIndicesIndex tid : entry.tids())
                {
                    if (visited[TriangleByIndicesIndex::integer_type(tid)])
                    {
                        continue;
                    }

                    if ((tris_[tid].bool_alg_data() & 1) != operand)
                    {
                        continue;
                    }

                    tris_[tid].set_bool_alg_data(tris_[tid].bool_alg_data() | inside_sig);
                    visited[TriangleByIndicesIndex::integer_type(tid)] = true;
                    work.push_back(tid);
                }
            }
        }
    }

    TriangleByIndicesIndex Mesh::FindTriForInsideTest(const ComponentType& trisInComponent)
    {
        TriangleByIndicesIndex current_tid{0U};
        TriangleByIndicesIndex best_tid = TriangleByIndicesIndex::integer_type( trisInComponent[0] );
        double best_area = 0.0;

        size_t searchIncrement = 1;

        //	We will adjust the search increment based on the number of triangles to search.

        if (trisInComponent.size() > 1000)
        {
            searchIncrement = 2;
        }

        if (trisInComponent.size() > 25000)
        {
            searchIncrement = 3;
        }

        if (trisInComponent.size() > 50000)
        {
            searchIncrement = 4;
        }

        if (trisInComponent.size() > 100000)
        {
            searchIncrement = 5;
        }

        //	Do the search - we are looking for the triangle with the greatest surface area to use
        //		as the representative triangle to test 'insideness' of component.

        for (size_t i = 0; i < trisInComponent.size(); i += searchIncrement)
        {
            current_tid = TriangleByIndicesIndex( TriangleByIndicesIndex::integer_type( trisInComponent[i] ));

            const Vector3D& va = verts_[tris_[current_tid].a()];
            const Vector3D& vb = verts_[tris_[current_tid].b()];
            const Vector3D& vc = verts_[tris_[current_tid].c()];

            double area =
                tri_area_squared(va, vb, vc);  //	We don't need the square root, we just want the biggest surface area

            if (area > best_area)
            {
                best_area = area;
                best_tid = current_tid;
            }
        }

        return (best_tid);
    }

    void Mesh::doDeleteAndFlip(std::function<TriCode(uint32_t bool_alg_data)> classify)
    {
        Math::Quantizer::GetQuantizerResult get_quantizer_result = quantizer();

        MeshTopoCache topocache(*this, get_quantizer_result.return_value());

        std::vector<TopoTri*> toDelete;

        toDelete.reserve(topocache.triangles().size() / 2);

        for (auto& currentTriangle : topocache.triangles())
        {
            TriCode code = classify(tris_[TriangleByIndicesIndex( currentTriangle.ref() )].bool_alg_data());

            switch (code)
            {
                case TriCode::DELETE_TRI:
                    toDelete.push_back(&currentTriangle);
                    break;

                case TriCode::FLIP_TRI:
                    topocache.flipTri(&currentTriangle);
                    break;

                case TriCode::KEEP_TRI:

                default:
                    break;
            }
        }

        for (auto tptr : toDelete)
        {
            topocache.deleteTri(tptr);
        }

        topocache.commit();
    }

    Mesh::BooleanOperationResult Mesh::Union(const CorkMesh& rhs, const SolverControlBlock& solverControlBlock) const
    {
        //	Collect some starting statistics

        //		unsigned long				startingVirtualMemory = GetConsumedVirtualMemory();

        boost::timer::cpu_timer elapsedTime;

        elapsedTime.start();

        //	Duplicate the mesh

        std::unique_ptr<Mesh> resultMesh(new Mesh(*this, solverControlBlock));

        //		resultMesh->m_performanceStats.setStartingVirtualMemorySizeInMB( startingVirtualMemory );

        SetupBooleanProblemResult result = resultMesh->SetupBooleanProblem(dynamic_cast<const Mesh&>(rhs));

        if (!result.succeeded())
        {
            return (BooleanOperationResult::failure(result,
                                                    BooleanOperationResultCodes::ERROR_DURING_BOOLEAN_PROBLEM_SETUP,
                                                    "Error Occurred During Boolean Problem Setup Phase."));
        }

        resultMesh->doDeleteAndFlip([](uint32_t data) -> TriCode {
            if ((data & 2) == 2)  // part of op 0/1 INSIDE op 1/0
            {
                return TriCode::DELETE_TRI;
            }
            else  // part of op 0/1 OUTSIDE op 1/0
            {
                return TriCode::KEEP_TRI;
            }
        });

        //	Collect the ending statistics

        elapsedTime.stop();
        resultMesh->performance_stats_.set_elapsed_cpu_time_in_nano_seconds(elapsedTime.elapsed().system +
                                                                            elapsedTime.elapsed().user);
        resultMesh->performance_stats_.set_elapsed_wall_time_in_nano_seconds(elapsedTime.elapsed().wall);

        resultMesh->performance_stats_.set_number_of_triangles_in_final_mesh(
            (unsigned long)resultMesh->triangles().size());
        //		resultMesh->m_performanceStats.setEndingVirtualMemorySizeInMB( GetConsumedVirtualMemory() );

        //	Finished with success

        return BooleanOperationResult(std::move(resultMesh));
    }

    Mesh::BooleanOperationResult Mesh::Difference(const CorkMesh& rhs,
                                                  const SolverControlBlock& solverControlBlock) const
    {
        //	Collect some starting statistics

        //		unsigned long		startingVirtualMemory = GetConsumedVirtualMemory();

        boost::timer::cpu_timer elapsedTime;

        elapsedTime.start();

        //	Duplicate the mesh

        std::unique_ptr<Mesh> resultMesh(new Mesh(*this, solverControlBlock));

        //		resultMesh->m_performanceStats.setStartingVirtualMemorySizeInMB( startingVirtualMemory );

        SetupBooleanProblemResult result = resultMesh->SetupBooleanProblem(dynamic_cast<const Mesh&>(rhs));

        if (!result.succeeded())
        {
            return BooleanOperationResult::failure(result,
                                                   BooleanOperationResultCodes::ERROR_DURING_BOOLEAN_PROBLEM_SETUP,
                                                   "Error Occurred During Boolean Problem Setup Phase.");
        }

        resultMesh->doDeleteAndFlip([](uint32_t data) -> TriCode {
            if (data == 2 || data == 1)  // part of op 0 INSIDE op 1, part of op 1 OUTSIDE op 0
            {
                return TriCode::DELETE_TRI;
            }
            else if (data == 3)  // part of op 1 INSIDE op 1
            {
                return TriCode::FLIP_TRI;
            }
            else  // part of op 0 OUTSIDE op 1
            {
                return TriCode::KEEP_TRI;
            }
        });

        //	Collect the ending statistics

        elapsedTime.stop();
        resultMesh->performance_stats_.set_elapsed_cpu_time_in_nano_seconds(elapsedTime.elapsed().system +
                                                                            elapsedTime.elapsed().user);
        resultMesh->performance_stats_.set_elapsed_wall_time_in_nano_seconds(elapsedTime.elapsed().wall);

        resultMesh->performance_stats_.set_number_of_triangles_in_final_mesh(
            (unsigned long)resultMesh->triangles().size());
        //		resultMesh->m_performanceStats.setEndingVirtualMemorySizeInMB( GetConsumedVirtualMemory() );

        //	Finished with success

        return BooleanOperationResult::success(std::move(resultMesh));
    }

    Mesh::BooleanOperationResult Mesh::Intersection(const CorkMesh& rhs,
                                                    const SolverControlBlock& solverControlBlock) const
    {
        //	Collect some starting statistics

        //		unsigned long			startingVirtualMemory = GetConsumedVirtualMemory();

        boost::timer::cpu_timer elapsedTime;

        elapsedTime.start();

        //	Duplicate the mesh

        std::unique_ptr<Mesh> resultMesh(new Mesh(*this, solverControlBlock));

        //		resultMesh->m_performanceStats.setStartingVirtualMemorySizeInMB( startingVirtualMemory );

        SetupBooleanProblemResult result = resultMesh->SetupBooleanProblem(dynamic_cast<const Mesh&>(rhs));

        if (!result.succeeded())
        {
            return BooleanOperationResult::failure(result,
                                                   BooleanOperationResultCodes::ERROR_DURING_BOOLEAN_PROBLEM_SETUP,
                                                   "Error Occurred During Boolean Problem Setup Phase.");
        }

        //	Don't let the returns below confuse you - the code is a lambda

        resultMesh->doDeleteAndFlip([](uint32_t data) -> TriCode {
            if ((data & 2) == 0)  // part of op 0/1 OUTSIDE op 1/0
            {
                return (TriCode::DELETE_TRI);
            }
            else  // part of op 0/1 INSIDE op 1/0
            {
                return (TriCode::KEEP_TRI);
            }
        });

        //	Collect the ending statistics

        elapsedTime.stop();
        resultMesh->performance_stats_.set_elapsed_cpu_time_in_nano_seconds(elapsedTime.elapsed().system +
                                                                            elapsedTime.elapsed().user);
        resultMesh->performance_stats_.set_elapsed_wall_time_in_nano_seconds(elapsedTime.elapsed().wall);

        resultMesh->performance_stats_.set_number_of_triangles_in_final_mesh(
            (unsigned long)resultMesh->triangles().size());
        //		resultMesh->m_performanceStats.setEndingVirtualMemorySizeInMB( GetConsumedVirtualMemory() );

        //	Finished with success

        return BooleanOperationResult::success(std::move(resultMesh));
    }

    Mesh::BooleanOperationResult Mesh::SymmetricDifference(const CorkMesh& rhs,
                                                           const SolverControlBlock& solverControlBlock) const
    {
        //	Collect some starting statistics

        //		unsigned long			startingVirtualMemory = GetConsumedVirtualMemory();

        boost::timer::cpu_timer elapsedTime;

        elapsedTime.start();

        //	Duplicate the mesh

        std::unique_ptr<Mesh> resultMesh(new Mesh(*this, solverControlBlock));

        //		resultMesh->m_performanceStats.setStartingVirtualMemorySizeInMB( startingVirtualMemory );

        SetupBooleanProblemResult result = resultMesh->SetupBooleanProblem(dynamic_cast<const Mesh&>(rhs));

        if (!result.succeeded())
        {
            return BooleanOperationResult::failure(result,
                                                   BooleanOperationResultCodes::ERROR_DURING_BOOLEAN_PROBLEM_SETUP,
                                                   "Error Occurred During Boolean Problem Setup Phase.");
        }

        //	Don't let the returns below confuse you - the code is a lambda

        resultMesh->doDeleteAndFlip([](uint32_t data) -> TriCode {
            if ((data & 2) == 0)  // part of op 0/1 OUTSIDE op 1/0
            {
                return (TriCode::KEEP_TRI);
            }
            else if ((data & 2) == 2)
            {
                return (TriCode::DELETE_TRI);
            }
            else
            {
                return (TriCode::FLIP_TRI);
            }
        });

        //	Collect the ending statistics

        elapsedTime.stop();
        resultMesh->performance_stats_.set_elapsed_cpu_time_in_nano_seconds(elapsedTime.elapsed().system +
                                                                            elapsedTime.elapsed().user);
        resultMesh->performance_stats_.set_elapsed_wall_time_in_nano_seconds(elapsedTime.elapsed().wall);

        resultMesh->performance_stats_.set_number_of_triangles_in_final_mesh(
            (unsigned long)resultMesh->triangles().size());
        //		resultMesh->m_performanceStats.setEndingVirtualMemorySizeInMB( GetConsumedVirtualMemory() );

        //	Finished with success

        return Mesh::BooleanOperationResult::success(std::move(resultMesh));
    }

    std::unique_ptr<TriangleMesh> Mesh::ToTriangleMesh() const
    {
        std::unique_ptr<IncrementalVertexIndexTriangleMeshBuilder> triangleMeshBuilder(
            IncrementalVertexIndexTriangleMeshBuilder::GetBuilder(vertices().size(), triangles().size()));

        for (auto& currentVertex : vertices())
        {
            triangleMeshBuilder->AddVertex(Vertex3D((NUMERIC_PRECISION)currentVertex.x(),
                                                    (NUMERIC_PRECISION)currentVertex.y(),
                                                    (NUMERIC_PRECISION)currentVertex.z()));
        }

        for_raw_tris([&](VertexIndex a, VertexIndex b, VertexIndex c) { triangleMeshBuilder->AddTriangle(a, b, c); });

        return (triangleMeshBuilder->Mesh());
    }

}  // namespace Cork::Meshes
