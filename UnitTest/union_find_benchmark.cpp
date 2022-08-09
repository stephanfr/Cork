
#include <catch2/catch_all.hpp>
#include <mutex>

#include "primitives/primitives.hpp"
#include "random_graph_generator.h"
#include "tbb/parallel_for.h"
#include "tbb/tbb.h"
#include "util/union_find.hpp"


typedef tbb::concurrent_vector<tbb::concurrent_unordered_set<size_t>> Components;

bool operator==(const Components& first, const Components& second)
{
    if (first.size() != second.size())
    {
        return (false);
    }

    for (int i = 0; i < first.size(); i++)
    {
        if (first[i].size() != second[i].size())
        {
            return (false);
        }

        for (size_t element : first[i])
        {
            if (second[i].find(element) == second[i].end())
            {
                return (false);
            }
        }
    }

    return (true);
}

std::unique_ptr<Components> GetComponents(const Graph& graph, IUnionFind& unionFind)
{
    std::unique_ptr<Components> components(new Components());

    std::vector<long> uq_ids(graph.numVertices(), long(-1));

    components->reserve(64);

    std::mutex vectorLock;

    tbb::parallel_for(tbb::blocked_range<size_t>((size_t)0, graph.numVertices()),
        [&](tbb::blocked_range<size_t> range)
        {
            for (auto i = range.begin(); i != range.end(); i++)
            {
                size_t ufid = unionFind.find(i);

            retry:

                if (uq_ids[ufid] == long(-1))
                {  // unassigned
                    std::lock_guard<std::mutex> lock(vectorLock);

                    if (uq_ids[ufid] != long(-1)) goto retry;

                    size_t N = components->size();
                    components->emplace_back();

                    uq_ids[ufid] = uq_ids[i] = (long)N;
                    (*components)[N].insert(i);
                }
                else
                {                              // assigned already
                    uq_ids[i] = uq_ids[ufid];  // propagate assignment
                    (*components)[uq_ids[i]].insert(i);
                }
            }
        });

    return (components);
}

std::unique_ptr<Components> GetReferenceComponents(const Graph& graph)
{
    RankWeightedSerialUnionFind refUnionFind(graph.numVertices());

    for (auto& edge : graph)
    {
        refUnionFind.unite(edge.first, edge.second);
    }

    return (GetComponents(graph, refUnionFind));
}

class UnionFindTestFixture
{
   public:
    UnionFindTestFixture()
    {
        m_graph = RandomGraphGenerator().GenerateGraph(2000000, 7, 0);

        std::unique_ptr<Graph> graph2(RandomGraphGenerator().GenerateGraph(700000, 4, 1));
        std::unique_ptr<Graph> graph3(RandomGraphGenerator().GenerateGraph(30000, 5, 2));
        std::unique_ptr<Graph> graph4(RandomGraphGenerator().GenerateGraph(40000, 3, 3));
        std::unique_ptr<Graph> graph5(RandomGraphGenerator().GenerateGraph(10, 3, 4));
        std::unique_ptr<Graph> graph6(RandomGraphGenerator().GenerateGraph(5, 4, 5));

        m_graph->merge(*graph2);
        m_graph->merge(*graph3);
        m_graph->merge(*graph4);
        m_graph->merge(*graph5);
        m_graph->merge(*graph6);

        m_graph->randomize(6);

        m_refComponents = GetReferenceComponents(*m_graph);
    }

    ~UnionFindTestFixture() {}

    const Graph& getGraph() const { return (*m_graph); }

    const Components& getRefComponents() const { return (*m_refComponents); }

   private:
    std::unique_ptr<Graph> m_graph;

    std::unique_ptr<Components> m_refComponents;
};

bool TestRankWeightedSerialUnionFind(const Graph& graph, const Components& refComponents,
                                     Catch::Benchmark::Chronometer meter)
{
    RankWeightedSerialUnionFind rankUnionFind(graph.numVertices());

    meter.measure(
        [&rankUnionFind, &graph]
        {
            for (auto& edge : graph)
            {
                rankUnionFind.unite(edge.first, edge.second);
            }
        });

    std::unique_ptr<Components> components(GetComponents(graph, rankUnionFind));

    REQUIRE(*components == refComponents);

    return (true);
}

bool TestRandWeightedSerialUnionFind(const Graph& graph, const Components& refComponents,
                                     Catch::Benchmark::Chronometer meter)
{
    RandomWeightedSerialUnionFind randUnionFind(graph.numVertices());

    meter.measure(
        [&randUnionFind, &graph]
        {
            for (auto& edge : graph)
            {
                randUnionFind.unite(edge.first, edge.second);
            }
        });

    std::unique_ptr<Components> components(GetComponents(graph, randUnionFind));

    REQUIRE(*components == refComponents);

    return (true);
}

bool TestRandWeightedParallelUnionFind(const Graph& graph, const Components& refComponents,
                                       Catch::Benchmark::Chronometer meter)
{
    RandomWeightedParallelUnionFind unionFind(graph.numVertices());

    meter.measure(
        [&unionFind, &graph]
        {
            tbb::parallel_for(
                tbb::blocked_range<Graph::const_iterator>(graph.begin(), graph.end()),
                [&](tbb::blocked_range<Graph::const_iterator> edges)
                                       {
                                           for (auto currentEdge : edges)
                                           {
                                               unionFind.unite(currentEdge.first, currentEdge.second);
                                           }
                                       });
        });

    std::unique_ptr<Components> components(GetComponents(graph, unionFind));

    REQUIRE(*components == refComponents);

    return (true);
}

bool TestRandWeightedParallelUnionFind1(const Graph& graph, const Components& refComponents,
                                        Catch::Benchmark::Chronometer meter)
{
    RandomWeightedParallelUnionFind1 unionFind(graph.numVertices());

    meter.measure(
        [&unionFind, &graph]
        {
            tbb::parallel_for(
                tbb::blocked_range<Graph::const_iterator>(graph.begin(), graph.end()),
                [&](tbb::blocked_range<Graph::const_iterator> edges)
                                       {
                                           for (auto current_edge : edges)
                                           {
                                               unionFind.unite(current_edge.first, current_edge.second);
                                           }
                                       });
        });

    std::unique_ptr<Components> components(GetComponents(graph, unionFind));

    REQUIRE(*components == refComponents);

    return (true);
}

TEST_CASE("Basic Union Find with Benchmarks", "[cork-base]")
{
    UnionFindTestFixture testFixture;

    BENCHMARK_ADVANCED("Test Rank Weighted Serial Union Find")(Catch::Benchmark::Chronometer meter)
    {
        return TestRankWeightedSerialUnionFind(testFixture.getGraph(), testFixture.getRefComponents(), meter);
    };

    BENCHMARK_ADVANCED("Test Random Weighted Serial Union Find")(Catch::Benchmark::Chronometer meter)
    {
        return TestRandWeightedSerialUnionFind(testFixture.getGraph(), testFixture.getRefComponents(), meter);
    };

    BENCHMARK_ADVANCED("Test Random Weighted Parallel Union Find")(Catch::Benchmark::Chronometer meter)
    {
        return TestRandWeightedParallelUnionFind(testFixture.getGraph(), testFixture.getRefComponents(), meter);
    };

    BENCHMARK_ADVANCED("Test Random Weighted Parallel Union Find 1")(Catch::Benchmark::Chronometer meter)
    {
        return TestRandWeightedParallelUnionFind1(testFixture.getGraph(), testFixture.getRefComponents(), meter);
    };
}
