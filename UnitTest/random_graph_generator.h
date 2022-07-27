#pragma once



// A C++ Program to generate test cases for
// an unweighted directed graph

#include <memory>
#include <set>
#include <vector>

using namespace std;



class Graph : public std::vector<std::pair<size_t, size_t>>
{
public :

	Graph( size_t		numVertices)
		: m_numVertices( numVertices )
	{}


	size_t		numVertices() const
	{
		return(m_numVertices);
	}


	void		merge(const Graph&		graphToMerge)
	{
		size_t	offset = m_numVertices;

		for (auto nextEdge : graphToMerge)
		{
			push_back(std::make_pair( nextEdge.first + offset, nextEdge.second + offset));
		}

		m_numVertices += graphToMerge.m_numVertices;
	};

	void		randomize( long		randomSeed )
	{
		//	The same seed should always result in the same graph

		srand(randomSeed);

		for (int i = 0; i < m_numVertices; i++)
		{
			size_t		index1 = rand() % m_numVertices;
			size_t		index2 = rand() % m_numVertices;

			std::swap(at(index1), at(index2));
		}
	}


private :

	size_t		m_numVertices;

};



class RandomGraphGenerator
{
public:

	RandomGraphGenerator()
	{}

	~RandomGraphGenerator()
	{}

	static std::unique_ptr<Graph>		GenerateGraph( long			numVertices,
													   long			connectivity,
													   long			randomSeed)
	{
		//	The same seed should always result in the same graph

		srand(randomSeed);

		//	Create the graph

		std::unique_ptr<Graph>				graph( new Graph(numVertices) );
		std::set<std::pair<long, long>>		currentEdges;
		std::vector<int>					availableVertices;

		availableVertices.reserve(numVertices);

		for (int i = 0; i < connectivity; i++)
		{
			for (int j = 0; j < numVertices; j++)
			{
				availableVertices.push_back(j);
			}

			while (availableVertices.size() > 1)
			{
				std::pair<long, long>		nextEdge(make_pair(getVertex(availableVertices), getVertex(availableVertices)));

				if (currentEdges.find(nextEdge) != currentEdges.end())
				{
					availableVertices.push_back(nextEdge.first);
					availableVertices.push_back(nextEdge.second);

					continue;
				}

				graph->push_back( nextEdge );
			}
		}

		return( graph );
	}


private :

	static long			getVertex(std::vector<int>&		availableVertices)
	{
		long		vertIndex = rand() % availableVertices.size();
		long		vert = availableVertices[vertIndex];

		if (vertIndex != availableVertices.size() - 1)
		{
			availableVertices[vertIndex] = availableVertices.back();
		}

		availableVertices.pop_back();

		return(vert);
	}

};

