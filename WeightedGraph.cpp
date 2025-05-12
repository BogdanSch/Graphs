#include "WeightedGraph.h"
#include <map>
#include <limits>
#include <iostream>

using std::map;
using std::numeric_limits;
using std::pair;
using std::cout;
using std::count;

void WeightedGraph::kruskal()
{
}

map<Vertex, int> WeightedGraph::mapVertices(Vertex begin)
{
    map<Vertex, int> mappedVertices;

    for (Vertex vertex : vertices)
    {
        int value = numeric_limits<int>::max();

        if (vertex == begin) value = 0;
        
        mappedVertices[vertex] = value;
    }
    return mappedVertices;
}

Vertex WeightedGraph::findMinDistanceVertex(const map<Vertex, int>& dist, const map<Vertex, bool>& visited)
{
    int minDist = numeric_limits<int>::max();
    Vertex minVertex = vertices[0];

    for (const Vertex& v : vertices)
    {
        if (!visited.at(v) && dist.at(v) < minDist)
        {
            minDist = dist.at(v);
            minVertex = v;
        }
    }

    return minVertex;
}

void WeightedGraph::addEdge(Vertex from, Vertex to, int weight)
{
    Edge newEdge(from, to, weight);

    int countOccurances = count(edges.begin(), edges.end(), newEdge);
    if (countOccurances > 0) return;

    edges.push_back(newEdge);

    countOccurances = count(vertices.begin(), vertices.end(), from);
    if (countOccurances <= 0) vertices.push_back(from);

    countOccurances = count(vertices.begin(), vertices.end(), to);
    if (countOccurances <= 0) vertices.push_back(to);
}

void WeightedGraph::dijkstra(Vertex begin, Vertex end)
{
    map<Vertex, int> distances = mapVertices(begin);
    map<Vertex, bool> visited;

    for (Vertex vertex : vertices)
    {
        visited[vertex] = false;
    }

    while (true)
    {
        Vertex neighbour = findMinDistanceVertex(distances, visited);

        if (visited[neighbour] || distances[neighbour] == numeric_limits<int>::max())
            break;

        visited[neighbour] = true;

        for (Edge edge : edges)
        {
            if (edge.from == neighbour)
            {
                Vertex to = edge.to;
                int alt = distances[neighbour] + edge.getWeight();

                if (alt < distances[to])
                {
                    distances[to] = alt;
                }
            }
        }
    }

	cout << "Shortest path from " << begin.name << " to " << end.name << " is: ";
	if (distances[end] == numeric_limits<int>::max())
	{
		cout << "No path found.\n";
	}
	else
	{
		cout << distances[end] << "\n";
	}
}

void WeightedGraph::printGraph()
{
	for (const Edge& edge : edges)
	{
		cout << edge.from.name << " --(" << edge.getWeight() << ")--> " << edge.to.name << "\n";
	}
}
