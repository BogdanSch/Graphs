#include "WeightedGraph.h"
#include "DisjointSet.h"
#include <map>
#include <limits>
#include <iostream>

using std::map;
using std::numeric_limits;
using std::pair;
using std::cout;
using std::count;
using std::find;

void WeightedGraph::sortEdges()
{
    for (int i = 0; i < edges.size(); i++)
    {
        int minIndex = i;

        for (int j = i; j < edges.size(); j++)
        {
            if (edges.at(j).getWeight() < edges.at(minIndex).getWeight())
            {
                minIndex = j;
            }
        }

        Edge temp = edges.at(minIndex);
        edges.at(minIndex) = edges.at(i);
        edges.at(i) = temp;
    }
}

vector<Edge> WeightedGraph::getNeighbouringEdges(Vertex origin)
{
    vector<Edge> neighbours = {};

    for (const Edge& edge : edges)
    {
        if (edge.from == origin || edge.to == origin)
            neighbours.push_back(edge);
    }

    return neighbours;
}

Vertex* WeightedGraph::getUnvisitedVertex(const vector<Vertex>& visitedVertices)
{
    for (Vertex& target : vertices)
    {
        auto it = find(visitedVertices.begin(), visitedVertices.end(), target);

        if (it == visitedVertices.end())
        {
            return &target;
        }
    }

    return nullptr;
}

map<Vertex, vector<Vertex>> WeightedGraph::prim_UAR_ADL()
{
    if (edges.empty())
    {
        cout << "Graph is empty.\n";
        return map<Vertex, vector<Vertex>>();
    }

    Vertex* current = &(vertices.at(0));
    vector<Vertex> visited = { *current };

    DisjointSet ds(vertices);
    WeightedGraph mstGraph;

    while (mstGraph.edges.size() <= vertices.size() - 1)
    {
        vector<Edge> neighbouringEdges = getNeighbouringEdges(*current);

        if (neighbouringEdges.empty())
        {
            current = getUnvisitedVertex(visited);
            continue;
        }

        Edge minEdge = neighbouringEdges.at(0);

        for (int i = 1; i < neighbouringEdges.size(); i++)
        {
            if (neighbouringEdges.at(i).getWeight() < minEdge.getWeight())
                minEdge = neighbouringEdges.at(i);
        }

        if (!ds.connected(minEdge.from, minEdge.to))
        {
            mstGraph.addEdge(minEdge.from, minEdge.to, minEdge.getWeight());
            ds.unite(minEdge.from, minEdge.to);
        }

        current = getUnvisitedVertex(visited);

        if (current == nullptr)
            break;

        visited.push_back(*current);
    }

    return mstGraph.createAdjacencyList();
}

void WeightedGraph::kruskal()
{
    if (edges.empty())
    {
        cout << "Graph is empty.\n";
        return;
    }

    sortEdges();

    DisjointSet ds(vertices);

    WeightedGraph mstGraph;

    for (const Edge& edge : edges)
    {
        if (!ds.connected(edge.from, edge.to))
        {
            mstGraph.addEdge(edge.from, edge.to, edge.getWeight());
            ds.unite(edge.from, edge.to);
        }

        if (mstGraph.edges.size() == vertices.size() - 1)
            break;
    }

    mstGraph.printGraph();
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

void WeightedGraph::removeLastEdge()
{
	if (edges.size() == 0)
	{
		cout << "Graph is empty.\n";
		return;
	}

	Edge lastEdge = edges.back();
	edges.pop_back();

	for (int i = 0; i < vertices.size(); i++)
	{
		Vertex vertex = vertices.at(i);

		if (vertex == lastEdge.from || vertex == lastEdge.to)
		{
            int countOccurances = count(vertices.begin(), vertices.end(), vertex);

			if (countOccurances == 1)
			{
				vertices.erase(vertices.begin() + i);
				i--;
			}
		}
	}
}

void WeightedGraph::dijkstra(Vertex begin, Vertex end)
{
    if (edges.size() == 0)
    {
        cout << "Graph is empty.\n";
        return;
    }

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

map<Vertex, vector<Vertex>> WeightedGraph::createAdjacencyList()
{
    map<Vertex, vector<Vertex>> adjacencyList;

    for (int i = 0; i < vertices.size(); i++)
    {
        adjacencyList[vertices.at(i)] = { };
    }

    for (int i = 0; i < edges.size(); i++)
    {
        Vertex from = edges.at(i).from;
        Vertex to = edges.at(i).to;

        adjacencyList[from].push_back(to);
        adjacencyList[to].push_back(from);
    }

    return adjacencyList;
}

void WeightedGraph::printGraph()
{
	if (edges.size() == 0)
	{
		cout << "Graph is empty.\n";
		return;
	}
	for (const Edge& edge : edges)
	{
		cout << edge.from.name << " --(" << edge.getWeight() << ")--> " << edge.to.name << "\n";
	}
}
