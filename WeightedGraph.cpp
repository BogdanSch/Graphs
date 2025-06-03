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

map<Vertex, vector<Vertex>> WeightedGraph::prim_UAR_ADL()
{
    if (edges.empty())
    {
        cout << "Graph is empty.\n";
        return createAdjacencyList();
    }

    vector<Vertex> visited = { vertices.at(0) };
    WeightedGraph mstGraph;

    while (visited.size() < vertices.size())
    {
        Edge minEdge;
        bool isMinEdgeFound = false;

        for (const Vertex& v : visited)
        {
            vector<Edge> neighbours = getNeighbouringEdges(v);

            for (const Edge& e : neighbours)
            {
                bool fromVisited = find(visited.begin(), visited.end(), e.from) != visited.end();
                bool toVisited = find(visited.begin(), visited.end(), e.to) != visited.end();

                if ((fromVisited && !toVisited) || (!fromVisited && toVisited))
                {
                    if (!isMinEdgeFound || e.getWeight() < minEdge.getWeight())
                    {
                        minEdge = e;
						isMinEdgeFound = true;
                    }
                }
            }
        }

        if (!isMinEdgeFound)
        {
            cout << "Graph is not connected.\n";
            break;
        }

        mstGraph.addEdge(minEdge.from, minEdge.to, minEdge.getWeight());

        Vertex next = find(visited.begin(), visited.end(), minEdge.from) == visited.end()
            ? minEdge.from : minEdge.to;

        visited.push_back(next);
    }

    return mstGraph.createAdjacencyList();
}

//O(e^2 + e * α(v))
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

Vertex WeightedGraph::findMinDistanceVertex(const map<Vertex, int>& distances, const map<Vertex, bool>& visited)
{
    int minDist = numeric_limits<int>::max();
    Vertex minVertex = vertices.at(0);

    for (const Vertex& v : vertices)
    {
        if (!visited.at(v) && distances.at(v) < minDist)
        {
            minDist = distances.at(v);
            minVertex = v;
        }
    }

    return minVertex;
}

//O(v * (v + e))
void WeightedGraph::dijkstra(Vertex begin, Vertex end)
{
    if (edges.empty())
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
                int cost = distances[neighbour] + edge.getWeight();

                if (cost < distances[to])
                {
                    distances[to] = cost;
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

void WeightedGraph::addEdge(Vertex from, Vertex to, int weight)
{
    Edge newEdge(from, to, weight);

    bool isEdgePresent = find(edges.begin(), edges.end(), newEdge) != edges.end();
    if (isEdgePresent) return;

    edges.push_back(newEdge);

    bool isVertexPresent = find(vertices.begin(), vertices.end(), from) != vertices.end();
    if (!isVertexPresent) vertices.push_back(from);

    isVertexPresent = find(vertices.begin(), vertices.end(), to) != vertices.end();
    if (!isVertexPresent) vertices.push_back(to);
}

map<Vertex, vector<Vertex>> WeightedGraph::createAdjacencyList()
{
    if (vertices.empty() || edges.empty()) return map<Vertex, vector<Vertex>>();

    map<Vertex, vector<Vertex>> adjacencyList;

    for (int i = 0; i < vertices.size(); i++)
    {
        adjacencyList[vertices.at(i)] = {};
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
