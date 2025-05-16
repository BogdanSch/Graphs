#pragma once
#include <vector>
#include <map>
#include "Edge.h"
#include "Vertex.h"

using std::map;
using std::vector;

class WeightedGraph
{
private:
	vector<Edge> edges;
	vector<Vertex> vertices;
	map<Vertex, int> mapVertices(Vertex begin);
	Vertex findMinDistanceVertex(const map<Vertex, int>& dist, const map<Vertex, bool>& visited);
	vector<Edge> getNeighbouringEdges(Vertex origin);
	Vertex* getUnvisitedVertex(const vector<Vertex>& visitedVertices);
	void sortEdges();
public:
	WeightedGraph() {}
	WeightedGraph(vector<Edge> edges, vector<Vertex> vertices) : edges(edges), vertices(vertices) {}
	~WeightedGraph() {}
	void addEdge(Vertex from, Vertex to, int weight);
	void removeLastEdge();
	//void addVertex(Vertex vertex) { vertices.push_back(vertex); }
	vector<Edge> getEdges() const { return edges; }
	vector<Vertex> getVertices() const { return vertices; }
	size_t getEdgesCount() const { return edges.size(); }
	size_t getVerticesCount() const { return vertices.size(); }
	map<Vertex, vector<Vertex>> prim_UAR_ADL();
	void kruskal();
	void dijkstra(Vertex begin, Vertex end);
	map<Vertex, vector<Vertex>> createAdjacencyList();
	void printGraph();
};

