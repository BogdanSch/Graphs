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
	void kruskal();
	void dijkstra(Vertex begin, Vertex end);
	void printGraph();
};

