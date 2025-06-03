#pragma once
#include "Vertex.h"

class Edge
{
private:
	int weight;
public:
	Vertex from;
	Vertex to;
	Edge() : weight(0), from(Vertex()), to(Vertex()) {}
	Edge(Vertex from, Vertex to, int weight) : from(from), to(to) { setWeight(weight); }
	void setWeight(int weight) { this->weight = weight >= 0 ? weight : 0; }
	int getWeight() const { return weight; }
	bool operator<(const Edge& other) const { return this->weight < other.getWeight(); }
	bool operator==(const Edge& other) const { return this->weight == other.getWeight() && this->from == other.from && this->to == other.to; }
};

