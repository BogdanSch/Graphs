#pragma once
#include <map>
#include <vector>
#include "Vertex.h"

using std::map;
using std::vector;

class DisjointSet {
private:
    std::map<Vertex, Vertex> parent;
public:
    DisjointSet(const vector<Vertex>& vertices);
    Vertex find(Vertex v);
    void unite(Vertex a, Vertex b);
    bool connected(Vertex a, Vertex b);
};

