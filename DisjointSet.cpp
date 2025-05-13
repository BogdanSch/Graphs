#include "DisjointSet.h"

using std::vector;

DisjointSet::DisjointSet(const vector<Vertex>& vertices)
{
    for (const Vertex& v : vertices) {
        parent[v] = v;
    }
}

Vertex DisjointSet::find(Vertex v)
{
    if (parent[v] == v) return v;
    return parent[v] = find(parent[v]);
}

void DisjointSet::unite(Vertex a, Vertex b)
{
    Vertex rootA = find(a);
    Vertex rootB = find(b);
    if (rootA != rootB) parent[rootA] = rootB;
}

bool DisjointSet::connected(Vertex a, Vertex b)
{
    return find(a) == find(b);
}