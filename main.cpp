#include <iostream>
#include "WeightedGraph.h"

using std::cout;

int main()
{
    Vertex A(1, "A");
    Vertex B(2, "B");
    Vertex C(3, "C");
    Vertex D(4, "D");
    Vertex E(5, "E");

    WeightedGraph graph;

    graph.addEdge(A, B, 2);
    graph.addEdge(A, C, 4);
    graph.addEdge(A, D, 1);
    graph.addEdge(B, E, 5);
    graph.addEdge(D, E, 1);
    graph.addEdge(D, C, 3);

	cout << "Graph Edges:\n";
	graph.printGraph();
    cout << "\n";

    cout << "The shortest path from A to E:\n";
	graph.dijkstra(A, E);
    cout << "\n";

	cout << "Minimum Spanning Tree:\n";
	graph.kruskal();
    cout << "\n";

    cout << "Prim's Minimum Spanning Tree:\n";
    map<Vertex, vector<Vertex>> adjacencyList = graph.prim_UAR_ADL();

    for (auto& it : adjacencyList)
    {
        cout << it.first << ": ";
        for (Vertex vertex : it.second)
        {
            cout << vertex << " ";
        }
        cout << "\n";
    }

	return 0;
}