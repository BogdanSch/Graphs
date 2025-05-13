#pragma once
#include <string>

using std::string;

class Vertex
{
public:
	int id;
	string name;
	Vertex(int id, string name) : id(id), name(name) {}
	Vertex() : id(0), name("") {}
	bool operator==(const Vertex& other) const { return id == other.id; }
	bool operator!=(const Vertex& other) const { return id != other.id; }
	bool operator<(const Vertex& other) const { return id < other.id; }
};

