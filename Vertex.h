#pragma once
#include <string>
#include <ostream>

using std::string;
using std::ostream;

class Vertex
{
private: 
	int id;
public:
	string name;
	Vertex(int id, string name) : id(id), name(name) {}
	Vertex() : id(0), name("") {}
	bool operator==(const Vertex& other) const { return id == other.id; }
	bool operator!=(const Vertex& other) const { return id != other.id; }
	bool operator<(const Vertex& other) const { return id < other.id; }
	friend std::ostream& operator<<(std::ostream& os, const Vertex& v) {
		return os << v.name;
	}
};

