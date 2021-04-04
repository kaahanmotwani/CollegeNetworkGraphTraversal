#include "graph.h"

#include <fstream>
#include <iostream>
#include <stack>
#include <array>
#include <unordered_map>
#include <queue>
#include <functional>
#include <vector>

using std::string;
using std::vector;
using std::stack;
using std::array;
using std::unordered_map;
using std::queue;
using std::make_pair;

CollegeMsgGraph::CollegeMsgGraph(const string& fileName, const int numUsers) {
    this->numUsers = numUsers;
    std::ifstream f(fileName);
    matrix = AdjacencyMatrix(numUsers, vector<Vertex>(numUsers, -1));
    Vertex from, to;
    long _;
    int current_index = 0;
    while (f >> from >> to >> _) {
        matrix[from - 1][to - 1] = current_index;
        vertexMap[from] += 1;
        edges.push_back(Edge(from - 1, to - 1, vertexMap[from]));
        ++current_index;
    }
}

CollegeMsgGraph::CollegeMsgGraph(const string& adjacencyMatrixPath, const string& edgeListPath, const int numUsers) {
    this->numUsers = numUsers;
    std::ifstream a(adjacencyMatrixPath);
    std::ifstream e(edgeListPath);
    int value;
    for (Vertex _ = 0; _ < numUsers; ++_) {
        vector<Vertex> row;
        for (Vertex __ = 0; __ < numUsers; ++__) {
            a >> value;
            row.push_back(value);
        }
        matrix.push_back(row);
    }

    Vertex from, to;
    Weight weight;
    while (e >> from >> to >> weight) {
        edges.push_back(Edge(from, to, weight));
    }
}

void CollegeMsgGraph::writeAdjacencyMatrix(const string& fileName) const {
    std::ofstream out(fileName);
    for (const vector<Vertex>& row : matrix) {
        for (const Vertex& val : row) {
            out << val << " ";
        }
        out << "" << std::endl;
    }
}

void CollegeMsgGraph::writeEdgeList(const string& fileName) const {
    writeEdgeSequence(fileName, edges);
}

void CollegeMsgGraph::writeBFS(const string& fileName) const {
    vector<Vertex> b = this->bfs();
    writeVertexSequence(fileName, b);
}

void CollegeMsgGraph::writeShortestPath(const string& fileName, Vertex source, Vertex destination) const {
    vector<Vertex> shortest_path = shortestPath(source, destination);
    if (shortest_path.empty() || shortest_path.front() != source) { //checks for valid paths
        std::cout << "path not found" << std::endl;
        std::ofstream out(fileName);
        out << "path not found" << std::endl;
        return;
    }
    std::cout << "writing path to " << fileName << std::endl;
    writeVertexSequence(fileName,shortest_path);
}

void CollegeMsgGraph::writeConnectedComponents(const string& fileName) const {
    unordered_map<Vertex, vector<Vertex>> components = getConnectedComponents();
    std::ofstream out(fileName);

    out << "Number of Components: " << components.size() << std::endl;
    out << "Components: " << std::endl;
    for (auto comp : components) {
        for (Vertex v : comp.second) {
            out << v << " ";
        }
        out << std::endl;
    }
    out << std::endl;
}

void CollegeMsgGraph::writeVertexSequence(const string& fileName, const vector<CollegeMsgGraph::Vertex>& vertices) {
    std::ofstream out(fileName);
    for (const Vertex& v : vertices) {
        out << v << std::endl;
    }
}

void CollegeMsgGraph::writeEdgeSequence(const string& fileName, const vector<CollegeMsgGraph::Edge>& edges) {
    std::ofstream out(fileName);
    for (const Edge& e : edges) {
        out << e.source << " " << e.destination << " " << e.weight << std::endl;
    }
}

unordered_map<CollegeMsgGraph::Vertex, vector<CollegeMsgGraph::Vertex>> CollegeMsgGraph::getConnectedComponents() const {    
    stack<Vertex> list;
    vector<bool> visited(numUsers, false);

    // Do DFS on each vertex to find all reachable vertices
    for (Vertex v = 0; v < numUsers; v++) {
        visit(v, list, visited);
    }

    unordered_map<Vertex, vector<Vertex>> components;
    vector<bool> assigned(numUsers, false);

    // Do DFS a second time on each vertex to find all vertices for which the current vertex is reachable
    while (!list.empty()) {
        assign(list.top(), list.top(), components, assigned);
        list.pop();
    }

    return components;
}

void CollegeMsgGraph::visit(Vertex curr, stack<Vertex>& list, vector<bool>& visited) const {
    if (visited[curr]) {
        return;
    }
    visited[curr] = true;
    
    // Recursive search on all outgoing edge neighbors
    for (Vertex x = 0; x < numUsers; x++) {
        if (matrix[curr][x] != -1) {
            visit(x, list, visited);
        }
    }

    list.push(curr);
}

void CollegeMsgGraph::assign(Vertex curr, Vertex root, unordered_map<Vertex, vector<Vertex>>& components, vector<bool>& assigned) const {
    if (assigned[curr]) {
        return;
    }

    // Add vertex to either an existing component or a new one
    assigned[curr] = true;
    components[root].push_back(curr);

    // Recursive search on all incoming edge neighbors
    for (Vertex x = 0; x < numUsers; x++) {
        if (matrix[x][curr] != -1) {
            assign(x, root, components, assigned);
        }
    }
}

vector<CollegeMsgGraph::Vertex> CollegeMsgGraph::bfs() const {
    vector<bool> visited(numUsers, false);
    vector<Vertex> traversal;
    for (int i = 0; i < numUsers; i++) {
        if (!visited[i]) {
            bfs(i, visited, traversal);
        }
    } 
    return traversal;
}

void CollegeMsgGraph::bfs(const Vertex start, vector<bool>& visited, vector<Vertex> & traversal) const {
    queue<Vertex> q; 
    q.push(start); 
    traversal.push_back(start);
    visited[start] = true; 
    while (!q.empty()) { 
        int f = q.front(); 
        q.pop(); 
        for (int i = 0; i < numUsers; i++) {
            if (matrix[f][i] == -1) {
                continue;
            }
            Edge edge = edges[matrix[f][i]];
            if (!visited[edge.destination]) {
                q.push(edge.destination);
                traversal.push_back(edge.destination);
                visited[edge.destination] = true;
            }
        }
    }
}

/**
 * 1) find edge-list with shortest paths to every vertex from source
 * 2) find destination vertex in the graph
 * 3) find edge with dest and track back to source while storing vertices along the path
 */
vector<CollegeMsgGraph::Vertex> CollegeMsgGraph::shortestPath(Vertex a, Vertex b) const {
    vector<Vertex> a_to_b;
    if (a > numUsers || a < 1 || b > numUsers || b < 1) return a_to_b; //checks for valid inputs

    vector<Edge> edges_a = dijkstraTree(a); //finds best path tree from a/source

    Vertex x = b;//finds every vertex from b to a
    int i = 0;
    while (x != a && i < numUsers) {
        a_to_b.insert(a_to_b.begin(), x);
        x = find_source(x, edges_a);
        i++;
    }
    if (x != -1 && i < numUsers) a_to_b.insert(a_to_b.begin(), x); //checks if there was a path from a to b

    return a_to_b;
}

CollegeMsgGraph::Vertex CollegeMsgGraph::find_source(Vertex destination, vector<Edge> edge_list) const {
    Vertex source = -1;
    for (size_t i = 0; i < edge_list.size(); i++) {
        if (edge_list[i].destination == destination) 
            source = edge_list[i].source;
    }
    return source;
}

vector<CollegeMsgGraph::Edge> CollegeMsgGraph::dijkstraTree(Vertex source) const {
    vector<Edge> paths;
    map<Vertex, int> depth;
    for (int i = 1; i <= numUsers; i++) {
        depth[i] = INT_MAX;//init all depths to max
    }
    source = source - 1;
    depth[source] = 0;//init start to 0

    std::priority_queue<vertex_depth_pair> distances;
    distances.push(make_pair(depth[source], source));

    while (!distances.empty()) {
        Vertex u = distances.top().second;//finds vertex associated with min edge weight
        distances.pop();
        for (int i = 0; i < numUsers; i++) { //looks for adj vertices
            if (matrix[u][i] != -1) {
                Vertex v = i;
                int weight = edges[matrix[u][v]].weight;
                if (depth[u] + weight < depth[v]) { //checks if new path is better
                    depth[v] = depth[u] + weight; //updates path
                    distances.push(make_pair((-1) * depth[v], v));
                    paths.push_back(Edge(u + 1, v + 1, depth[v]));
                }
            }
        }
    }

    return paths;
}