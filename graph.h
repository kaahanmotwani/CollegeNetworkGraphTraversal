#pragma once

#include <vector>
#include <fstream>
#include <iostream>
#include <stack>
#include <array>
#include <unordered_map>

#include <map>

using std::vector;
using std::map;
using std::string;
using std::vector;
using std::stack;
using std::array;
using std::unordered_map;

/*
Graph definition for the UCI CollegeMsg Temporal Network Dataset.
https://snap.stanford.edu/data/CollegeMsg.html
Using an adjacency matrix implementation with an edge list.
Datset comes in the form of a text file with each line
following the format "SRC DST UNIXTS" where SRC is the source user,
DST is the destination user, and UNIXTS is the time of the message
as a unix timestamp.
Note that the dataset must be sorted from earliest to latest by timestamp.
The Timestamp is ignored for this project.
*/
class CollegeMsgGraph {
    public:
        /* A vertex is defined by its integer user id from the dataset. */
        typedef int Vertex;

        /* A weight is an integer which represents the number of times the source user 
        has sent a message including the current message being considered.
        */
        typedef int Weight;

        /* An edge connectes a pair of Vertices (Source, Destination) and stores a Weight
        which corresponds to the cumulative number of messages sent by the "source" user
        at the time of the current message which is represented by the edge.*/
        struct Edge {
            Vertex source;
            Vertex destination;
            Weight weight;
            Edge(Vertex s, Vertex d, Weight w) : source(s), destination(d), weight(w) {};
        };

        /* An adjacency matrix is a 2D vector where each element stores the index of the 
        corresponding Edge in the edgeList if there is an Edge, and -1 if not.*/
        typedef vector<vector<int>> AdjacencyMatrix;
        
        /**
         * @brief maps the depth of a vertex to its ID.
         * 
         */
        typedef std::pair<int, Vertex> vertex_depth_pair;
    
        /** 
        * Constructor for a CollegeMsgGraph from the raw dataset.
        * @param fileName The file path to read from.
        * @param numUsers The number of nodes (users) in the graph.
        */
        CollegeMsgGraph(const string& fileName, const int numUsers);

        /** 
        * Constructor for a CollegeMsgGraph from the adjacency matrix output file
        * and the edge list output file.
        * @param adjacencyMatrixPath File path to an adjacency matrix provided.
        * @param edgeListPath File path to an edge list provided.
        * @param numUsers The number of nodes (users) in the graph.
        */
        CollegeMsgGraph(const string& adjacencyMatrixPath, const string& edgeListPath, const int numUsers);

        /**
         * Writes the adjacency matrix to an output file.
         * Each element in the matrix is separated by a single white space.
         * Each row is on its own line.
         * Each element is the index in the edge list of the corresponding edge.
         * Note: this matrix is meaningless without the corresponding edge list.
         * @param fileName The output file path.
         */
        void writeAdjacencyMatrix(const string& fileName) const;

        /**
         * Returns all of the strongly connected components in the graph.
         * A strongly connected component is a subset of vertices in which every vertex is
         * reachable from every other vertex.
         */
        unordered_map<Vertex, vector<Vertex>> getConnectedComponents() const;

        /**
         * @brief finds the shortest path according to edge weights from source to destination
         * 
         * @param source starting vertex
         * @param destination ending vertex
         * @return vector<Vertex>  all vertices in path from source to destination
         */
        vector<Vertex> shortestPath(Vertex source, Vertex destination) const;

        /**
         * @brief returns BFS traversal of graph
         * 
         * @return vector<Vertex> the traversed vertices
         */
        vector<Vertex> bfs() const;

        /**
         * Writes the shortest path to an output file.
         * @param fileName The output file path.
         * @param bfs The ordered sequence of edges representing the shortest path between two vertices.
         */
        void writeShortestPath(const string& fileName, Vertex source, Vertex destination) const;

        /**
         * Writes the bfs traversal to an output file.
         * @param fileName The output file path.
         * @param bfs The ordered sequence of edges representing the bfs starting at a paritcular vertex.
         */
        void writeBFS(const string& fileName) const;

        /**
         * Writes the connected components to an output file
         * @param fileName The output file path.
         */
        void writeConnectedComponents(const string& fileName) const;

        /**
         * Writes the edge list to an output file.
         * @param fileName The output file path.
         */
        void writeEdgeList(const string& fileName) const;
        
    private: 
        /* The edge list for the graph. */ 
        vector<Edge> edges;

        /* The adjacency matrix for the graph. */
        AdjacencyMatrix matrix;

        /* A map of all vertices onto the number of times that user has sent a message */
        map<Vertex, Weight> vertexMap;

        /* Number of nodes in the graph */
        int numUsers;

        /* Helper function to write a sequence of vertices to an output file.
         * Each vertex is on one line.
         * @param fileName The output file path.
         */
        static void writeVertexSequence(const string& fileName, const vector<Vertex>& vertices);

        /* Helper function to write a sequence of edges to an output file.
         * Each vertex is on one line.
         * @param fileName The output file path.
         */
        static void writeEdges(const string& fileName, const vector<Edge>& edges);

        /* Helper functions for strongly connected components. */
        void visit(Vertex curr, stack<Vertex>& list, vector<bool>& visited) const;
        void assign(Vertex curr, Vertex root, unordered_map<Vertex, vector<Vertex>>& components, vector<bool>& assigned) const;
        static void writeEdgeSequence(const string& fileName, const vector<Edge>& edges);

        /**
         * @brief given an edge list and vertex finds the source of the vertex
         * 
         * @param destination vertex to find source of
         * @param edge_list list of edges
         * @return Vertex source of destination
         */
        Vertex find_source(Vertex destination, vector<Edge> edge_list) const;

        /**
         * @brief finds the shortest path from source to every vertex in graph
         * 
         * @param source: starting vertex
         * @return vector<Edge>: all edges representing best paths
         */
        vector<Edge> dijkstraTree(Vertex source) const;

        /**
         * @brief Breadth first traversal of the given graph
         * 
         * @param start starting vertex
         * @param visited vector of visited vertices
         * @param traversal traversed vertices
         */
        void bfs(const Vertex start, vector<bool>& visited, vector<Vertex> & traversal) const;
};