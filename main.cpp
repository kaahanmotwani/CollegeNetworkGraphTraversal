#include "graph.h"
#include <string>
#include <unordered_map>
#include <iostream>

using std::unordered_map;

int main(/*int argc, const char * argv[]*/) {
    //GRAPH initialization

    //graph with 8 nodes
    CollegeMsgGraph g1("tests/test_g1.txt", 8); 

    //graph with 10 nodes
    CollegeMsgGraph g2("tests/test_g2.txt", 10);
    
    g1.writeEdgeList("g1_edge.list");

    g2.writeEdgeList("g2_edge.list");

    g1.writeAdjacencyMatrix("g1_adj.matrix");

    g2.writeAdjacencyMatrix("g2_adj.matrix");

    CollegeMsgGraph sccp1("tests/sccp_test1.txt", 3);
    CollegeMsgGraph sccp2("tests/sccp_test2.txt", 4);
    CollegeMsgGraph sccp3("tests/sccp_test3.txt", 4);
    CollegeMsgGraph sccp4("tests/sccp_test4.txt", 8);

    // --------------------------------------------------------------------------------------------------

    // BFS TESTS

    /* Expected:
        0
        1
        3
        2
        4
        5
        6
        7
    */
    g1.writeBFS("bfs1.out");

    /* Expected:
        0
        1
        3
        2
        4
        5
        6
        7
        8
        9
    */
    g2.writeBFS("bfs2.out");

    // -------------------------------------------------------------------------------------------------

    //  SHORTEST PATH TESTS

    /* NOTE: tests based on graph provided by professor Evans in lecture.
    Graph is in tests/testDijkstra.txt
    To use interface with actual dataset download CollegeMsgDataset here : https://snap.stanford.edu/data/CollegeMsg.html
    and pass correct file path */

    /* Valid path test for small graph
       Expected:
        1
        2
        3
        8
    */
    g2.writeShortestPath("valid_path.list", 1, 8);
    
    /* Invalid source test
       Expected:
        path not found
    */
    g2.writeShortestPath("invalid_source.list", 0, 8);

    /* Unconnected components test
       Expected:
        path not found
    */
    g2.writeShortestPath("disjoint.list", 1, 9); 
    
    /* Invalid destination test
       Expected:
        path not found
    */
    g2.writeShortestPath("invalid_destination.list", 1, 11);

    // -------------------------------------------------------------------------------------------------

    //  STRONGLY CONNECTED COMPONENTS TESTS

    /* One cycle component
       Expected:
        Number of Components: 1
        Components: 
        0 2 1 
    */
    sccp1.writeConnectedComponents("sccp_result1.txt");

    /* Two pairs of doubly-linked vertices
       Expected:
        Number of Components: 2
        Components: 
        0 1 
        2 3 
    */
    sccp2.writeConnectedComponents("sccp_result2.txt");

    /* One long one way path - all separate components
       Expected:
        Number of Components: 4
        Components: 
        3 
        2 
        1 
        0
    */
    sccp3.writeConnectedComponents("sccp_result3.txt");

    /* Small mixed sample - 3 components
       Expected:
        Number of Components: 3
        Components: 
        6 5 
        2 3 7 
        0 4 1
    */
    sccp4.writeConnectedComponents("sccp_result4.txt");
}