#ifndef GRAPH_HPP_INCLUDED
#define GRAPH_HPP_INCLUDED
#include <cassert>
#include <iostream>


using namespace std;

enum visited {UNVISITED, VISITED};

class Graph
{
    private:
        int numVertex, numEdge;                 // number of edges and vertices
        int **matrix;                           // pointer for adjacency matrix
        int *mark;                              // array of visited nodes

    public:
        Graph(int n);                           // Constructor
        ~Graph();                               // Destructor

        int n() const;                          // number of vertices
        int e() const;                          // number of edges

        int first(int v);                        // first neighbor of vertex v
        int next(int v, int w);                  // v's next neighbor after w

        void setEdge(int v1, int v2, int wt);   // set edge with weight wt
        void delEdge(int v1, int v2);           // delete edge
        bool isEdge(int v1, int v2);            // is (v1,v2) an edge?

        int weight(int v1, int v2);             // get the weight of (v1,v2)
        int getMark(int v);                     // get the mark of vertex v
        void setMark(int v, int val);           // set the mark
        void clearMark();                       // clear all marks

        // Function to return the adjacency matrix
        int** getMatrix() const {
            return matrix;
        }

        // Function to print the adjacency matrix
        void printMatrix() const {

        cout << "Adjacency Matrix:" << endl;
        for (int i = 0; i < numVertex; ++i) {

            for (int j = 0; j < numVertex; ++j) {

                cout << matrix[i][j] << " ";
            }
            cout << endl;
            }
        }

        // Function to print the graph
        void printGraph() const {

            cout << "Graph:" << endl;
            for (int i = 0; i < numVertex; ++i) {
                for (int j = 0; j < numVertex; ++j) {
                    if (matrix[i][j] != 0) {
                        cout << i << " --> " << j << endl;
                    }
                }
            }
    }
};

// Constructor
Graph::Graph(int n){

    int i;

    numVertex = n;
    numEdge   = 0;

    // Initialize mark array
    mark      = new int[numVertex];

    for (i = 0; i < numVertex; i++)
        mark[i] = UNVISITED;
    // Make adjacency matrix

    matrix    = new int*[numVertex];
    for (i = 0; i < numVertex; i++)
        matrix[i] = new int[numVertex];
    for (i = 0; i < numVertex; i++)
        for (int j = 0; j < numVertex; j++)
            matrix[i][j] = 0;
}

// Destructor
Graph::~Graph(){
    delete[] mark;
    for (int i = 0; i < numVertex; i++)
        delete[] matrix[i];
    delete [] matrix;
}

// Return number of vertices
int Graph::n() const{
    return numVertex;
}

// Return number of edges
int Graph::e() const{
    return numEdge;
}

// Return first neighbor of v
int Graph::first(int v){
    for (int i = 0; i < numVertex; i++)
        if(matrix[v][i] != 0)
            return i;
    return numVertex;  // Return n if none
}

// Return v's next neighbor after w
int Graph::next(int v, int w){
    for (int i = w+1; i < numVertex; i++)
        if(matrix[v][i] != 0)
            return i;
    return numVertex;   // Return n if none
}

// Set edge
void Graph::setEdge(int v1, int v2, int wt){
    assert(wt > 0);

    if (matrix[v1][v2] == 0)
        numEdge++;
    matrix[v1][v2]  = wt;
}

// Delete edge
void Graph::delEdge(int v1, int v2){
    if (matrix[v1][v2] != 0)
        numEdge--;
    matrix[v1][v2] = 0;
}

// Is edge
bool Graph::isEdge(int i, int j){
    return matrix[i][j] != 0;
}

// Return weight
int Graph::weight(int v1, int v2){
    return matrix[v1][v2];
}

// Get mark of vertex v
int Graph::getMark(int v){
    return mark[v];
}

// Set mark of vertex v
void Graph::setMark(int v, int val){
    mark[v] = val;
}

// Clear all marks
void Graph::clearMark(void){
    for(int i = 0; i < numVertex; i++)
        mark[i] = UNVISITED;
}

//Graph createGrid(int N)
//{
//    // Create N by N grid
//    int node_number = N*N;
//
//    Graph g(node_number);
//
//    for (int i = 0; i < node_number; i++)
//    {
//
//        if (i%N == 0){
//
//                if(i == 0)
//                {
//                    g.setEdge(i,i+1,1);
//                    g.setEdge(i,i+N,1);
//                }
//                else if(i == node_number-N)
//                {
//                    g.setEdge(i,i+1,1);
//                    g.setEdge(i,i-N,1);
//                }
//                else
//                {
//                    g.setEdge(i,i+1,1);
//                    g.setEdge(i,i+N,1);
//                    g.setEdge(i,i-N,1);
//                }
//        }
//        else if(i%N == N-1){
//
//                if (i == N-1){
//                    g.setEdge(i,i-1,1);
//                    g.setEdge(i,i+N,1);
//                }
//                else if (i == node_number-1){
//                    g.setEdge(i,i-1,1);
//                    g.setEdge(i,i-N,1);
//                }
//                else{
//                    g.setEdge(i,i-1,1);
//                    g.setEdge(i,i+N,1);
//                    g.setEdge(i,i-N,1);
//                }
//
//        }
//        else{
//            if(i < N){
//                g.setEdge(i,i-1,1);
//                g.setEdge(i,i+1,1);
//                g.setEdge(i,i+N,1);
//            }
//            else if(i > node_number-N){
//                g.setEdge(i,i-1,1);
//                g.setEdge(i,i+1,1);
//                g.setEdge(i,i-N,1);
//            }
//            else{
//                g.setEdge(i,i-1,1);
//                g.setEdge(i,i+1,1);
//                g.setEdge(i,i-N,1);
//                g.setEdge(i,i+N,1);
//            }
//
//        }
//    }
//
//    return g;
//}



#endif // GRAPH_HPP_INCLUDED
