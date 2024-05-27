#ifndef GRID_H_INCLUDED
#define GRID_H_INCLUDED

#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <queue>
#include <stack>
#include <climits> // For INT_MAX

using namespace std;

// Struct for output of calculatePath

struct Path{
    std::vector<int> path;
    int dead_tile;
};


// visiting status of Tile
enum class Visited {UNVISITED, VISITED};

class Tile
{
    private:

        int id;                     // number of grid
        std::vector<int> neighbors; // neighbor vector of one tile

    public:
        // Constructors
        Tile();
        Tile(int id, std::vector<int>& neighbors);
        // Return ID
        int getId();
        // Return neighbors
        std::vector<int>& getNeighbors();
        // Status of Tile: visited or unvisited
        Visited status;
        // Weight of the crossing this tile
        int weight = 0;
        // Return status
        int getStatus();
};

class Grid
{
    private:
        // Tiles of grid
        std::vector<Tile> tiles;

        // Edge number
        int N;

    public:

        // Constructors
        Grid();
        Grid(int N);

        // Visited status vector
        std::vector<int> unvisited_tiles;

        // Base Tiles
        std::vector<int> base_tiles = {0};

        // Tile functions
        void addTile(int id, std::vector<int>& neighbors);

        // add Obstacle (remove Tile due to infinity weight)
        void addObstacle(int id);

        // Return tile
        Tile* getTile(int id);

        // Print grid
        void printGrid();

        // Change tile status
        void changeTileStatus(int id,Visited status);

        // Get tile status
        int getTileStatus(int id);

        // Get length of tiles
        int getTileNumber();

        // Calculate shortest path
        Path calculatePath(int source, int destination);

        //Check the tile is in the grid
        bool isInSubgrid(int tile);

        // Integer vector of Tile's id's
        std::vector<int> Tiles_IDs_vector();
};

//**** HELPER FUNCTIONS ****//
int Find(vector<int> vec, int element) {

    int L = vec.size();

    for(int i=0; i<L; i++) {

        if(vec[i] == element)
            return i;
    }

    return -1;
}
// The function to get index value of key in the vector
int getTileIndex(vector<Tile> tiles, int key)
{
    int index = 0;
    for (Tile& tile : tiles)
    {
        if (tile.getId() == key) // If key is found
            return index;
        index++;
    }

    // If the element is not found
    return -1;
}

// Remove zeros function for the path finding algorithm
void removeZeros(vector<int>& v)
{
    for (auto it = v.begin(); it != v.end(); )
    {
        if (*it == 0)         // If iterator pointing's value is 0
            it = v.erase(it); // Delete zeros
        else
            it++;             // Increase the iterator pointing
    }
}

// Helper function to remove Grid

void removetilesById(std::vector<Tile>& tiles, int idToRemove) {
    // Find the position of the element to remove
   auto it = std::find_if(tiles.begin(), tiles.end(), [idToRemove](Tile& t) {
        return t.getId() == idToRemove;
    });;

    // Check if the element exists in the vector
    if (it != tiles.end()) {
        // Erase the element from the vector
        tiles.erase(it);
    }
}

void removeElementByValue(std::vector<int>& vec, int elementToRemove) {
    // Find the position of the element to remove
    auto it = find(vec.begin(), vec.end(), elementToRemove);

    // Check if the element exists in the vector
    if (it != vec.end()) {
        // Erase the element from the vector
        vec.erase(it);
    }
}

// ***** Tile Class *****

// Constructors
// Default constructor
Tile::Tile():id(0)
{}
// Constructor
Tile::Tile(int id, vector<int>& neighbors):id(id),neighbors(neighbors),status(Visited::UNVISITED) {}

// the functions getId, getStatus and getNeighbors.
int Tile::getId()
{
    return id;
}

int Tile::getStatus()
{
    if(status == Visited::VISITED)
        return 1;
    else
        return 0;
}

vector<int>& Tile::getNeighbors()
{
    return neighbors;
}


// ***** Grid Class *****

// Constructor
Grid::Grid(int N):N(N) {}

//functions

// Tile's id vector
vector<int> Grid::Tiles_IDs_vector()
{
    vector<int>ids;
    for (Tile& T : tiles){
        ids.push_back(T.getId());
    }
    return ids;
    return ids;
}
// Check the tile is in the grid
bool Grid::isInSubgrid(int tile) {


    int L = tiles.size();

    for(int i=0; i<L; i++) {

        if(tiles[i].getId() == tile)
            return true;
    }

    return false;
}

Tile* Grid::getTile(int id)
{
    // For each Tile in tiles
    for (Tile& Tile : tiles)
    {
        if (Tile.getId() == id) // If the Tile id is equal to id
        {
            return &Tile;
        }
    }
    return nullptr; // If the id is not exists
}

void Grid::addTile(int id, std::vector<int>& neighbors)
{
    bool FLAG_EXIST = true;
    int NeighborId;
    Tile* NeighborTile;
    // Create a new Tile
    Tile NewTile(id,neighbors);
    // Add Tile to tiles vector
    tiles.push_back(NewTile);

    for (int i = 0; i<neighbors.size(); i++)
    {
        FLAG_EXIST = true;
        // Neighbor's Id
        NeighborId     = neighbors[i];
        // Get the neighbor
        NeighborTile   = getTile(NeighborId);
        // If the NeighborTile exists add Tile to its neighbors
        if(NeighborTile != nullptr)
        {
            vector<int> NeighbOfNeigb = NeighborTile->getNeighbors(); // Neighbors vector of neighbor of Tile
            for (int n : NeighbOfNeigb)
            {
                if(n == id) // If the Tile already exists in neighbors of its neighbors, make the flag false
                    FLAG_EXIST = false;
            }
            if (FLAG_EXIST) // If the Tile does not exists in neighbors of its neighbors, add to its neighbors' neighbors
                NeighborTile->getNeighbors().push_back(id);
        }

    }
    unvisited_tiles.push_back(id);
}

void Grid::addObstacle(int id)
{
    // Check for each Tile
    for(Tile& Tile : tiles){
        // Remove Tile from neighbors vector
        vector<int>& neighbors = Tile.getNeighbors();
        // Check each neighbor of Tile's neighbors
        for (int neighbor : neighbors){
            // If removed Tile is found
            if(neighbor == id){
                for (size_t j = 0; j < neighbors.size(); ++j)
                {
                    if (neighbors[j] == id){
                        neighbors.erase(neighbors.begin() + j);
                        break;
                    }
                }
            }
        }

    }
    for(Tile& Tile : tiles){
                // Remove Tile
        if(Tile.getId() == id)
            removetilesById(tiles, id);
            removeElementByValue(unvisited_tiles,id);
            removeElementByValue(base_tiles,id);
    }

}

// Print Grid
void Grid::printGrid()
{
    for(Tile& Tile : tiles)
    {
        // Remove Tile from neighbors vector
        vector<int>& neighbors = Tile.getNeighbors();
        // Check each neighbor of Tile's neighbors
        cout<<"Tile: " << Tile.getId() << " " << "Neigbors: " ;
        for (int neighbor : neighbors)
        {
            cout<< neighbor<<" ";
        }
        cout<< endl;
    }
}
// Grid member function to change Tile visiting status
void Grid::changeTileStatus(int id,Visited status)
{
    // Get tile
    int index   = getTileIndex(tiles,id);
    Tile& t     = tiles[index];
    // Update status
    t.status    = status;

    // If the tile is visited remove it from unvisited vector
    if(status == Visited::VISITED){
        removeElementByValue(unvisited_tiles,id);
        // Update weight
        t.weight    = t.weight + 1;
    }else{
        unvisited_tiles.push_back(id);
    }

}

// Get tile's status with id
int Grid::getTileStatus(int id)
{
    int index = getTileIndex(tiles,id);
    Tile& t = tiles[index];
    cout<<"Tile: " << id <<"Status: " <<t.getStatus()<< endl;
}

// Get tile number
int Grid::getTileNumber()
{
    return tiles.size();
}


// Implementation of calculatePath

Path Grid::calculatePath(int source, int destination) {

    Path output;
    output.dead_tile = -1;

    const int node_num = tiles.size();
    const int visited = 1, unvisited = 0;
    bool mark[node_num];             // mark array to indicate visited/unvisited tiles
    std::vector<int> id(node_num);   // id vector containing id of each node
    int prev[node_num];              // prev array to keep track of previous tiles in the path
    int dist[node_num];
    std::vector<int> path;           // path vector to be returned

    for (int i = 0; i < node_num; i++) {
        mark[i] = unvisited;         // initialize all tiles as unvisited
        id[i] = tiles[i].getId();
        prev[i] = -1;                // initialize all previous node id as -1
        dist[i] = INT_MAX;
    }

    std::queue<int> Q;               // queue for BFS algorithm
    Q.push(source);
    mark[Find(id, source)] = visited; // start with source node
    prev[Find(id, source)] = 0;       // assign its previous node 0 as stopping condition
    dist[Find(id, source)] = 0;

    // Traverse the tiles using breadth-first strategy
    while (!Q.empty()) {
        int w = Q.front();
        Q.pop();

        int w_index = Find(id, w);
        std::vector<int> w_neighbors = tiles[w_index].getNeighbors(); // neighbors of current node

        for (int i = 0; i < w_neighbors.size(); i++) {
            int idx = Find(id, w_neighbors[i]);
            Tile* v_Tile = getTile(w_neighbors[i]);
//            int weight   = (v_Tile->getStatus() == 0 ? 1 : 2);
            int weight   = v_Tile->weight;
            // In case the neighbor has not been visited yet
            if (dist[idx] > dist[w_index] + weight) {
                mark[idx] = visited;        // mark as visited
                prev[idx] = w;              // assign the previous node
                Q.push(w_neighbors[i]);     // insert it to the queue
                dist[idx] = dist[w_index] + weight;
            }
        }
    }

    int v = destination;
    if (prev[Find(id, destination)] == -1){
        output.path      = {0};
        output.dead_tile = destination;
        return output;}

    // In case no possible path exists
    if (prev[Find(id, v)] == -1 || Find(id, source) == -1 || Find(id, destination) == -1){
        output.path = {0};
        output.dead_tile = -1;
        return output;
    }


    // Construct the path vector by prev[] array
    // stop the loop when the source node is reached
    while (v) {
        path.push_back(v);          // push back v to the path vector
        int idx = Find(id, v);
        v = prev[idx];              // assign v to previous node's id for next iteration
    }

    std::reverse(path.begin(), path.end()); // reverse the path to obtain correct order
    output.path = path;
    output.dead_tile = -1;
    return output;
}
#endif // GRID_H_INCLUDED
