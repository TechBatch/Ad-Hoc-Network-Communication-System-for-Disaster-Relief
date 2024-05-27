#ifndef GRID_FUNCTIONS_H_INCLUDED
#define GRID_FUNCTIONS_H_INCLUDED

#include <iostream>
#include "grid.h"
#include <algorithm>

using namespace std;

#include <cmath> // For std::sqrt and std::pow

vector<int> boundary_31 = {44,54,64,74,84};      // boundary 3 1
vector<int> boundary_13 = {43,53,63,73,83};      // boundary 1 3
vector<int> boundary_21 = {4,14,24};             // boundary 2 1
vector<int> boundary_12 = {3,13,23};             // boundary 1 2
vector<int> boundary_32 = {45,46,47,48,49};      // boundary 3 2
vector<int> boundary_23 = {35,36,37,38,39};      // boundary 2 3
vector<int> dead_edges  = {1,9,81,89};           // dead edges


using namespace std;
//****** HELPER FUNCTIONS ******//

void removeElementByValue2(std::vector<int>& vec, int elementToRemove) {
    // Find the position of the element to remove
    auto it = find(vec.begin(), vec.end(), elementToRemove);

    // Check if the element exists in the vector
    if (it != vec.end()) {
        // Erase the element from the vector
        vec.erase(it);
    }
}


int findClosestElement(const vector<int>& elements, int target) {

    // Decode the target position
    int targetRow = target / 10;
    int targetCol = target % 10;

    int closestElement = elements[0];
    double smallestDistance = std::numeric_limits<double>::max();

    // Decode each element's position and compute Euclidean distance to the target position
    for (int element : elements) {
        int elementRow = element / 10;
        int elementCol = element % 10;
        double distance = std::sqrt(std::pow(elementRow - targetRow, 2) + std::pow(elementCol - targetCol, 2));

        if (distance < smallestDistance) {
            closestElement = element;
            smallestDistance = distance;
        }
    }

    return closestElement;
}

// Functions for solve Target position problem if Target is in 2 and has 3 neighbors
struct ThreeNeighbors {
    vector<int> sameElements;         // Indices of the two same elements
    int differentElement;             // Index of the different element
};

// Find where the target is
int which_subgrid(int Target, Grid g1, Grid g2, Grid g3){
    if (g1.isInSubgrid(Target)){
        return 1;
    }
    else if (g2.isInSubgrid(Target)){
        return 2;
    }
    else if (g3.isInSubgrid(Target)){
        return 3;
    }
}

// Find which tile is different other tiles (for 3 tiles)
ThreeNeighbors findDifferentTile(const std::vector<int>& elements,Grid g1, Grid g2, Grid g3) {
    ThreeNeighbors indices;

    if (which_subgrid(elements[0],g1,g2,g3) == which_subgrid(elements[1],g1,g2,g3)) {
        indices.sameElements = {0, 1};
        indices.differentElement = 2;
    } else if (which_subgrid(elements[0],g1,g2,g3) == which_subgrid(elements[2],g1,g2,g3)) {
        indices.sameElements = {0, 2};
        indices.differentElement = 1;
    } else {
        indices.sameElements = {1, 2};
        indices.differentElement = 0;
    }

    return indices;
}



bool isDeadInBoundary(const std::vector<int>& boundary, int dead_tile) {
    return std::find(boundary.begin(), boundary.end(), dead_tile) != boundary.end();
}

void changePathVisited (Grid& g, vector<int> path){
    for (int p: path)
    {
        g.changeTileStatus(p,Visited::VISITED);
    }
}
// Update subgrids
void updateGrids(Grid& g1,Grid& g2,Grid& g3,int dead_tile){

    vector<int> neighbor;
    bool is_in_boundary_12 = isDeadInBoundary(boundary_12, dead_tile);
    bool is_in_boundary_13 = isDeadInBoundary(boundary_13, dead_tile);
    bool is_in_boundary_21 = isDeadInBoundary(boundary_21, dead_tile);
    bool is_in_boundary_23 = isDeadInBoundary(boundary_23, dead_tile);
    bool is_in_boundary_31 = isDeadInBoundary(boundary_31, dead_tile);
    bool is_in_boundary_32 = isDeadInBoundary(boundary_32, dead_tile);

    bool is_in_dead_edges  = isDeadInBoundary(dead_edges, dead_tile);

    Tile* t_44, *t_45; // Obstacles blocking base tiles for subgrid 3
    Tile* tile_change;

    // Check tiles 44 45
    t_44 = g3.getTile(44);
    t_45 = g3.getTile(45);

    if (is_in_boundary_12) {
        g1.addObstacle(dead_tile);          // Extract the dead tile from grid
        neighbor = {dead_tile + 1};         // Neighbor vector
        g2.addTile(dead_tile,neighbor);     // Add tile to other grid
    } else if(is_in_boundary_13){

        g1.addObstacle(dead_tile);          // Extract the dead tile from grid
        neighbor = {dead_tile + 1};         // Neighbor vector
        g3.addTile(dead_tile,neighbor);     // Add tile to other grid

    } else if(is_in_boundary_21){

        g2.addObstacle(dead_tile);          // Extract the dead tile from grid
        neighbor = {dead_tile - 1};         // Neighbor vector
        g1.addTile(dead_tile,neighbor);     // Add tile to other grid

    } else if(is_in_boundary_23){

        g2.addObstacle(dead_tile);          // Extract the dead tile from grid
        neighbor = {dead_tile + 10};         // Neighbor vector
        g3.addTile(dead_tile,neighbor);     // Add tile to other grid

    } else if(is_in_boundary_31){

        g3.addObstacle(dead_tile);          // Extract the dead tile from grid
        neighbor = {dead_tile - 1};         // Neighbor vector
        g1.addTile(dead_tile,neighbor);     // Add tile to other grid

    } else if(is_in_boundary_32){

        g3.addObstacle(dead_tile);          // Extract the dead tile from grid
        neighbor = {dead_tile - 10};        // Neighbor vector
        g2.addTile(dead_tile,neighbor);     // Add tile to other grid

    }
    else if(is_in_dead_edges){

        g1.addObstacle(dead_tile);
        g2.addObstacle(dead_tile);
        g3.addObstacle(dead_tile);
    }
    else if ((t_44 == nullptr) && (t_45 == nullptr)){

        for (int tile: boundary_23){
            g2.addObstacle(tile);
        }
        vector<int> n35 = {36};
        vector<int> n36 = {35,37,46};
        vector<int> n37 = {36,38,47};
        vector<int> n38 = {37,39,48};
        vector<int> n39 = {38,49};

        g3.addTile(35,n35);
        g3.addTile(36,n36);
        g3.addTile(37,n37);
        g3.addTile(38,n38);
        g3.addTile(39,n39);

        g3.base_tiles = {35};
    }
    else{
        std::cout<<"No update"<<std::endl;

    }
}

Grid createGrid(int N, int M, int init_id)
{
    /*
        N: number of rows
        M: number of colums
        init_id: initial id of grid
    */

    //Define base 34 ekle
    // int base = 34;
    // Create N by M grid
    int node_number = N*M;

    Grid g(node_number);

    int k = init_id/10; // row number
    int l = init_id%10; // column number

    for (int i = k; i < N+k; i++){
        for (int j = l; j < M+l; j++){

            int id = i*10 + j;         // id of current tile

            if (j == l){              // First column
                if (i == k){          // First tile
                    vector<int> tile = {id+1,id+10};
                    g.addTile(id,tile);

                }
                else if (i == N+k-1){
                    vector<int> tile = {id+1,id-10};
                    g.addTile(id,tile);

                }
                else{
                    vector<int> tile = {id-10,id+1,id+10};
                    g.addTile(id,tile);

                }
            }
            else if(j == M+l-1){          // Last column
                if (i == k){
                    vector<int> tile = {id-1,id+10};
                    g.addTile(id,tile);

                }
                else if(i == N+k-1){
                    vector<int> tile = {id-1,id-10};
                    g.addTile(id,tile);

                }
                else{
                    vector<int> tile = {id-10,id-1,id+10};
                    g.addTile(id,tile);

                }
            }
            else{
                if(i == k)
                {
                    vector<int> tile = {id-1,id+1,id+10};
                    g.addTile(id,tile);

                }
                else if(i == N+k-1){
                    vector<int> tile = {id-10,id-1,id+1};
                    g.addTile(id,tile);

                }
                else{
                    vector<int> tile = {id-10,id-1,id+1,id+10};
                    g.addTile(id,tile);

                }
            }
        }
    }
    return g;
}

// Subgrid partitioning algorithm
Grid createSubGrid(int sub_id,int obs1,int obs2,int obs3)
{

    int base = 34;
    /*
        sub_id : id for 3 different search area
        obs1   : id of first obstacle
        obs2   : id of second obstacle
        obs3   : id of third
        base   : id of base unit
    */
    Grid subGrid(0);
    switch (sub_id)
        {
            case 0:
                {
                subGrid = createGrid(9,9,1);
                break;
                }
            case 1:
                {
                subGrid = createGrid(9,3,1);
                subGrid.base_tiles = {33,23,43};
                break;
                }

            case 2:
                {
                subGrid = createGrid(4,6,4);
                subGrid.base_tiles = {35,24,25};
                break;
                }

            case 3:
                {
                subGrid = createGrid(5,6,44);
                subGrid.base_tiles = {44,45};
                break;
                }
            default:
                cout<<"Wrong subGrid"<<endl;
        }

    // Add obstacles
    subGrid.addObstacle(obs1);
    subGrid.addObstacle(obs2);
    subGrid.addObstacle(obs3);
    // Add base
    subGrid.addObstacle(34);


    return subGrid;

}


Path func(Grid subgrid,int init_loc) {


    // Mark the starting point as VISITED
    subgrid.changeTileStatus(init_loc,Visited::VISITED);
    // Get unvisited tiles for the searching
    vector<int> unvisited_tiles = subgrid.unvisited_tiles;

    // Define the paths to hold and return
    vector<int> path;                       // Dijkstra's path
    vector<int> new_path;                   // Paths of each best path candidate
    vector<int> final_path;                 // Return the all searching path

    Path output_calc_path;
    Path output;

    Tile* currentTile;                      // Get current tile to check if it is visited or not
    // Initialize paramaters
    int unvisited_current = 0;              // Number of unvisited tiles of best path candidates
    int unvisited_max     = INT_MIN;
    int start_point       = init_loc;       // Start point for each loop

    while(1){

        // Breaking condition
        if(!unvisited_tiles.size())
            break;

        // For each unvisited tiles calculate the paths
        for(int i : unvisited_tiles) {

            // Calculate path
            output_calc_path = subgrid.calculatePath(start_point,i);
            path             = output_calc_path.path;
            if(path.size() == 1){
                output.path = {};
                output.dead_tile = output_calc_path.dead_tile;
                return output;}
            // Update the parameter for each path
            unvisited_current = 0;

            // For each tile in the path find the number of tiles that are not visited
            for(int j: path){

                // Get current tile to check it's status
                currentTile = subgrid.getTile(j);
                // If it is visited increase the unvisited_current by 1
                if (currentTile->getStatus() == 0)
                    unvisited_current++;
            }
            // If the unvisited tiles of current path is larger than the path having maximum unvisited tiles
            // update parameter and path
            if (unvisited_current > unvisited_max){
                unvisited_max = unvisited_current;
                new_path      = path;
            }
        }

        // After finding the new path, change the status of each tile in the path
        for (int p: new_path){

            subgrid.changeTileStatus(p,Visited::VISITED);
        }

        // Update unvisited tiles
        unvisited_tiles = subgrid.unvisited_tiles;

        // Update parameters for new unvisited_tiles set

        // Update starting point
        start_point = new_path.back();

        // Update unvisited_max to int_min
        unvisited_max     = INT_MIN;

        // Delete extra visiting nodes
        if (unvisited_tiles.size() > 0)
            new_path.pop_back();

        // Concatenate the new path with the final path
        final_path.insert(final_path.end(), new_path.begin(), new_path.end());

    }
    subgrid.addObstacle(init_loc);
    // Update output
    output.path = final_path;
    output.dead_tile = output_calc_path.dead_tile;
    return output;

}
// Base function, tell the MUs target locations
//void GetTargetTiles(int& Target1, int& Target2, int& Target3, int Target,Grid global_grid)
//{
//    Tile* Target_Tile;
//    Tile* Neighb_Tile;
//    Tile* Neigh_Neighb_Tile;
//
//    vector<int> TilesNeighbors; // First Neighbors
//    vector<int> NeighborsNeigh; // Neighbors'neighbors
//    vector<int> NeighNeighNeigh;// Neighbors neighbors'neighbors
//    int size_neighbors;
//
//    Target_Tile     = global_grid.getTile(Target);       // Get Target tile to get its neighbors
//    TilesNeighbors  = Target_Tile->getNeighbors();       // Get neighbors of the tile to assembly
//
//    // Add obstacle tile to avoid find it as a neighbor
//    global_grid.addObstacle(Target);
//
//    sort(TilesNeighbors.begin(), TilesNeighbors.end());  // Sort the neighbors
//    size_neighbors  = TilesNeighbors.size();             // Get size of the neighbors
//
//    cout<<size_neighbors;
//
//    switch (size_neighbors)
//    {
//        case 0:
//        {
//            cout<<"Impossible to get" <<endl;
//            Target1 = -1; // If that is impossible to assemble assign -1
//            Target2 = -1;
//            Target3 = -1;
//
//            break;
//        }
//        case 1:
//        {
//            Neighb_Tile    = global_grid.getTile(TilesNeighbors[0]); // Get neighbor tile to get its neighbors
//            Target1        = TilesNeighbors[0];                      // The nearest neighbor is assigned first MU
//            NeighborsNeigh = Neighb_Tile->getNeighbors();            // Get neighbor's neighbors
//            global_grid.addObstacle(Target1);                        // Add as an obstacle avoid find it as a neighbor
//
//            sort(NeighborsNeigh.begin(), NeighborsNeigh.end());      // Sort the neighbors
//
//            if(NeighborsNeigh.size() == 1){
//                Target2           = NeighborsNeigh[0];                       // Assign second MU target position
//                Neigh_Neighb_Tile = global_grid.getTile(Target2);            // Get this tile to get its neighbors
//                NeighNeighNeigh   = Neigh_Neighb_Tile->getNeighbors();       // Get neighbors of neighbors of neighbors
//                global_grid.addObstacle(Target2);                            // Add as an obstacle avoid find it as a neighbor
//                sort(NeighNeighNeigh.begin(), NeighNeighNeigh.end());        // Sort the neighbors *** BURAYI DUSUN***
//                Target3           = NeighNeighNeigh[0];                      // Assign second MU target position
//
//                // ad global obstacle do not forget
//            }
//            else if(NeighborsNeigh.size() >= 2){
//                Target2           = NeighborsNeigh[0]; // Neighbors first
//                Target3           = NeighborsNeigh[1]; // Neighbors second
//            }
//            break;
//        }
//
//        case 2:
//            {
//                Target1        = TilesNeighbors[0];
//
//                Neighb_Tile    = global_grid.getTile(Target1);               // Get neighbor tile to get target1's neighbors
//                NeighborsNeigh = Neighb_Tile->getNeighbors();                // Get target1's neighbors's neighbors
//                global_grid.addObstacle(Target1);                            // Add as an obstacle avoid find it as a neighbor
//                if(NeighborsNeigh.size() == 1){
//                    Target2             = NeighborsNeigh[0];
//                    Neigh_Neighb_Tile   = global_grid.getTile(Target2);
//                    NeighNeighNeigh     = Neigh_Neighb_Tile->getNeighbors();
//                    global_grid.addObstacle(Target2);                            // Add as an obstacle avoid find it as a neighbor
//                    sort(NeighNeighNeigh.begin(), NeighNeighNeigh.end());          // Sort the neighbors
//                    Target3             = NeighNeighNeigh[0];
//                }
//                else if(NeighborsNeigh.size() >= 2){
//                    Target2        = NeighborsNeigh[0];
//                    Target3        = NeighborsNeigh[1];
//                }
//                else{
//                    Target2        = TilesNeighbors[1];
//                    Neighb_Tile    = global_grid.getTile(Target2);               // Get neighbor tile to get target2's neighbors
//                    NeighNeighNeigh= Neighb_Tile->getNeighbors();                       // Get target2's neighbors
//                    cout<< "Case 2 Target 1 in komsusu yok"<<endl;
//
//                    if(NeighNeighNeigh.size() >= 1){
//                        Target3        = NeighNeighNeigh[0];
//                    }
//                    else {
//                        Target3        = -1;
//                    }
//                }
//
//
//
//                break;
//            }
//        default:
//        {
//            Target1 = TilesNeighbors[0];
//            Target2 = TilesNeighbors[1];
//            Target3 = TilesNeighbors[2];
//        }
//
//    }
//
//
//
//}

void GetTargetTiles(int& Target1, int& Target2, int& Target3, int Target,Grid global_grid,Grid g1, Grid g2, Grid g3)
{
    Tile* Target_Tile;
    Tile* Neighb_Tile;
    Tile* Neigh_Neighb_Tile;

    vector<int> TargetNeighbors; // First Neighbors
    vector<int> NeighborsNeigh; // Neighbors'neighbors
    vector<int> NeighNeighNeigh;// Neighbors neighbors'neighbors

    int size_neighbors;
    int target_subgrid;

    Target_Tile     = global_grid.getTile(Target);       // Get Target tile to get its neighbors
    TargetNeighbors = Target_Tile->getNeighbors();       // Get neighbors of the Target to assembly

    // Add obstacle tile to avoid find it as a neighbor
    global_grid.addObstacle(Target);

    size_neighbors  = TargetNeighbors.size();             // Get size of the neighbors

    // Find where the target is
    target_subgrid = which_subgrid(Target,g1,g2,g3);


    // If the MU find the target in its search area

    //if (target_subgrid == finder){
        switch (size_neighbors)
        {
            case 4:
            {
                if (target_subgrid == 1){
                    if(isDeadInBoundary(boundary_12,Target) || (isDeadInBoundary(boundary_13,Target))){
                        Target1 = Target + 10;  // Finder's location
                        Target2 = Target - 1;   // Second's aim
                        Target3 = Target - 10;   // Last's aim
                    }

                    else{
                        Target1 = Target + 10;  // Finder's location
                        Target2 = Target - 1;   // Second's aim
                        Target3 = Target + 1;   // Last's aim

                    }


                }
                else if (target_subgrid == 3){

                    if(isDeadInBoundary(boundary_32,Target)){
                        Target1 = Target + 10;  // Finder's location
                        Target2 = Target + 1;   // Second's aim
                        Target3 = Target - 1;   // Last's aim
                    }
                    else if(isDeadInBoundary(boundary_31,Target)){
                        Target1 = Target - 10;  // Finder's location
                        Target2 = Target + 1;   // Second's aim
                        Target3 = Target + 10;   // Last's aim
                    }
                    else{

                        Target1 = Target + 10;  // Finder's location
                        Target2 = Target + 1;   // Second's aim
                        Target3 = Target - 1;   // Last's aim
                    }


                }
                else if (target_subgrid == 2){
                    if(isDeadInBoundary(boundary_23,Target)){
                        Target1 = Target - 10;  // Finder's location
                        Target2 = Target + 1;   // Second's aim
                        Target3 = Target - 1;   // Last's aim
                    }
                    else if(isDeadInBoundary(boundary_21,Target)){
                        Target1 = Target - 10;  // Finder's location
                        Target2 = Target + 1;   // Second's aim
                        Target3 = Target + 10;   // Last's aim
                    }
                    else{
                        Target1 = Target + 10;  // Finder's location
                        Target2 = Target + 1;   // Second's aim
                        Target3 = Target - 1;   // Last's aim
                    }


                }

                break;
            }
            case 3:
            {
                 sort(TargetNeighbors.begin(), TargetNeighbors.end());      // Sort the neighbors
                 if (target_subgrid == 1){
                    // Check whether the target is in 13 or 12 boundary
                    if ((isDeadInBoundary(boundary_13,Target)) || (isDeadInBoundary(boundary_12,Target))){
                        // Find which tile is not in subgrid 2
                        ThreeNeighbors indices  =  findDifferentTile(TargetNeighbors,g1,g2,g3);

                        Target1 = TargetNeighbors[indices.sameElements[0]];   // Finder's location
                        Target2 = TargetNeighbors[indices.sameElements[1]];   // Second's aim
                        Target3 = TargetNeighbors[indices.differentElement];  // Assign the last one into the other subgrid
                    }
                    else{
                        Target1 = TargetNeighbors[0];  // Finder's location
                        Target2 = TargetNeighbors[1];  // Second's aim
                        Target3 = TargetNeighbors[2];  // Last's aim
                    }
                 }
                 else if (target_subgrid == 2){
                     // Check whether the target is in 21 or 23 boundary
                    if ((isDeadInBoundary(boundary_21,Target)) || (isDeadInBoundary(boundary_23,Target))){
                        // Find which tile is not in subgrid 2
                        ThreeNeighbors indices  =  findDifferentTile(TargetNeighbors,g1,g2,g3);

                        Target1 = TargetNeighbors[indices.sameElements[0]];   // Finder's location
                        Target2 = TargetNeighbors[indices.sameElements[1]];   // Second's aim
                        Target3 = TargetNeighbors[indices.differentElement];  // Assign the last one into the other subgrid
                    }
                    else{
                        Target1 = TargetNeighbors[0];  // Finder's location
                        Target2 = TargetNeighbors[1];  // Second's aim
                        Target3 = TargetNeighbors[2];  // Last's aim
                    }
                 }
                else if (target_subgrid == 3){
                        // Check whether the target is in 31 or 32 boundary
                        if ((isDeadInBoundary(boundary_31,Target)) || (isDeadInBoundary(boundary_32,Target))){
                        // Find which tile is not in subgrid 3
                        ThreeNeighbors indices  =  findDifferentTile(TargetNeighbors,g1,g2,g3);

                        Target1 = TargetNeighbors[indices.sameElements[0]];   // Finder's location
                        Target2 = TargetNeighbors[indices.sameElements[1]];   // Second's aim
                        Target3 = TargetNeighbors[indices.differentElement];  // Assign the last one into the other subgrid
                    }
                    else{
                        Target1 = TargetNeighbors[0];  // Finder's location
                        Target2 = TargetNeighbors[1];  // Second's aim
                        Target3 = TargetNeighbors[2];  // Last's aim
                    }
                }

                break;
            }

            case 2:
            {
                if (target_subgrid == 1){

                    // Check whether the target is in 13 or 12 boundary
                    if ((isDeadInBoundary(boundary_13,Target)) || (isDeadInBoundary(boundary_12,Target))){
                        // Find which tile is in subgrid 1
                        if (g1.isInSubgrid(TargetNeighbors[0])){
                            Target1 = TargetNeighbors[0];
                            Target2 = TargetNeighbors[1];
                            // Find neighbors of the Target1
                            Neighb_Tile    = global_grid.getTile(Target1);
                            // To avoid find it in neighbors add Target 1 as obstacle

                            NeighborsNeigh = Neighb_Tile->getNeighbors();
                            if (NeighborsNeigh.size() == 3){

                                ThreeNeighbors indices  =  findDifferentTile(NeighborsNeigh,g1,g2,g3);
                                // Find closest one
                                Target3 = findClosestElement({NeighborsNeigh[indices.sameElements[0]],NeighborsNeigh[indices.sameElements[1]]},Target);

                            }
                            else if (NeighborsNeigh.size() == 2){
                                // Assign closest one
                                if((NeighborsNeigh[0] - Target) > (NeighborsNeigh[1] - Target)){
                                    Target3 = NeighborsNeigh[1];
                                }
                                else{
                                    Target3 = NeighborsNeigh[0];
                                }

                            }
                            else if (NeighborsNeigh.size() == 1){
                                  Target3 = NeighborsNeigh[0];
                            }
                            else {
                                Neighb_Tile    = global_grid.getTile(Target2);
                                // To avoid find it in neighbors add Target 1 as obstacle

                                NeighborsNeigh = Neighb_Tile->getNeighbors();
                                if (NeighborsNeigh.size() == 3){

                                    ThreeNeighbors indices  =  findDifferentTile(NeighborsNeigh,g1,g2,g3);
                                    // Find closest one
                                    Target3 = findClosestElement({NeighborsNeigh[indices.sameElements[0]],NeighborsNeigh[indices.sameElements[1]]},Target);

                                }
                                else if (NeighborsNeigh.size() == 2){
                                    // Assign closest one
                                    if((NeighborsNeigh[0] - Target) > (NeighborsNeigh[1] - Target)){
                                        Target3 = NeighborsNeigh[1];
                                    }
                                    else{
                                        Target3 = NeighborsNeigh[0];
                                    }

                                }
                                else if (NeighborsNeigh.size() == 1){
                                      Target3 = NeighborsNeigh[0];
                                }

                            }

                        }
                        else{


                            Target1 = TargetNeighbors[1];
                            Target2 = TargetNeighbors[0];
                            // Find neighbors of the Target1
                            Neighb_Tile    = global_grid.getTile(Target1);
                            // To avoid find it in neighbors add Target 1 as obstacle

                            NeighborsNeigh = Neighb_Tile->getNeighbors();
                            if (NeighborsNeigh.size() == 3){

                                ThreeNeighbors indices  =  findDifferentTile(NeighborsNeigh,g1,g2,g3);
                                // Find closest one
                                Target3 = findClosestElement({NeighborsNeigh[indices.sameElements[0]],NeighborsNeigh[indices.sameElements[1]]},Target);


                            }
                            else if (NeighborsNeigh.size() == 2){
                                // Assign closest one
                                cout<<NeighborsNeigh[0]%10<<endl;
                                if(((NeighborsNeigh[0]%10) - Target) > ((NeighborsNeigh[1]%10) - Target)){
                                    Target3 = NeighborsNeigh[1];
                                }
                                else{
                                    Target3 = NeighborsNeigh[0];
                                }

                            }
                            else if (NeighborsNeigh.size() == 1){
                                    cout<<NeighborsNeigh[0]%10<<endl;
                                  Target3 = NeighborsNeigh[0];
                            }
                        }

                    }
                    else{
                            if(Target == 71){
                                // Sort the vector in descending order using a lambda expression as the comparator
                                std::sort(TargetNeighbors.begin(), TargetNeighbors.end(), [](int a, int b) {
                                    return a > b;  // Comparator for descending order
                            });

                            }
                            // Dummy neighbors to delete element
                            vector <int> Neighbors_dmy = TargetNeighbors;
                            Target1 = findClosestElement(Neighbors_dmy,Target); // Assign closest one as 1
                            Neighbors_dmy.erase(remove(Neighbors_dmy.begin(), Neighbors_dmy.end(), Target1), Neighbors_dmy.end());
                            Target2 = Neighbors_dmy[0];

                            // Find Target 3
                            Tile * Neighb_Tile1 = global_grid.getTile(Target1);
                            Tile * Neighb_Tile2 = global_grid.getTile(Target2);

                            vector<int> neighs1 = Neighb_Tile1->getNeighbors();
                            vector<int> neighs2 = Neighb_Tile2->getNeighbors();

                            // Concatenate neighs2 to the end of neighs1
                            neighs1.insert(neighs1.end(), neighs2.begin(), neighs2.end());
                            // Sort vector ascending order
                            sort(neighs1.begin(), neighs1.end());

                            Target3 = findClosestElement(neighs1,Target);


                    }
                }
                else if (target_subgrid == 2){

                    // Check whether the target is in 23 or 21 boundary
                    if ((isDeadInBoundary(boundary_23,Target)) || (isDeadInBoundary(boundary_21,Target))){
                        // Find which tile is in subgrid 2
                        if (g2.isInSubgrid(TargetNeighbors[0])){
                            Target1 = TargetNeighbors[0];
                            Target2 = TargetNeighbors[1];
                            // Find Target 2
                            Tile * Neighb_Tile1 = global_grid.getTile(Target1);
                            Tile * Neighb_Tile3 = global_grid.getTile(Target2);

                            vector<int> neighs1 = Neighb_Tile1->getNeighbors();
                            vector<int> neighs3 = Neighb_Tile3->getNeighbors();

                            // Concatenate neighs3 to the end of neighs1
                            neighs1.insert(neighs1.end(), neighs3.begin(), neighs3.end());
                            // Sort vector ascending order
                            sort(neighs1.begin(), neighs1.end());
                            Target3 = findClosestElement(neighs1,Target);
                        }
                        else{

                        // Comparator for descending order
                            Target1 = TargetNeighbors[1];
                            Target3 = TargetNeighbors[0];
                            // Find Target 2
                            Tile * Neighb_Tile1 = global_grid.getTile(Target1);
                            Tile * Neighb_Tile3 = global_grid.getTile(Target3);

                            vector<int> neighs1 = Neighb_Tile1->getNeighbors();
                            vector<int> neighs3 = Neighb_Tile3->getNeighbors();

                            // Concatenate neighs3 to the end of neighs1
                            neighs1.insert(neighs1.end(), neighs3.begin(), neighs3.end());
                            // Sort vector ascending order
                            sort(neighs1.begin(), neighs1.end());
                            Target2 = findClosestElement(neighs1,Target);

                        }

                    }
                    else{
                        // Sort the vector in descending order using a lambda expression as the comparator
                        std::sort(TargetNeighbors.begin(), TargetNeighbors.end(), [](int a, int b) {
                        return a > b;
                        });
                        Target1 = TargetNeighbors[0];
                        Target2 = TargetNeighbors[1];
                        // Find Target 3
                        Tile * Neighb_Tile1 = global_grid.getTile(Target1);
                        Tile * Neighb_Tile2 = global_grid.getTile(Target2);

                        vector<int> neighs1 = Neighb_Tile1->getNeighbors();
                        vector<int> neighs2 = Neighb_Tile2->getNeighbors();

                        // Concatenate neighs3 to the end of neighs1
                        neighs1.insert(neighs1.end(), neighs2.begin(), neighs2.end());
                        // Sort vector ascending order
                        sort(neighs1.begin(), neighs1.end());
                        Target3 = findClosestElement(neighs1,Target);

                    }

                }
                else if (target_subgrid == 3){

                    // Check whether the target is in 32 or 31 boundary
                    if ((isDeadInBoundary(boundary_32,Target)) || (isDeadInBoundary(boundary_31,Target))){
                        // Find which tile is in subgrid 3
                        if (g3.isInSubgrid(TargetNeighbors[0])){
                            Target1 = TargetNeighbors[0];
                            Target3 = TargetNeighbors[1];
                            // Find Target 2
                            Tile * Neighb_Tile1 = global_grid.getTile(Target1);
                            Tile * Neighb_Tile3 = global_grid.getTile(Target3);

                            vector<int> neighs1 = Neighb_Tile1->getNeighbors();
                            vector<int> neighs3 = Neighb_Tile3->getNeighbors();

                            // Concatenate neighs3 to the end of neighs1
                            neighs1.insert(neighs1.end(), neighs3.begin(), neighs3.end());
                            // Sort the vector in descending order using a lambda expression as the comparator
                            std::sort(neighs1.begin(), neighs1.end(), [](int a, int b) {
                                return a > b;  // Comparator for descending order
                            });
                            Target2 = findClosestElement(neighs1,Target);
                        }
                        else{
                            Target1 = TargetNeighbors[1];
                            Target3 = TargetNeighbors[0];
                            // Find Target 2
                            Tile * Neighb_Tile1 = global_grid.getTile(Target1);
                            Tile * Neighb_Tile3 = global_grid.getTile(Target3);

                            vector<int> neighs1 = Neighb_Tile1->getNeighbors();
                            vector<int> neighs3 = Neighb_Tile3->getNeighbors();

                            // Concatenate neighs3 to the end of neighs1
                            neighs1.insert(neighs1.end(), neighs3.begin(), neighs3.end());

                            // Sort the vector in descending order using a lambda expression as the comparator
                            std::sort(neighs1.begin(), neighs1.end(), [](int a, int b) {
                                return a > b;  // Comparator for descending order
                            });
                            Target2 = findClosestElement(neighs1,Target);

                        }

                    }
                    else{
                          std::sort(TargetNeighbors.begin(), TargetNeighbors.end(), [](int a, int b) {
                                return a > b;  // Comparator for descending order
                            });

                            Target1 = TargetNeighbors[0];
                            Target2 = TargetNeighbors[1];
                            // Find Target 3
                            Tile * Neighb_Tile1 = global_grid.getTile(Target1);
                            Tile * Neighb_Tile2 = global_grid.getTile(Target2);

                            vector<int> neighs1 = Neighb_Tile1->getNeighbors();
                            vector<int> neighs2 = Neighb_Tile2->getNeighbors();

                            // Concatenate neighs3 to the end of neighs1
                            neighs1.insert(neighs1.end(), neighs2.begin(), neighs2.end());
                            // Sort the vector in descending order using a lambda expression as the comparator
                            std::sort(neighs1.begin(), neighs1.end(), [](int a, int b) {
                                return a > b;  // Comparator for descending order
                            });

                            Target3 = findClosestElement(neighs1,Target);

                    }

                }
                break;
            }
            case 1:
            {

                Target1 = TargetNeighbors[0]; // Assign first's destination
                Neighb_Tile = global_grid.getTile(Target1);
                NeighborsNeigh = Neighb_Tile -> getNeighbors();
                if (NeighborsNeigh.size() != 1){
                    Target2 = findClosestElement(NeighborsNeigh,Target);
                    removeElementByValue2(NeighborsNeigh,Target2);
                    Target3 = findClosestElement(NeighborsNeigh,Target);
                }
                else{
                    global_grid.addObstacle(Target1);
                    Target2 = findClosestElement(NeighborsNeigh,Target);
                    Neigh_Neighb_Tile = global_grid.getTile(Target2);
                    NeighNeighNeigh = Neigh_Neighb_Tile->getNeighbors();
                    Target3 = findClosestElement(NeighNeighNeigh,Target);
                }


                break;
            }
        default:
        {
            Target1 = TargetNeighbors[0];
            Target2 = TargetNeighbors[1];
            Target3 = TargetNeighbors[2];
        }

    }

    }





Path TargetPath(int your_turn,int your_id, int finder,int current_location,int base1,int base2, int base3, int Target1, int Target2, int Target3,Grid gb){
/*
    your_turn: base communication turn
    finde    : Mu who finds the target

*/

    int target;
    Path path1, path2, path3,your_path;

    switch(your_turn)
    {
        case 1:
        {
            target  = Target1;
            your_path = gb.calculatePath(current_location,target);
            cout<<"case1 ";

            break;
        }
        case 2:
        {
            target = Target2;
            if (finder == 1)
            {
                path1 = gb.calculatePath(base1,Target1);
                changePathVisited(gb,path1.path);
                gb.addObstacle(Target1);
                gb.addObstacle(base1);
                your_path = gb.calculatePath(current_location,target);
            }
            else if(finder == 2)
            {
                path2 = gb.calculatePath(base2,Target1);
                changePathVisited(gb,path2.path);
                gb.addObstacle(Target1);
                gb.addObstacle(base2);
                your_path = gb.calculatePath(current_location,target);

            }
            else if(finder == 3)
            {
                path3 = gb.calculatePath(base3,Target1);
                changePathVisited(gb,path3.path);
                gb.addObstacle(Target1);
                gb.addObstacle(base3);
                your_path = gb.calculatePath(current_location,target);
            }
            else{
                cout<<"error in your_turn case 2 in Target path"<< endl;
            }
            break;
        }
        case 3:
        {
            target = Target3;
            if ((finder == 1) && (your_id == 2)){

                path1 = gb.calculatePath(base1,Target1);
                changePathVisited(gb,path1.path);
                gb.addObstacle(Target1);
                gb.addObstacle(base1);
                path3 = gb.calculatePath(base3,Target2);
                changePathVisited(gb,path3.path);
                gb.addObstacle(Target2);
                gb.addObstacle(base3);
                your_path = gb.calculatePath(current_location,target);

            }
            else if ((finder == 1) && (your_id == 3)){
                path1 = gb.calculatePath(base1,Target1);
                changePathVisited(gb,path1.path);
                gb.addObstacle(Target1);
                gb.addObstacle(base1);
                path2 = gb.calculatePath(base2,Target2);
                changePathVisited(gb,path2.path);
                gb.addObstacle(Target2);
                gb.addObstacle(base2);
                your_path = gb.calculatePath(current_location,target);

            }
            else if ((finder == 2) && (your_id == 1)){
                path2 = gb.calculatePath(base2,Target1);
                changePathVisited(gb,path2.path);
                gb.addObstacle(Target1);
                gb.addObstacle(base2);
                path3 = gb.calculatePath(base3,Target2);
                changePathVisited(gb,path3.path);
                gb.addObstacle(Target2);
                gb.addObstacle(base3);
                your_path = gb.calculatePath(current_location,target);


            }
            else if ((finder == 2) && (your_id == 3)){
                path2 = gb.calculatePath(base2,Target1);
                changePathVisited(gb,path2.path);
                gb.addObstacle(Target1);
                gb.addObstacle(base2);
                path1 = gb.calculatePath(base1,Target2);
                changePathVisited(gb,path1.path);
                gb.addObstacle(Target2);
                gb.addObstacle(base1);
                your_path = gb.calculatePath(current_location,target);

            }
            else if ((finder == 3) && (your_id == 1)){
                path3 = gb.calculatePath(base3,Target1);
                changePathVisited(gb,path3.path);
                gb.addObstacle(Target1);
                gb.addObstacle(base3);
                path2 = gb.calculatePath(base2,Target2);
                changePathVisited(gb,path2.path);
                gb.addObstacle(Target2);
                gb.addObstacle(base2);
                your_path = gb.calculatePath(current_location,target);

            }
            else if ((finder == 3) && (your_id == 2)){
                path3 = gb.calculatePath(base3,Target1);
                changePathVisited(gb,path3.path);
                gb.addObstacle(Target1);
                gb.addObstacle(base3);
                path1 = gb.calculatePath(base1,Target2);
                changePathVisited(gb,path1.path);
                gb.addObstacle(Target2);
                gb.addObstacle(base1);
                your_path = gb.calculatePath(current_location,target);

            }
            else{
                cout<< "error in case 3 in Target path"<<endl;
            }

            break;
        }
        default:
            {
                cout<<"default case";
            }

    }
    return your_path;
}
#endif // GRID_FUNCTIONS_H_INCLUDED
