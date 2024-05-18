#ifndef GRID_FUNCTIONS_H_INCLUDED
#define GRID_FUNCTIONS_H_INCLUDED

#include <iostream>
#include "grid.h"
#include <algorithm>

using namespace std;

vector<int> boundary_31 = {44,54,64,74,84};      // boundary 3 1
vector<int> boundary_13 = {43,53,63,73,83};      // boundary 1 3
vector<int> boundary_21 = {4,14,24};             // boundary 2 1
vector<int> boundary_12 = {3,13,23};             // boundary 1 2
vector<int> boundary_32 = {45,46,47,48,49};      // boundary 3 2
vector<int> boundary_23 = {35,36,37,38,39};   // boundary 2 3
vector<int> dead_edges  = {1,9,81,89};           // dead edges


using namespace std;

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
                subGrid.base_tiles = {24,35,25};
                break;
                }

            case 3:
                {
                subGrid = createGrid(5,6,44);
                subGrid.base_tiles = {44,45,54};
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
void GetTargetTiles(int& Target1, int& Target2, int& Target3, int Target,Grid global_grid)
{
    Tile* Target_Tile;
    Tile* Neighb_Tile;
    Tile* Neigh_Neighb_Tile;

    vector<int> TilesNeighbors; // First Neighbors
    vector<int> NeighborsNeigh; // Neighbors'neighbors
    vector<int> NeighNeighNeigh;// Neighbors neighbors'neighbors
    int size_neighbors;

    Target_Tile     = global_grid.getTile(Target);       // Get Target tile to get its neighbors
    TilesNeighbors  = Target_Tile->getNeighbors();       // Get neighbors of the tile to assembly

    // Add obstacle tile to avoid find it as a neighbor
    global_grid.addObstacle(Target);

    sort(TilesNeighbors.begin(), TilesNeighbors.end());  // Sort the neighbors
    size_neighbors  = TilesNeighbors.size();             // Get size of the neighbors

    cout<<size_neighbors;

    switch (size_neighbors)
    {
        case 0:
        {
            cout<<"Impossible to get" <<endl;
            Target1 = -1; // If that is impossible to assemble assign -1
            Target2 = -1;
            Target3 = -1;

            break;
        }
        case 1:
        {
            Neighb_Tile    = global_grid.getTile(TilesNeighbors[0]); // Get neighbor tile to get its neighbors
            Target1        = TilesNeighbors[0];                      // The nearest neighbor is assigned first MU
            NeighborsNeigh = Neighb_Tile->getNeighbors();            // Get neighbor's neighbors
            global_grid.addObstacle(Target1);                        // Add as an obstacle avoid find it as a neighbor

            sort(NeighborsNeigh.begin(), NeighborsNeigh.end());      // Sort the neighbors

            if(NeighborsNeigh.size() == 1){
                Target2           = NeighborsNeigh[0];                       // Assign second MU target position
                Neigh_Neighb_Tile = global_grid.getTile(Target2);            // Get this tile to get its neighbors
                NeighNeighNeigh   = Neigh_Neighb_Tile->getNeighbors();       // Get neighbors of neighbors of neighbors
                global_grid.addObstacle(Target2);                            // Add as an obstacle avoid find it as a neighbor
                sort(NeighNeighNeigh.begin(), NeighNeighNeigh.end());        // Sort the neighbors *** BURAYI DUSUN***
                Target3           = NeighNeighNeigh[0];                      // Assign second MU target position

                // ad global obstacle do not forget
            }
            else if(NeighborsNeigh.size() >= 2){
                Target2           = NeighborsNeigh[0]; // Neighbors first
                Target3           = NeighborsNeigh[1]; // Neighbors second
            }
            break;
        }

        case 2:
            {
                Target1        = TilesNeighbors[0];

                Neighb_Tile    = global_grid.getTile(Target1);               // Get neighbor tile to get target1's neighbors
                NeighborsNeigh = Neighb_Tile->getNeighbors();                // Get target1's neighbors's neighbors
                global_grid.addObstacle(Target1);                            // Add as an obstacle avoid find it as a neighbor
                if(NeighborsNeigh.size() == 1){
                    Target2             = NeighborsNeigh[0];
                    Neigh_Neighb_Tile   = global_grid.getTile(Target2);
                    NeighNeighNeigh     = Neigh_Neighb_Tile->getNeighbors();
                    global_grid.addObstacle(Target2);                            // Add as an obstacle avoid find it as a neighbor
                    sort(NeighNeighNeigh.begin(), NeighNeighNeigh.end());          // Sort the neighbors
                    Target3             = NeighNeighNeigh[0];
                }
                else if(NeighborsNeigh.size() >= 2){
                    Target2        = NeighborsNeigh[0];
                    Target3        = NeighborsNeigh[1];
                }
                else{
                    Target2        = TilesNeighbors[1];
                    Neighb_Tile    = global_grid.getTile(Target2);               // Get neighbor tile to get target2's neighbors
                    NeighNeighNeigh= Neighb_Tile->getNeighbors();                       // Get target2's neighbors
                    cout<< "Case 2 Target 1 in komsusu yok"<<endl;

                    if(NeighNeighNeigh.size() >= 1){
                        Target3        = NeighNeighNeigh[0];
                    }
                    else {
                        Target3        = -1;
                    }
                }



                break;
            }
        default:
        {
            Target1 = TilesNeighbors[0];
            Target2 = TilesNeighbors[1];
            Target3 = TilesNeighbors[2];
        }

    }



}
Path TargetPath(int your_turn,int your_id, int finder,int current_location,int base1,int base2, int base3, int Target1, int Target2, int Target3,Grid gb){
/*
    your_turn: base communication turn
    finder    : Mu who finds the target
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
                your_path = gb.calculatePath(current_location,target);
            }
            else if(finder == 2)
            {
                path2 = gb.calculatePath(base2,Target1);
                changePathVisited(gb,path2.path);
                gb.addObstacle(Target1);
                your_path = gb.calculatePath(current_location,target);
            }
            else if(finder == 3)
            {
                path3 = gb.calculatePath(base3,Target1);
                changePathVisited(gb,path3.path);
                gb.addObstacle(Target1);
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
                path3 = gb.calculatePath(base3,Target2);
                changePathVisited(gb,path3.path);
                gb.addObstacle(Target2);
                your_path = gb.calculatePath(current_location,target);
            }
            else if ((finder == 1) && (your_id == 3)){
                path1 = gb.calculatePath(base1,Target1);
                changePathVisited(gb,path1.path);
                gb.addObstacle(Target1);
                path2 = gb.calculatePath(base2,Target2);
                changePathVisited(gb,path2.path);
                gb.addObstacle(Target2);
                your_path = gb.calculatePath(current_location,target);

            }
            else if ((finder == 2) && (your_id == 1)){
                path2 = gb.calculatePath(base2,Target1);
                changePathVisited(gb,path2.path);
                gb.addObstacle(Target1);
                path3 = gb.calculatePath(base3,Target2);
                changePathVisited(gb,path3.path);
                gb.addObstacle(Target2);
                your_path = gb.calculatePath(current_location,target);

            }
            else if ((finder == 2) && (your_id == 3)){
                path2 = gb.calculatePath(base2,Target1);
                changePathVisited(gb,path2.path);
                gb.addObstacle(Target1);
                path1 = gb.calculatePath(base1,Target2);
                changePathVisited(gb,path1.path);
                gb.addObstacle(Target2);
                your_path = gb.calculatePath(current_location,target);

            }
            else if ((finder == 3) && (your_id == 1)){
                path3 = gb.calculatePath(base3,Target1);
                changePathVisited(gb,path3.path);
                gb.addObstacle(Target1);
                path2 = gb.calculatePath(base2,Target2);
                changePathVisited(gb,path2.path);
                gb.addObstacle(Target2);
                your_path = gb.calculatePath(current_location,target);

            }
            else if ((finder == 3) && (your_id == 2)){
                path3 = gb.calculatePath(base3,Target1);
                changePathVisited(gb,path3.path);
                gb.addObstacle(Target1);
                path1 = gb.calculatePath(base1,Target2);
                changePathVisited(gb,path1.path);
                gb.addObstacle(Target2);
                your_path = gb.calculatePath(current_location,target);

            }
            else{
                cout<< "error in case 3 in Target pathh"<<endl;
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
