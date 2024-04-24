#ifndef GRID_FUNCTIONS_H_INCLUDED
#define GRID_FUNCTIONS_H_INCLUDED

#include <iostream>
#include "grid.h"

using namespace std;



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
                    g.unvisited_tiles.push_back(id); // Add the new tile to unvisited tiles vector
                }
                else if (i == N+k-1){
                    vector<int> tile = {id+1,id-10};
                    g.addTile(id,tile);
                    g.unvisited_tiles.push_back(id);
                }
                else{
                    vector<int> tile = {id-10,id+1,id+10};
                    g.addTile(id,tile);
                    g.unvisited_tiles.push_back(id);
                }
            }
            else if(j == M+l-1){          // Last column
                if (i == k){
                    vector<int> tile = {id-1,id+10};
                    g.addTile(id,tile);
                    g.unvisited_tiles.push_back(id);
                }
                else if(i == N+k-1){
                    vector<int> tile = {id-1,id-10};
                    g.addTile(id,tile);
                    g.unvisited_tiles.push_back(id);
                }
                else{
                    vector<int> tile = {id-10,id-1,id+10};
                    g.addTile(id,tile);
                    g.unvisited_tiles.push_back(id);
                }
            }
            else{
                if(i == k)
                {
                    vector<int> tile = {id-1,id+1,id+10};
                    g.addTile(id,tile);
                    g.unvisited_tiles.push_back(id);
                }
                else if(i == N+k-1){
                    vector<int> tile = {id-10,id-1,id+1};
                    g.addTile(id,tile);
                    g.unvisited_tiles.push_back(id);
                }
                else{
                    vector<int> tile = {id-10,id-1,id+1,id+10};
                    g.addTile(id,tile);
                    g.unvisited_tiles.push_back(id);
                }
            }
        }
    }
    return g;
}

// Subgrid partitioning algorithm
Grid createSubGrid(int sub_id,int initial,int obs1,int obs2,int obs3)
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
            case 1:
                {
                subGrid = createGrid(9,3,1);
                break;
                }

            case 2:
                {
                subGrid = createGrid(4,6,4);
                break;
                }

            case 3:
                {
                subGrid = createGrid(5,6,44);
                break;
                }
            default:
                cout<<"Wrong subGrid"<<endl;
        }

    // Add obstacles
    subGrid.addObstacle(obs1);
    subGrid.addObstacle(obs2);
    subGrid.addObstacle(obs3);
    subGrid.addObstacle(initial);
    // Add base
    subGrid.addObstacle(34);


    return subGrid;

}


vector<int> func(Grid subgrid,int init_loc) {

  

    // Mark the starting point as VISITED
    subgrid.changeTileStatus(init_loc,Visited::VISITED);
    // Get unvisited tiles for the searching
    vector<int> unvisited_tiles = subgrid.unvisited_tiles;

    // Define the paths to hold and return
    vector<int> path;                       // Dijkstra's path
    vector<int> new_path;                   // Paths of each best path candidate
    vector<int> final_path;                 // Return the all searching path

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
            path = subgrid.calculatePath(start_point,i);
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
    return final_path;

}
#endif // GRID_FUNCTIONS_H_INCLUDED
