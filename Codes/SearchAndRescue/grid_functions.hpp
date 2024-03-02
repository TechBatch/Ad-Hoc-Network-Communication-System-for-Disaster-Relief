#ifndef GRID_FUNCTIONS_HPP_INCLUDED
#define GRID_FUNCTIONS_HPP_INCLUDED

#include <iostream>
#include "grid.hpp"

using namespace std;
Grid createGrid(int N)
{
    // Create N by N grid
    int node_number = N*N;

    Grid g(node_number);

    for (int i = 0; i < node_number; i++)
    {

        if (i%N == 0){

                if(i == 0)
                {
                    vector<int> tile = {i+1+1,i+N+1};
                    g.addTile(i+1,tile);
                }
                else if(i == node_number-N)
                {
                    vector<int> tile = {i-N+1,i+1+1};
                    g.addTile(i+1,tile);
                }
                else
                {
;                   vector<int> tile = {i-N+1,i+1+1,i+N+1};
                    g.addTile(i+1,tile);
                }
        }
        else if(i%N == N-1){

                if (i == N-1){
                    vector<int> tile = {i-1+1,i+N+1};
                    g.addTile(i+1,tile);
                }
                else if (i == node_number-1){
                    vector<int> tile = {i-N+1,i-1+1};
                    g.addTile(i+1,tile);
                }
                else{
                    vector<int> tile = {i-N+1,i-1+1,i+N+1};
                    g.addTile(i+1,tile);
                }

        }
        else{
            if(i < N){
                vector<int> tile = {i-1+1,i+1+1,i+N+1};
                g.addTile(i+1,tile);
            }
            else if(i > node_number-N){
                vector<int> tile = {i-N+1,i-1+1,i+1+1};
                g.addTile(i+1,tile);
            }
            else{
                vector<int> tile = {i-N+1,i-1+1,i+1+1,i+N+1};
                g.addTile(i+1,tile);
            }

        }
    }

    return g;
}


#endif // GRID_FUNCTIONS_HPP_INCLUDED
