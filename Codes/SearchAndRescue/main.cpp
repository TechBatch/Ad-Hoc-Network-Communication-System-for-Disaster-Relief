#include <iostream>
#include "graph.hpp"
#include "grid.hpp"
#include "grid_functions.hpp"


using namespace std;
int main()
{

    Grid g(2);
    g = createGrid(2);
   // g.printGrid();
    g.changeTileStatus(1,Visited::VISITED);
    g.getTileStatus(1);


    return 0;
}




