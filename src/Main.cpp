#include "SimplePathPlanner.hpp"

#include <iostream>

int main(int argc, char* argv[]) {

    int width = 10, height = 10;
    nav_graph_search::TraversabilityMap map(width, height, Eigen::Affine3d::Identity(), 0x00);

    std::vector<uint8_t> trav_values;
    for(int y=0; y < map.ySize(); y++) {
        for(int x=0; x < map.xSize(); x++) {
            trav_values.push_back((uint8_t)x+2);
        }
    }
    
    map.fill(trav_values);

    nav_graph_search::TerrainClasses terrain_classes;
    struct nav_graph_search::TerrainClass terrain_class;

    // cost -> 0: map_scale / 0.3 
    //         1: 1000000
    //         2: map_scale / 0.083 
    // to 
    //        12: map_scale / 0.83
    for(int i=1; i<12; i++) {
        terrain_class.in = terrain_class.out = i;
        terrain_class.cost = (i-1) / 12.0; 
        terrain_classes.push_back(terrain_class);
    }
    terrain_class.in = terrain_class.out = 0; // Unknown.
    terrain_class.cost = 3.5 / 12;
    terrain_classes.push_back(terrain_class);

    SimplePathPlanner::SimplePathPlanner planner(map, terrain_classes, 0, false);
    planner.setStartPositionLocal(Eigen::Vector2i(1,1));
    planner.setGoalPositionLocal(Eigen::Vector2i(8,8));

    planner.calculateTrajectory();

    planner.printInformations();

    // Mirror slope and recalculate.
    for(int y = 0; y < map.ySize(); ++y) {
        for(int x = 0; x < map.xSize() / 2; ++x) {
            uint8_t class_tmp_left; planner.getCell(x,y, class_tmp_left);
            uint8_t class_tmp_right; planner.getCell(map.xSize() - 1 - x, y, class_tmp_right);
            planner.updateCellEnvire(x, y, class_tmp_right); // just for testing, np because in == out
            planner.updateCell(map.xSize() - 1 - x, y, class_tmp_left);
        }
    }

    planner.calculateTrajectory();

    planner.printInformations();
   
    return 0;
}
