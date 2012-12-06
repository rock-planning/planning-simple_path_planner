#include "SimplePathPlanner.hpp"

// PUBLIC
SimplePathPlanner::SimplePathPlanner(nav_graph_search::TraversabilityMap& trav_map,
        std::list<nav_graph_search::TerrainClass> terrain_classes,
        int robot_footprint,
        bool inflate_max) : 
                mpTraversabilityMap(NULL),
                mpDStar(NULL),
                mTerrainClasses(),
                mTerrainClassInToOut(),
                mStartPos(0,0), 
                mGoalPos(0,0),
                mTrajectory()
{
    // Create internal copy of the passed traversability map.
    mpTraversabilityMap = new nav_graph_search::TraversabilityMap(trav_map);
    mTerrainClasses = terrain_classes;
    mpDStar = new nav_graph_search::DStar(*mpTraversabilityMap, 
            mTerrainClasses, 
            robot_footprint, 
            inflate_max);

    // Create lookup table for terrain classes.
    std::list<nav_graph_search::TerrainClass>::iterator it = mTerrainClasses.begin();
    for(; it != mTerrainClasses.end(); ++it) {
        mTerrainClassInToOut.insert(std::pair<int,int>(it->in, it->out));
    }
}

SimplePathPlanner::~SimplePathPlanner()
{
    if(mpDStar != NULL) {
        delete mpDStar; mpDStar = NULL;
    }
    if(mpTraversabilityMap != NULL) {
        delete mpTraversabilityMap; mpTraversabilityMap = NULL;
    }
}

bool SimplePathPlanner::setStartPositionWorld(Eigen::Vector3d pos) {
    nav_graph_search::PointID point = mpTraversabilityMap->toLocal(pos);
    Eigen::Vector2i eigen_point(point.x, point.y);
    return setStartPositionLocal(eigen_point);
}

bool SimplePathPlanner::setStartPositionLocal(Eigen::Vector2i pos) {
    if(pos[0] >= 0 && pos[0] < mpTraversabilityMap->xSize() &&
            pos[1] >= 0 && pos[1] < mpTraversabilityMap->ySize()) {
        mStartPos = pos;
        LOG_INFO("Set start position to (%d,%d)", pos[0], pos[1]);
        return true;
    } else {
        LOG_WARN("Start position could not be set, (%d,%d) is not located within the grid",pos[0],pos[1]);
        return false;
    }
}

bool SimplePathPlanner::setGoalPositionWorld(Eigen::Vector3d pos) {
    nav_graph_search::PointID point = mpTraversabilityMap->toLocal(pos);
    Eigen::Vector2i eigen_point(point.x, point.y);
    return setGoalPositionLocal(eigen_point);
}

bool SimplePathPlanner::setGoalPositionLocal(Eigen::Vector2i pos) {
    if(pos[0] >= 0 && pos[0] < mpTraversabilityMap->xSize() &&
            pos[1] >= 0 && pos[1] < mpTraversabilityMap->ySize()) {
        mGoalPos = pos;
        LOG_INFO("Set goal position to (%d,%d)", pos[0], pos[1]);
        return true;
    } else {
        LOG_WARN("Goal position could not be set, (%d,%d) is not located within the grid",pos[0],pos[1]);
        return false;
    }
}

bool SimplePathPlanner::toLocal(double x, double y, size_t& xi, size_t& yi) {
    nav_graph_search::PointID point = mpTraversabilityMap->toLocal(Eigen::Vector3d(x,y,0));
    if(point.x >= 0 && point.x < mpTraversabilityMap->xSize() &&
            point.y >= 0 && point.y < mpTraversabilityMap->ySize()) {
        xi = point.x;
        yi = point.y;
        return true;
    } else {
        LOG_INFO("Point (%d,%d) (world) is not located within the grid", x, y);
        return false;
    }
}

bool SimplePathPlanner::updateCell(size_t xi, size_t yi, uint8_t value) {
    if((long)xi < mpTraversabilityMap->xSize() && (long)yi < mpTraversabilityMap->ySize()) {
        mpTraversabilityMap->setValue(xi, yi, value);
        mpDStar->setTraversability(xi,yi,value);
        mpDStar->updated(xi, yi);
        return true;
    } else {
        LOG_WARN("Cell could not be updated, (%d,%d) is not located within the grid");
        return false;
    }
}

bool SimplePathPlanner::updateCellEnvire(size_t xi, size_t yi, uint8_t envire_class_value) {
    std::map<int, int>::iterator it = mTerrainClassInToOut.find(envire_class_value);
    if(it == mTerrainClassInToOut.end()) {
        LOG_WARN("Terrain class %d unknown, cell has not been updated", (int)envire_class_value);
        return false;
    }
    return updateCell(xi, yi, it->second);
}

bool SimplePathPlanner::getCell(size_t const xi, size_t const yi, uint8_t& class_value) {
    if((int)xi > mpTraversabilityMap->xSize() - 1 || (int)yi > mpTraversabilityMap->ySize() - 1) {
        LOG_WARN("Cell (%d,%d) is not located within the grid, class value 0 is returned", xi, yi);
        return false;
    }
    class_value = mpTraversabilityMap->getValue(xi, yi);
    return true;
}

bool SimplePathPlanner::calculateTrajectory() {

    if(mGoalPos[0] == mStartPos[0] &&  mGoalPos[1] == mStartPos[1]) {
        return true;
    }

    // Tries to calculate the trajectory using 'max_cost == 0'.
    double ret = mpDStar->run(mGoalPos[0], mGoalPos[1], mStartPos[0], mStartPos[1]);
    if(ret == base::unknown<double>()) {
        LOG_INFO("Trajectory could not be calculated");
        return false;
    } else {
        LOG_INFO("Trajectory found, calculated cost: %4.2f", ret);
    }

    // Fill mTrajectory.
    mTrajectory.clear();

    nav_graph_search::GridGraph const& grid_graph = mpDStar->graph();
    nav_graph_search::NeighbourConstIterator parent;
    size_t xi = mStartPos[0], yi = mStartPos[1];
    size_t next_xi = 0, next_yi = 0;
    addVectorToTrajectory(xi,yi);

    // Run through the trajectory from goal to start.
    do {
        parent = grid_graph.parentsBegin(xi, yi); // Next optimal neighbour patch/cell to (xi,yi).
        next_xi = parent.x(); 
        next_yi = parent.y();

        if((int)next_xi > mpTraversabilityMap->xSize() - 1 || (int)next_yi > mpTraversabilityMap->ySize() - 1) {
            LOG_ERROR("Patch (%d,%d) is not located within the map, erroneous trajectory found!");
            return false;
        }

        if(next_xi == xi && next_yi == yi) { // Stucked.
            double distance_to_goal = sqrt(pow((mGoalPos[0] - xi),2) + pow((mGoalPos[1] - yi),2));
            LOG_WARN("Trajectory not found, stucked at (%d,%d), still %4.2f patches to go", 
                    next_xi, next_yi, distance_to_goal);
            return false;
        }    

        addVectorToTrajectory(next_xi, next_yi);

        if(next_xi == (size_t)mGoalPos[0] && next_yi == (size_t)mGoalPos[1]) { // Trajectory found.
            LOG_INFO("Trajectory found containing %d waypoints", mTrajectory.size());
            return true;
        }

        xi = next_xi; yi = next_yi;
    } while(!parent.isEnd());

    LOG_WARN("Start position not reached and no further parent-patches available");
    return false;
}

std::vector<base::Vector3d> SimplePathPlanner::getTrajectory() {
    return mTrajectory;
}

void SimplePathPlanner::printInformations() {
    std::cout << "Cost of classes: " << std::endl;
    for (int i = 0; i < nav_graph_search::TraversabilityMap::CLASSES_COUNT; ++i) {
        std::cout << std::setw(2) << i << " " << mpDStar->costOfClass(i) << std::endl;
    }        

    std::cout << "Trajectory from start to goal:" << std::endl;
    base::Vector3d v;
    for(unsigned int i=0; i < mTrajectory.size(); ++i) {
        v = mTrajectory[i];
        std::cout << "(" << v[0] << ", " << v[1] << ", " << v[2] << ") " << std::endl; 
    }

    std::cout << "Store current traversability map to trav_map.ppm" << std::endl;
    mpTraversabilityMap->writeToPPM("trav_map");
    int num = 4;
    std::cout << "First " << num << " patch-values (0,0), (1,0), ...: " << std::endl;
    for(int i=0; i<4; ++i) {
        std::cout << (int)mpTraversabilityMap->getValue(i, 0) << " ";
    }
    std::cout << " " << std::endl;
}

// PRIVATE
void SimplePathPlanner::addVectorToTrajectory(size_t xi, size_t yi) {
    nav_graph_search::PointID point(xi, yi);
    base::Vector3d vector = mpTraversabilityMap->toWorld(point);
    mTrajectory.push_back(vector);
    LOG_DEBUG("Added waypoint to trajectory, local (%d,%d) transformed to world (%4.2f,%4.2f,%4.2f)",
            xi, yi, vector[0], vector[1], vector[2]);
}
