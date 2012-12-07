#ifndef _PATH_PLANNER_H_
#define _PATH_PLANNER_H_

#include <iostream>
#include <string>
#include <vector>

#include <base/eigen.h>
#include <base/logging.h>

#include <nav_graph_search/traversability_map.hpp>
#include <nav_graph_search/dstar.hpp>
#include <nav_graph_search/terrain_classes.hpp>
#include <nav_graph_search/point.hpp>

class SimplePathPlanner {
 public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * Initializes the DStar search algorithm using the passed parameters.
     * All other parameters (such as grid size, grid content and transformations)
     * are included in the traversability map.
     * If the default paramters are used, every patch get the cost value of 'terrain_classes'.
     * \param trav_map An internal copy of the passed object will be created. 
     * \param terrain_classes Only the parameters 'in' and 'out' (can get the same value) and 'cost' 
     * (calculated with x / cost) have to be used.\n
     * In general as simpler the terrain as bigger the cost value should be, e.g. class 2: 0.083, class 12: 0.83.\n
     * The cost of obstacles (class 1) have to be set to 0 (which will lead to an internal cost of 1.000.000), 
     * the cost of unknown patches (class 0) should not be too low to enforce exploration.
     * \param robot_footprint If != 0, the footprint of the robot will be taken into account.
     * \param inflate_max (?)"Allowes cost margin, as a rate w.r.t. the optimal cost (i.e. 0.9 means 10% below optimal)"
     */
    SimplePathPlanner(nav_graph_search::TraversabilityMap& trav_map,
            std::list<nav_graph_search::TerrainClass> terrain_classes,
            int robot_footprint = 0,
            bool inflate_max = false);

    ~SimplePathPlanner();

    /**
     * Transforms the point in the world to the raster / traversability map.
     * Done with: \n
     * Eigen::Affine3d local_to_world = 
     *      env.relativeTransform(trav_map.getFrameNode(), env.getRootNode()) * 
     *      Eigen::Translation3d( map.getOffsetX(), map.getOffsetY(), 0 ) \n
     * Eigen::Affine3d world_to_local = local_to_world.invers()
     */
    bool setStartPositionWorld(Eigen::Vector3d pos);

    bool setStartPositionLocal(Eigen::Vector2i pos);

    /**
     * See setStartPositionWorld() for details.
     */ 
    bool setGoalPositionWorld(Eigen::Vector3d pos);

    bool setGoalPositionLocal(Eigen::Vector2i pos);

    /**
     * Transforms the point in the world to the local grid.
     * If the point is not within the grid, false is returned.
     */
    bool toLocal(double x, double y, size_t& xi, size_t& yi);
    

    /**
     * Updates the cell in the nav-graph-search traversability map.
     * \param class_value Class value used in the nav-graph-search
     * traversability map. The envire class values can be used directly, if
     * the same values have been used for 'TerrainClass::in' and 'TerrainClass::out'.
     */
    bool updateCell(size_t xi, size_t yi, uint8_t class_value);

    /**
     * Converts the envire class value to the nav-grap-search class value
     * and calls 'updateCell()'.  
     */
    bool updateCellEnvire(size_t xi, size_t yi, uint8_t envire_class_value);

    /**
     * Sets the passed class_value parameter to the class value of the nav_graph_search::TraversabilityMap.
     * \return false if (xi, yi) is not located within the grid.
     */
    bool getCell(size_t const xi, size_t const yi, uint8_t& class_value);

    bool calculateTrajectory();

    /**
     * Returns the trajectory containing waypoints from start to goal located in the world
     * frame. 
     */
    std::vector<base::Vector3d> getTrajectory();

    /**
     * Prints 
     */
    void printInformations();

 private:
    nav_graph_search::TraversabilityMap* mpTraversabilityMap;
    nav_graph_search::DStar* mpDStar;
    std::list<nav_graph_search::TerrainClass> mTerrainClasses;
    std::map<int, int> mTerrainClassInToOut;

    Eigen::Vector2i mStartPos, mGoalPos;

    std::vector<base::Vector3d> mTrajectory; // Contains the waypoints from goal to start.

    SimplePathPlanner() {}

    /**
     * Transforms the traversability map point to the world and adds the world point
     * to 'mTrajectory'.
     */
    void addVectorToTrajectory(size_t xi, size_t yi);
};

#endif
