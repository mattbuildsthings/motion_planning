/// \file
/// \brief Draws Each Obstacle in RViz using MarkerArrays
///
/// PARAMETERS:
/// PUBLISHES:
/// SUBSCRIBES:
/// FUNCTIONS:

#include <nav_msgs/OccupancyGrid.h>
#include "nuslam/TurtleMap.h"
#include <visualization_msgs/MarkerArray.h>
#include "GPlanUtilities.hpp"

// #include <math.h>
// #include <string>
// #include <vector>
// #include "map/map.hpp"
// #include "map/prm.hpp"
// #include "map/grid.hpp"
// #include <functional> // To use std::bind
//#include <visualization_msgs/Marker.h>
// #include <geometry_msgs/Point.h>
// // // Used to deal with YAML list of lists
//  #include <xmlrpcpp/XmlRpcValue.h> // catkin component

int main(int argc, char **argv)
/// The Main Function ///
{
  ROS_INFO("STARTING NODE: astar");

  ros::init(argc, argv, "astar_node"); // register the node on ROS
  ros::NodeHandle nh;                  // get a handle to ROS
  ros::NodeHandle nh_("~");            // get a handle to ROS

  //Get parameters associated with run, override defaults with ones from launch file
  GPlanUtilities::Params runParams(nh_);

  //Get Obstacles map, this will throw errors so it's good to have here.
  std::vector<map::Obstacle> obstacles_v = GPlanUtilities::GetObstacles(runParams);

  // Initialize grid_map outside
  nav_msgs::OccupancyGrid grid_map;

  // rviz representation of the grid
  std::vector<int8_t> map;

  // Init Marker Array
  visualization_msgs::MarkerArray map_arr;    //PRM, main while(mw)
  visualization_msgs::MarkerArray path_arr;   //PRM, Grid, mw
  visualization_msgs::MarkerArray path_debug; //PRM, Grid, mw

  if (runParams.map_type == "prm")
  // PRM VERSION
  {
    //This version takes a reference to runParams, does a check on k, possibly modifies it.
    GPlanUtilities::BuildPRM(runParams, obstacles_v, map_arr, path_arr, path_debug);
  }
  else
  // GRID Version
  {
    GPlanUtilities::BuildGrid(runParams, obstacles_v, map, grid_map, path_arr, path_debug);
  }

  // Init Marker Array Publisher
  ros::Publisher map_pub = nh.advertise<visualization_msgs::MarkerArray>("prm", 1);
  ros::Publisher path_pub = nh.advertise<visualization_msgs::MarkerArray>("path", 1);
  ros::Publisher debug_pub = nh.advertise<visualization_msgs::MarkerArray>("path_debug", 1);
  ros::Publisher grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("grid_map", 1);

  ros::Rate rate(runParams.frequency); //tick time = this + comp time
  // Main While
  while (ros::ok())
  {
    ros::spinOnce();

    // Publish PRM Map
    map_pub.publish(map_arr);

    // Publish GRID Map
    grid_map.header.stamp = ros::Time::now();
    grid_map.info.map_load_time = ros::Time::now();
    grid_map.data = map;
    grid_pub.publish(grid_map);

    // Publish Path
    path_pub.publish(path_arr);

    // Publish Path DEBUG
    debug_pub.publish(path_debug);
    rate.sleep();
  }
  return 0;
}