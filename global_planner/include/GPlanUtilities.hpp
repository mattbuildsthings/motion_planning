#include <vector>
#include "map/map.hpp"
#include <ros/ros.h>
#include "global_planner/heuristic.hpp"

namespace GPlanUtilities
{
        //Struct for run
    struct Params
    {
        // Vars
        double frequency = 10.0;
        // Scale by which to transform marker poses (10 cm/cell so we use 10)
        double SCALE = 10.0;
        std::string frame_id = "base_footprint";
        std::string planner_type = "astar";
        std::string map_type = "prm";
        XmlRpc::XmlRpcValue xml_obstacles;
        std::vector<double> start_vec{7.0, 3.0};
        std::vector<double> goal_vec{7.0, 26.0};
        // resolution by which to transform marker poses (10 cm/cell so we use 10)
        double resolution = 0.01;

        // PRM Parameters
        int n = 10;
        int k = 5;
        double thresh = 0.01;
        double inflate = 0.1;

        rigid2d::Vector2D start;
        rigid2d::Vector2D goal;

        Params()
        {
            start = rigid2d::Vector2D(start_vec.at(0) / SCALE, start_vec.at(1) / SCALE);
            goal = rigid2d::Vector2D(goal_vec.at(0) / SCALE, goal_vec.at(1) / SCALE);
        }
        
        Params(ros::NodeHandle nh_)
        {
            // Parameters
            nh_.getParam("frequency", frequency);
            //<rosparam command="load" file="$(find map)/config/map.yaml"/> <-- this command loads the file (in a .launch file)
            //obstacles: <-- this is a field in that yaml file and gets loaded as a paramter much like the explicit 'n' or 'k' params
            nh_.getParam("obstacles", xml_obstacles);
            nh_.getParam("start", start_vec);
            nh_.getParam("goal", goal_vec);
            nh_.getParam("map_frame_id", frame_id);
            nh_.getParam("n", n);
            nh_.getParam("k", k);
            nh_.getParam("thresh", thresh);
            nh_.getParam("inflate", inflate);
            nh_.getParam("scale", SCALE);
            nh_.getParam("planner", planner_type);
            nh_.getParam("map_type", map_type);
            nh_.getParam("resolution", resolution);
            start = rigid2d::Vector2D(start_vec.at(0) / SCALE, start_vec.at(1) / SCALE);
            goal = rigid2d::Vector2D(goal_vec.at(0) / SCALE, goal_vec.at(1) / SCALE);
        }
    };

    //This just processes the parameter from the launch file.
    std::vector<map::Obstacle> GetObstacles(const Params &runParams);

    void InitMarkers(visualization_msgs::Marker &marker, visualization_msgs::Marker &sph_mkr,
                     visualization_msgs::Marker &path_marker, visualization_msgs::Marker &path_sph_mkr, const GPlanUtilities::Params &runParams);

    void BuildPRM(GPlanUtilities::Params &runParams, const std::vector<map::Obstacle> &obstacles_v,
                  visualization_msgs::MarkerArray &map_arr,  //PRM, main while(mw)
                  visualization_msgs::MarkerArray &path_arr, //PRM, Grid, mw
                  visualization_msgs::MarkerArray &path_debug);

    void BuildGrid(const GPlanUtilities::Params &runParams, const std::vector<map::Obstacle> &obstacles_v, std::vector<int8_t> &map,
                   nav_msgs::OccupancyGrid &grid_map,
                   visualization_msgs::MarkerArray &path_arr, //PRM, Grid, mw
                   visualization_msgs::MarkerArray &path_debug);
}