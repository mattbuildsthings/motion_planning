#include "map/map.hpp"
#include <ros/ros.h> //for debug

namespace map
{
	using rigid2d::Vector2D;

	// Obstacle
	Obstacle::Obstacle()
	{
	}

	Obstacle::Obstacle(const std::vector<Vector2D> &vertices_)
	{
		vertices = vertices_;
	}

	// Map
	Map::Map()
	{
		find_map_extent();
		inflate_robot = 0.2;
	}

	Map::Map(const std::vector<Obstacle> &obstacles_)
	{
		obstacles = obstacles_;

		find_map_extent();

		inflate_robot = 0.2;
	}

	Map::Map(const std::vector<Obstacle> &obstacles_, const double inflate_robot_)
	{
		obstacles = obstacles_;

		find_map_extent();

		inflate_robot = inflate_robot_;
	}

	std::vector<Obstacle> Map::return_obstacles()
	{
		return obstacles;
	}

	void Map::find_map_extent()
	{
		double x_min = 0, x_max = 0, y_min = 0, y_max = 0;
		bool maxChange = false, minChange = false;
		map_max = Vector2D(x_max, y_max);
		map_min = Vector2D(x_min, y_min);

		//go through each obstacle
		for(const auto & obs : obstacles)
		{
			//go through each vertex, base map extent on vertices
			for (const auto & vert : obs.vertices)
			{
				if (vert.x > x_max)
				{
					x_max = vert.x;
					maxChange = true;
				}
				else if (vert.x < x_min)
				{
					x_min = vert.x;
					minChange = true;
				}

				if (vert.y > y_max)
				{
					y_max = vert.y ;
					maxChange = true;
				}
				else if (vert.y  < y_min)
				{
					y_min = vert.y;
					minChange = true;
				}

				if (maxChange)
				{
					map_max = Vector2D(x_max, y_max);
					maxChange = false;
				}
				if (minChange)
				{
					map_min = Vector2D(x_min, y_min);
					minChange = false;
				}
			}
		}
		ROS_INFO("Map Max x: %f, y: %f", x_max, y_max);
		ROS_INFO("Map Min x: %f, y: %f", x_min, y_min);
	}

	std::vector<Vector2D> Map::return_map_bounds()
	{
		std::vector<Vector2D> v;
		v.push_back(map_min);
		v.push_back(map_max);

		return v;
	}

	// Helper Functions
	double euclidean_distance(const double &x_rel, const double &y_rel)
	{
		return sqrt(pow(x_rel, 2) + pow(y_rel, 2));
	}

}