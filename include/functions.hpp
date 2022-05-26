#ifndef functions_H
#define functions_H
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <sstream>
#include <iostream>
#include <string>
#include <chrono>
#include <string>
#include <memory>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "stdint.h"
#include "mtrand.hpp"

// rdm class, for gentaring random flot numbers
class rdm{
int i;
public:
rdm();
float randomize();
};


//Norm function prototype
float Norm( std::vector<float> , std::vector<float> );

//sign function prototype
float sign(float );

//Nearest function prototype
std::vector<float> Nearest(  std::vector< std::vector<float>  > , std::vector<float> );

//Steer function prototype
std::vector<float> Steer(  std::vector<float>, std::vector<float>, float );

//gridValue function prototype
int gridValue(nav_msgs::msg::OccupancyGrid &, std::vector<float>);

//ObstacleFree function prototype
char ObstacleFree(std::vector<float> , std::vector<float> & , nav_msgs::msg::OccupancyGrid);
#endif
