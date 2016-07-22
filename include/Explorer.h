#ifndef _EXPLORER_
#define _EXPLORER_

#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <box_finder/GetGoal.h>
#include <cmath>

#define ROUND(A) std::floor(A + 0.5)

class Explorer
{
    ros::NodeHandle n, n_for_params;
    ros::ServiceServer service;
    ros::ServiceClient client;

    nav_msgs::OccupancyGrid map;

    // variables that will contain parameters
    int region_size;
    double white_perc, initial_radius, black_perc, grey_perc, delta;

    bool callback(box_finder::GetGoal::Request &req,
                  box_finder::GetGoal::Response &res);

    double x_from_j(int j);
    double y_from_i(int i);
    bool processRegion(int i, int j);

public:
    Explorer();
    ~Explorer();
    void start_work();
};

#endif //_EXPLORER_
