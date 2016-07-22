#ifndef _MOVETOTABLENODE_
#define _MOVETOTABLENODE_

#include <ros/ros.h>
#include <RobotClass.hpp>
#include <nav_msgs/GetMap.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cmath>

#define ISNULL(A) A.x == 0.0 && A.y == 0.0 && A.z == 0.0 && A.w == 0.0
#define IS_SUIT_SIZE(A,B) ( (std::abs(minRect.size.width*map.info.resolution  - A) < max_size_error) && \
                            (std::abs(minRect.size.height*map.info.resolution - B) < max_size_error) )

class MoveToTableNode{
    ros::NodeHandle nh, nh_for_param;
    RobotClass robot;
    ros::ServiceClient map_reader;
    ros::Publisher pose_publisher;

    double table_width, table_length, max_size_error;
    double distance_to_table, angle_with_table;

    geometry_msgs::PoseStamped findTable(nav_msgs::OccupancyGrid &map);

public:

    MoveToTableNode();
    ~MoveToTableNode();
    void doYourWork();

};

#endif //_MOVETOTABLENODE_