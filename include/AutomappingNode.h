#ifndef _AUTOMAPPINGNODE_
#define _AUTOMAPPINGNODE_

#include <RobotClass.hpp>
#include <nav_msgs/GetPlan.h>
#include <box_finder/GetGoal.h>
#include <cstdlib>
#include <vector>
#include <fstream>

#define EQUAL(A, B) A.x == B.x && A.y == B.y
#define ISNULL(A) A.x == 0.0 && A.y == 0.0 && A.z == 0.0 && A.w == 0.0

class AutomappingNode{

    ros::NodeHandle nh, nh_for_params;
    RobotClass robot;
    ros::ServiceClient goal_requester, pathfinder;

    double safe_distance, tolerance, rotational_speed;
    std::string map_name, pose_params_file_name;

    geometry_msgs::PoseStamped safeGoalInPlan(const nav_msgs::Path &plan);
    double distanceBetweenPoses(const geometry_msgs::PoseStamped &pose1, const geometry_msgs::PoseStamped &pose2);

public:

    AutomappingNode();
    ~AutomappingNode();
    void doYourWork();
};

#endif //_AUTOMAPPINGNODE_
