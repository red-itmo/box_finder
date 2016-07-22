#include <RobotClass.hpp>

RobotClass::RobotClass() : mb_client("move_base", true){
    vel_publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    mb_client.waitForServer();
}



RobotClass::~RobotClass(){
    vel_publisher.shutdown();
}



std::string RobotClass::moveTo(geometry_msgs::PoseStamped goal){
    move_base_msgs::MoveBaseGoal mb_goal;
    mb_goal.target_pose = goal;
    std::string state = mb_client.sendGoalAndWait(mb_goal).toString();
    stop();
    return state;
}



void RobotClass::rotateInPlace(double angle, double abs_speed){

    geometry_msgs::Twist velocity;
    abs_speed = std::abs(abs_speed) * (angle > 0.0 ? 1 : -1);
    velocity.angular.z = abs_speed;

    TrueAngle robot_angle;
    geometry_msgs::PoseStamped robot_pose;
    ros::Rate sleeper(10);
    try{
        robot_pose = currentPose("odom");
        while(std::abs(robot_angle(robot_pose.pose.orientation)) < std::abs(angle)){
            vel_publisher.publish(velocity);
            sleeper.sleep();
            robot_pose = currentPose("odom");
        }
    }
    catch(tf::TransformException ex){
        throw ;
    }

    stop();
}



void RobotClass::stop(){
    geometry_msgs::Twist null_velocity;
    //this 3 from for{} is not used anywhere else
    for(int i = 0; i < 3; i++)
            vel_publisher.publish(null_velocity);
}



geometry_msgs::PoseStamped RobotClass::currentPose(std::string in_frame){
    tf::TransformListener listener;
    tf::StampedTransform transform;

    ros::Time now = ros::Time(0);
    listener.waitForTransform(in_frame, "base_link", now, ros::Duration(5.0));
    listener.lookupTransform(in_frame, "base_link", now, transform);

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = in_frame;
    pose.header.stamp = transform.stamp_;
    pose.pose.position.x = transform.getOrigin().x();
    pose.pose.position.y = transform.getOrigin().y();
    tf::quaternionTFToMsg(transform.getRotation(), pose.pose.orientation);
    return pose;
}
