#include <MoveToTableNode.h>


MoveToTableNode::MoveToTableNode() : nh_for_param("~"){
    ROS_INFO_STREAM("Initialization...");
    nh_for_param.param<double>("distance_to_table", distance_to_table, 0.5);
    nh_for_param.param<double>("angle_with_table", angle_with_table, 0.0);
    nh_for_param.param<double>("table_width", table_width, 0.5);
    nh_for_param.param<double>("table_length", table_length, 0.5);
    nh_for_param.param<double>("max_size_error", max_size_error, 0.05);

    map_reader = nh.serviceClient<nav_msgs::GetMap>("static_map");
    pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("pose_near_table", 10, true);
    ROS_INFO_STREAM("Initialization done");
}



MoveToTableNode::~MoveToTableNode(){
    map_reader.shutdown();
    pose_publisher.shutdown();
}



geometry_msgs::PoseStamped MoveToTableNode::findTable(nav_msgs::OccupancyGrid& map){

    cv::Mat mat_edges;
    cv::Mat map_mat = cv::Mat(map.data).reshape(0, map.info.height);
    map_mat.convertTo(mat_edges, CV_8U);

    cv::Canny(mat_edges, mat_edges, 10, 30, 5);

    std::vector<std::vector<cv::Point> > contours;
    cv::findContours( mat_edges, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    geometry_msgs::PoseStamped goal;
    for(int i = 0; i < contours.size(); i++){

        cv::RotatedRect minRect;
        minRect = cv::minAreaRect( cv::Mat(contours[i]) );

        if(IS_SUIT_SIZE(table_length, table_width) || IS_SUIT_SIZE(table_width, table_length)){

            double table_angle = minRect.angle * M_PI / 180;
            if(minRect.size.width < minRect.size.height){
                table_angle -= M_PI/2;
            }

            goal.pose.position.x = minRect.center.x * map.info.resolution -
                                   distance_to_table * std::sin(table_angle) +
                                   map.info.origin.position.x;
            goal.pose.position.y = minRect.center.y * map.info.resolution +
                                   distance_to_table * std::cos(table_angle) +
                                   map.info.origin.position.y;
            goal.pose.orientation = tf::createQuaternionMsgFromYaw(table_angle - angle_with_table);
            goal.header.frame_id = "map";
            break;
        }

    }

    return goal;
}



void MoveToTableNode::doYourWork(){

    ROS_INFO_STREAM("Waiting for \"" << map_reader.getService() << "\" service appearance...");
    map_reader.waitForExistence();
    ROS_INFO_STREAM("\"" << map_reader.getService() << "\" service appeared");

    ROS_INFO_STREAM("Getting map...");
    nav_msgs::GetMap map_srv;
    map_reader.call(map_srv);
    ROS_INFO_STREAM("Map obtained");

    ROS_INFO_STREAM("Finding table...");
    geometry_msgs::PoseStamped pose_near_table = findTable(map_srv.response.map);
    if(ISNULL(pose_near_table.pose.orientation)){
        ROS_WARN_STREAM("No table found!");
        ROS_INFO_STREAM("Shutdowning...");
        return;
    }
    ROS_INFO_STREAM("Table found. Sending robot to it...");

    pose_publisher.publish(pose_near_table);

    std::string status = robot.moveTo(pose_near_table);
    if(status == "SUCCEEDED")
        ROS_INFO_STREAM("Robot successfully achieved the goal");
    else
        ROS_INFO_STREAM("Robot did not achieve the goal");

    ROS_INFO_STREAM("Shutdowning...");
}