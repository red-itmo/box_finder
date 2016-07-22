#include <TrueAngle.h>


double TrueAngle::operator()(geometry_msgs::Quaternion quaternion){

    double curr_quat_angle = tf::getYaw(quaternion);

    if(is_first_time){
        is_first_time = false;
        true_angle = 0;
    }
    else
        if( (curr_quat_angle - last_quat_angle < M_PI) && (curr_quat_angle - last_quat_angle > -M_PI) )
            true_angle += curr_quat_angle - last_quat_angle;
        else if(curr_quat_angle - last_quat_angle > M_PI)
            true_angle += (curr_quat_angle - last_quat_angle) - 2 * M_PI;
        else
            true_angle += (curr_quat_angle - last_quat_angle) + 2 * M_PI;

    last_quat_angle = curr_quat_angle;
    return true_angle;
}
