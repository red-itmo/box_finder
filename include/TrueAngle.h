#ifndef _TRUEANGLE_
#define _TRUEANGLE_

#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <cmath>

class TrueAngle{
    bool is_first_time;
    double last_quat_angle, true_angle;
public:
    TrueAngle() {is_first_time = true;}
    double operator()(geometry_msgs::Quaternion quaternion);
};

#endif //_TRUEANGLE_