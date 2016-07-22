#include <Explorer.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_goal_at_map_server");
    Explorer exp;
    exp.start_work();
    return 0;
}
