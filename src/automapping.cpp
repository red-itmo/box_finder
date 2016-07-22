#include <AutomappingNode.h>

int main (int argc, char **argv){
    ros::init(argc, argv, "automapping_node");
    AutomappingNode node;
    node.doYourWork();
    return 0;
}
