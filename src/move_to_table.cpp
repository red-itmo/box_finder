#include <MoveToTableNode.h>

int main (int argc, char **argv){
    ros::init(argc, argv, "move_to_table_node");
    MoveToTableNode node;
    node.doYourWork();
    return 0;
}
