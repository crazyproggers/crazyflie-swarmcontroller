#include <ros/ros.h>
#include "gesture_recognizer.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "neuron_controller_node");
    ros::NodeHandle node("~");

    GestureRecognizer gr(node);

    while (ros::ok())
        ros::Rate(1).sleep();

    return 0;
}