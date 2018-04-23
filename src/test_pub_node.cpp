
#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_pub_node");
    ros::NodeHandle node;
    ros::Publisher test_pub = node.advertise<geometry_msgs::PoseStamped>("/mocap_data_rigid1",10);
    ros::Rate _rate(100);
    int i = 0;
    while(ros::ok()) {
        geometry_msgs::PoseStamped msg;
        i++;
        msg.pose.position.x = i;
        msg.header.stamp = ros::Time::now();
        test_pub.publish(msg);
        ros::spinOnce();
        _rate.sleep();
    }
    return 0;
}