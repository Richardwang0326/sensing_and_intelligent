#include "interbotix_sdk/arm_obj.h"
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <ros/package.h>
#include <cstdlib>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <cmath>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <interbotix_sdk/JointCommands.h>

using namespace cv;
using namespace std;

interbotix_sdk::JointCommands joint_pose;

int main( int argc, char** argv )
{
    ros::init(argc, argv, "arm_operation");
    ros::NodeHandle n;
    double home[8] = {0.023009711876511574, -0.6565437912940979, -0.45405831933021545, -1.2793400287628174, 0.00920388475060463, -0.003067961661145091, 0.019450684557548427, -0.019450684557548427},
	   pick[8] = {0.016873789951205254, 0.010737866163253784, -0.699495255947113, -0.7056311964988708, 0.015339808538556099, -0.004601942375302315, 0.019429268134630993, -0.019429268134630993},
	   goal[8] = {1.5186409950256348, -0.5737088322639465, -0.5445631742477417, -1.3897866010665894, -0.1395922601222992, -0.003067961661145091, 0.019450684557548427, -0.019450684557548427},
   	   place[8] = {1.6198837757110596, 0.1257864236831665, -0.5123496055603027, -0.8482913970947266, -0.12732040882110596, -0.003067961661145091, 0.019450684557548427, -0.019450684557548427},
	   drop[8] = {-0.05982525274157524, 0.9572040438652039, 0.05368933081626892, 0.8866409063339233, 0.012271846644580364, 0.24083499610424042, 0.023116707930464334, -0.023116707930464334};
    vector<double> home_array(home,home+8),pick_array(pick,pick+8),goal_array(goal,goal+8),place_array(place,place+8),drop_array(drop,drop+8);

    ros::Publisher joint_pose_pub = n.advertise<interbotix_sdk::JointCommands>("/wx200/joint/commands", 0);

    ros::Duration(1.5).sleep();
    joint_pose.cmd = home_array;
    ROS_INFO("OK");
    ros::Duration(2.5).sleep();
    joint_pose_pub.publish(joint_pose);

    joint_pose.cmd = drop_array;
    ROS_INFO("OK");
    ros::Duration(2.5).sleep();
    joint_pose_pub.publish(joint_pose);

    joint_pose.cmd = home_array;
    ROS_INFO("OK");
    ros::Duration(2.5).sleep();
    joint_pose_pub.publish(joint_pose);


/*
    joint_pose.cmd = pick_array;
    ROS_INFO("OK");
    ros::Duration(2.5).sleep();
    joint_pose_pub.publish(joint_pose);

    joint_pose.cmd = goal_array;
    ROS_INFO("OK");
    ros::Duration(2.5).sleep();
    joint_pose_pub.publish(joint_pose);

    joint_pose.cmd = place_array;
    ROS_INFO("OK");
    ros::Duration(2.5).sleep();
    joint_pose_pub.publish(joint_pose);
*/

    ros::spinOnce();
    return 0;
}

