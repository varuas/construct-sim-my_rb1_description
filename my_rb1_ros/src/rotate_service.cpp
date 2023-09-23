#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include <cmath>
#include <my_rb1_ros/Rotate.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

using namespace std;

ros::Subscriber odom_sub;
ros::Publisher vel_pub;

double current_yaw = 0;
double target_yaw = 0;

geometry_msgs::Twist vel_msg;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                   msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  current_yaw = yaw;
}

bool rotate_callback(my_rb1_ros::Rotate::Request &req,
                     my_rb1_ros::Rotate::Response &res) {
  ROS_INFO("The Service /rotate_robot has been called");

  target_yaw = current_yaw + (req.degrees * M_PI / 180);

  ros::Rate loop_rate(100);
  while (abs(target_yaw - current_yaw) > 0.01) {
    vel_msg.linear.x = 0;
    vel_msg.angular.z = target_yaw > current_yaw ? 0.2 : -0.2;
    vel_pub.publish(vel_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  vel_msg.linear.x = 0;
  vel_msg.angular.z = 0;
  vel_pub.publish(vel_msg);

  res.result = "Rotation successful";
  ROS_INFO("Finished service /rotate_robot");
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "rotate_service_node");
  ros::NodeHandle nh;

  odom_sub = nh.subscribe("odom", 1000, odomCallback);
  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::ServiceServer rotate_service =
      nh.advertiseService("/rotate_robot", rotate_callback);
  ROS_INFO("Service /rotate_robot Ready");
  ros::spin();

  return 0;
}