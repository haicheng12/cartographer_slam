#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <iostream>
#include <cmath>

using namespace std;

ros::Publisher pose_marker_pub;

double x, y, z, qx, qy, qz, qw;

geometry_msgs::PoseWithCovarianceStamped pose_now;

void pubMarker(double &current_x, double &current_y) // 发布小车实时位置
{
  visualization_msgs::Marker robot;
  robot.header.frame_id = "/map";
  robot.header.stamp = ros::Time::now();
  robot.ns = "robot";
  robot.id = 0;
  robot.type = visualization_msgs::Marker::SPHERE;
  robot.action = visualization_msgs::Marker::ADD;
  robot.pose.position.x = current_x;
  robot.pose.position.y = current_y;
  robot.pose.position.z = 0.0;
  robot.pose.orientation.x = 0.0;
  robot.pose.orientation.y = 0.0;
  robot.pose.orientation.z = 0.0;
  robot.pose.orientation.w = 1.0;

  robot.scale.x = 0.3;
  robot.scale.y = 0.3;
  robot.scale.z = 0.3;

  robot.color.a = 1.0; // Don't forget to set the alpha!
  robot.color.r = 1.0;
  robot.color.g = 0.0;
  robot.color.b = 0.0;

  pose_marker_pub.publish(robot);
  ros::spinOnce();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pub_local_pose");

  ros::NodeHandle node;
  ros::Publisher pose_pub = node.advertise<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10);
  pose_marker_pub = node.advertise<visualization_msgs::Marker>("/pose_marker", 1);

  tf::StampedTransform transform;
  tf::TransformListener listener;

  ros::Rate rate(10.0);

  while (ros::ok())
  {
    ros::Time start = ros::Time::now();
    // cout << "StartTime:" << start << endl;
    tf::StampedTransform transform;
    try
    {
      // 得到坐map和坐标base_link之间的关系
      listener.waitForTransform("map", "laser_mount_link", ros::Time(0), ros::Duration(3.0));
      listener.lookupTransform("map", "laser_mount_link", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    x = transform.getOrigin().x();
    y = transform.getOrigin().y();
    z = transform.getOrigin().z();

    tf::Quaternion q = transform.getRotation();
    qx = q.x();
    qy = q.y();
    qz = q.z();
    qw = q.w();

    pose_now.pose.pose.position.x = transform.getOrigin().x();
    pose_now.pose.pose.position.y = transform.getOrigin().y();
    pose_now.pose.pose.orientation.x = qx;
    pose_now.pose.pose.orientation.y = qy;
    pose_now.pose.pose.orientation.z = qz;
    pose_now.pose.pose.orientation.w = qw;
    pose_pub.publish(pose_now);

    double yaw = tf::getYaw(pose_now.pose.pose.orientation);
    // printf("x: %f, y: %f, z: %f, qx: %f,qy: %f,qz: %f, qw: %f, theta: %f\n", x, y, z, qx, qy, qz, qw, pos_now.theta);
    printf("x: %f, y: %f, yaw: %f \n", x, y, yaw);

    pubMarker(x, y);

    rate.sleep();

    ros::Time end = ros::Time::now();
    // cout << "EndTime:" << end << endl;
  }

  return 0;
};
