#ifndef IMUODOM_H_
#define IMUODOM_H_

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Vector3.h"

#include <iostream>

#define ACCELERATION_OF_GRAVITY 9.68 // 重力加速度，根据实际情况给，赤道附近为9.78m/s平方

namespace ImuOdomNS
{
  class ImuOdom
  {
  protected:
    // ROS messages (topics)
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber imu_sub_;
    ros::Publisher odom_pub_;

  public:
    ImuOdom();
    ~ImuOdom();

    void MainLoop();

  private:
    void imuCallback(const sensor_msgs::Imu &data); // 回调陀螺仪原始数据
    void calculateOdom();                           // 计算里程计
    inline double thetaLimit(double yaw);           // 角度限制

    // 线速度
    double imu_angular_velocity_x_;
    double imu_angular_velocity_y_;
    double imu_angular_velocity_z_;

    // 线加速度
    double imu_linear_acceleration_x_;
    double imu_linear_acceleration_y_;
    double imu_linear_acceleration_z_;

    // 速度
    double imu_vx_;
    double imu_vy_;
    double imu_vz_;

    // 里程计
    double imu_x_;
    double imu_y_;
    double imu_z_;

    // 时间间隔
    double dt_;

    double imu_data_yaw_;

    int count_up_;
    int count_down_;
  };
}

#endif
