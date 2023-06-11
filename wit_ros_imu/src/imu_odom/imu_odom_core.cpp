#include "imu_odom/imu_odom.h"

namespace ImuOdomNS
{
  ImuOdom::ImuOdom()
  {
    count_up_ = 0;
    count_down_ = 0;
    // odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);            // 发布里程计
    imu_sub_ = nh_.subscribe("/wit/imu", 10, &ImuOdom::imuCallback, this); // 回调陀螺仪原始数据
  }

  ImuOdom::~ImuOdom()
  {
  }

  inline double ImuOdom::thetaLimit(double yaw) // 角度限制
  {
    double theta = yaw;
    if (theta > M_PI)
    {
      theta = theta - 2 * M_PI;
    }
    else if (theta < -M_PI)
    {
      theta = theta + 2 * M_PI;
    }
    return theta;
  }

  void ImuOdom::imuCallback(const sensor_msgs::Imu &data) // 陀螺仪角度回调
  {
    imu_angular_velocity_x_ = data.angular_velocity.x; // 左右翻转的速度，往右边翻转为正，往左边翻转为负
    imu_angular_velocity_y_ = data.angular_velocity.y; // 前后翻转的速度，往前翻转为正，往后翻转为负
    imu_angular_velocity_z_ = data.angular_velocity.z; // 原地水平旋转的速度，左正右负
    // std::cout << "imu_angular_velocity_z_ " << imu_angular_velocity_z_ << std::endl;

    imu_linear_acceleration_x_ = data.linear_acceleration.x;
    imu_linear_acceleration_y_ = data.linear_acceleration.y;
    imu_linear_acceleration_z_ = data.linear_acceleration.z; // 重力加速度
    // std::cout << "imu_linear_acceleration_z_ " << imu_linear_acceleration_z_ << std::endl;
    double dis_acc_gravity = imu_linear_acceleration_z_ - ACCELERATION_OF_GRAVITY;
    // std::cout << "dis_acc_gravity " << dis_acc_gravity << std::endl;
    if (dis_acc_gravity >= 0.3)
    {
      count_up_++;
    }
    else if (dis_acc_gravity <= -0.3)
    {
      count_down_++;
    }

    if (count_up_ >= 5 && dt_ <= 2.0)
    {
      count_up_ = 0;
      std::cout << "向上移动" << std::endl;
    }
    if (count_down_ >= 5 && dt_ <= 2.0)
    {
      count_down_ = 0;
      std::cout << "向下移动" << std::endl;
    }

    // std::cout << " imu_angular_velocity_x_ " << imu_angular_velocity_x_;
    // std::cout << "  imu_angular_velocity_y_ " << imu_angular_velocity_y_;
    // std::cout << "  imu_angular_velocity_z_ " << imu_angular_velocity_z_;

    imu_data_yaw_ = thetaLimit(tf::getYaw(data.orientation)); // 角度，从四元数获取的
    std::cout << "角度  imu_data_yaw_ " << imu_data_yaw_ << std::endl;
  }

  void ImuOdom::calculateOdom() // 计算里程计
  {
    // imu_vx_ = imu_angular_velocity_x_;
    // imu_vy_ = imu_angular_velocity_y_;
    // imu_vz_ = imu_linear_acceleration_z_ - ACCELERATION_OF_GRAVITY;
    // imu_vz_ += imu_vz_;
    // std::cout << "imu_vz_ " << imu_vz_ << std::endl;

    // // double delta_x = imu_angular_velocity_x_ * dt_;
    // // double delta_y = imu_angular_velocity_y_ * dt_;
    // double delta_z = imu_vz_ * dt_;
    // std::cout << "delta_z " << delta_z << std::endl;

    // // 推算出来的imu的里程计
    // // imu_x_ += delta_x;
    // // imu_y_ += delta_y;
    // imu_z_ += delta_z;
    // std::cout << "imu_z_ " << imu_z_ << std::endl;

    // std::cout << "imu_x_ " << imu_x_;
    // std::cout << "  imu_y_ " << imu_y_;
    // std::cout << "  imu_z_ " << imu_z_ << std::endl;
  }

  void ImuOdom::MainLoop()
  {
    ros::Time current_time;
    ros::Time last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(10);

    // message declarations
    // geometry_msgs::TransformStamped odom_trans;
    // odom_trans.header.frame_id = "odom";
    // odom_trans.child_frame_id = "base_link";

    while (ros::ok())
    {
      current_time = ros::Time::now();
      dt_ = (current_time - last_time).toSec();

      calculateOdom();

      // geometry_msgs::Quaternion odom_quat;
      // odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, imu_data_yaw_);

      // // update transform
      // odom_trans.header.stamp = current_time;
      // odom_trans.transform.translation.x = imu_x_;
      // odom_trans.transform.translation.y = imu_y_;
      // odom_trans.transform.translation.z = imu_z_;
      // odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(imu_data_yaw_);

      // // filling the odometry
      // nav_msgs::Odometry odom;
      // odom.header.stamp = current_time;
      // odom.header.frame_id = "odom";
      // odom.child_frame_id = "base_link";

      // // position
      // odom.pose.pose.position.x = imu_x_;
      // odom.pose.pose.position.y = imu_y_;
      // odom.pose.pose.position.z = imu_z_;
      // odom.pose.pose.orientation = odom_quat;

      // // velocity
      // odom.twist.twist.linear.x = imu_vx_;
      // odom.twist.twist.linear.y = imu_vy_;
      // odom.twist.twist.linear.z = imu_vz_;
      // odom.twist.twist.angular.x = 0.0;
      // odom.twist.twist.angular.y = 0.0;
      // odom.twist.twist.angular.z = 0.0;

      last_time = current_time;

      // // publishing the odometry and the new tf
      // broadcaster.sendTransform(odom_trans);
      // odom_pub_.publish(odom);

      ros::spinOnce();
      loop_rate.sleep();
    }
  }
}
