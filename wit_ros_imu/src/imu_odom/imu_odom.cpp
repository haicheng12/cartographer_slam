#include "imu_odom/imu_odom.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_odom");

  ImuOdomNS::ImuOdom imu_odom;
  imu_odom.MainLoop();

  return 0;
}
