#include <ros/ros.h>
#include <ros/node_handle.h>
#include <string.h>

#include "ftsensor_read_can.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ftread");
  ros::NodeHandle nh("~");

  // can port number and frequency should be inserted
  if (argc < 3) {ROS_ERROR("Input argument is not enough!");return 0;}

  double sampling_time = 1/std::stod(argv[2]);
  char can_name[4] = "can";  strcat(can_name,"0");

  ftsensor_read_can ft_thread(can_name, nh, sampling_time);

  ROS_INFO("F/T sensor read thread!!");
  ft_thread.run();

  return 0;

}
