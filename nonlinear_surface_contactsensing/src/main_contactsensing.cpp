#include <ros/ros.h>
#include <ros/node_handle.h>

#include "nonlinear_surface_contactsensing.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nonlinear_surface_contactsensing");
  ros::NodeHandle nh("~");

  std::string mesh_directory;
  if(!nh.getParam("/mesh_directory",mesh_directory))
    ROS_ERROR("Can't find mesh_directory!");

  std::vector<float> transformation;
  if(!nh.getParam("/mesh_transformation",transformation))
    ROS_ERROR("Can't find mesh_transformation!");

  Transformation T;
  T << transformation[0], transformation[1], transformation[2],  transformation[3],
      transformation[4], transformation[5], transformation[6],  transformation[7],
      transformation[8], transformation[9], transformation[10], transformation[11],
      0,                 0,                 0,                  1;

  NonlinearSurfaceContactsensing cs_thread(nh);
  cs_thread.set_ContactShape(mesh_directory, T);

  ROS_INFO("Contactsensing thread!!");
  cs_thread.run();

  return 0;

}
