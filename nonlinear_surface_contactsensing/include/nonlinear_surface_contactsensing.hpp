// Contact sensing code from DownJ 2020.11.17
// using characteristic of triangular mesh
#ifndef CONTACT_SENSING_HPP
#define CONTACT_SENSING_HPP

#include <ros/ros.h>
#include <ros/node_handle.h>

#include "stl_loader.hpp"
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PointStamped.h>
#include <vector>

// include eigen
#ifndef EIGEN
#define EIGEN
#include "Eigen/Dense"
using namespace Eigen;
#endif

struct Wrench {float at[3];};
struct WrenchAxis{Wrench r0; Wrench f;};

inline void insert_wrench(Wrench &w, float x, float y, float z) {
    w.at[0] = x;    w.at[1] = y;    w.at[2] = z;
}

class NonlinearSurfaceContactsensing
{
private:
  ros::NodeHandle nh_;
  ros::Timer timer_;
  ros::Publisher cp_pub_;
  ros::Subscriber ftsensor_sub_;

  // loaded STL file class.
  StlLoader surface_shape_;

  bool ft_measure_flag_{false};
  std::vector<float> ft_value_;
  std::vector<float> cp_;

  // get wrench axis by force sensor
  void get_WrenchAxis(std::vector<float> ft, WrenchAxis& out);

  // get closest surface triangle when first calculation.
  int get_ClosestSurface(WrenchAxis r);

  // get the distance between wrench axis and point.
  float get_DistanceLinePoint(WrenchAxis r, float x, float y, float z);

  // get the next triangle.
  // next triangle goes to the direction of previous contact point.
  int get_NextTriangle(int idx, std::vector<int>& vert_idx);

  // determine that surface contact wrench axis.
  bool is_SurfaceContact(int idx, WrenchAxis r, std::vector<int> &vert_idx, std::vector<float> cp);

  void ftsensor_cb(const std_msgs::Float32MultiArray &msg);

public:
  NonlinearSurfaceContactsensing(ros::NodeHandle nh);
  void run();

  // set the STL file and transformation matrix between STL file and sensor frame.
  bool set_ContactShape(const string& filename, Transformation pos);

  // calculate the contact point in sensor frame.
  void get_ContactPoint(std::vector<float> ft, std::vector<float> cp);
};

#endif // CONTACT_SENSING_HPP
