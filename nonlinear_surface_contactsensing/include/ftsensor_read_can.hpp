#ifndef ftsensor_read_can_H
#define ftsensor_read_can_H
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <std_msgs/Float32MultiArray.h>

#include <net/if.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <fstream>
#include <string.h>


class ftsensor_read_can
{
private:
  ros::NodeHandle nh_;
  ros::Timer timer;
  ros::Publisher sensor_pub;

  int ms_;
  char* can_name;

  struct sockaddr_can addr;
  struct ifreq ifr;
  struct can_frame rxframe;
  struct can_frame txframe;
  time_t txcycle;
  int s;

  bool bias_flag{true};
  float FT[6];
  float FT_bias[6];

  time_t time_in_ms();

  // initialize CAN network setting
  void can_initialize();

public:
  ftsensor_read_can(char* name, ros::NodeHandle nh, int ms);

  // read the ftsensor values
  void read_ftsensor();

  // set the bias flag
  void set_bias();

  // publish the ftsensor values
  void get_ftdata(const ros::TimerEvent& event);

  void run();
};

#endif // ftsensor_read_can_H
