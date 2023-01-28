#include "ftsensor_read_can.hpp"

ftsensor_read_can::ftsensor_read_can(char* name, ros::NodeHandle nh, int ms):nh_(nh){
  can_name = name;  ms_ = ms;
  timer = nh_.createTimer(ros::Duration(ms_), &ftsensor_read_can::get_ftdata, this);
  sensor_pub = nh_.advertise<std_msgs::Float32MultiArray>("/ftsensor/measured_value",10);

  can_initialize();
}

void ftsensor_read_can::can_initialize(){
  if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) { perror("Socket");}
  strcpy(ifr.ifr_name, can_name);
  ioctl(s, SIOCGIFINDEX, &ifr);

  memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) { perror("Bind");}

  txframe.can_id = 0x64;
  txframe.can_dlc = 8;
  memcpy(txframe.data, "\011\000\000\000\000\000\000\000", 8);
  write(s, &txframe, sizeof(struct can_frame)) != sizeof(struct can_frame);
  txcycle = time_in_ms();
}

time_t ftsensor_read_can::time_in_ms(){
  struct timeval tv = {0};

  gettimeofday(&tv, NULL);
  return (tv.tv_sec * 1000UL + tv.tv_usec / 1000UL);
}

// reading the ftsensor values
void ftsensor_read_can::read_ftsensor(){
  short pData[6];
  int nbytes;
  unsigned char recvCandata[2][8];

  nbytes = recv(s, &rxframe, sizeof(struct can_frame), MSG_DONTWAIT);

  if (nbytes > 0) {
    for(int i = 0; i < rxframe.can_dlc; i++) recvCandata[rxframe.can_id-1][i] = (unsigned char)rxframe.data[i];
  }

  if(txcycle < time_in_ms()) {
    txcycle = time_in_ms() + 1;

    if (write(s, &txframe, sizeof(struct can_frame)) != sizeof(struct can_frame)) perror("Write");

    txframe.data[0] = 0x0B;
  }

  pData[0] = (unsigned short)(((unsigned short)recvCandata[0][1])*256 + ((unsigned short)recvCandata[0][2]));
  pData[1] = (unsigned short)(((unsigned short)recvCandata[0][3])*256 + ((unsigned short)recvCandata[0][4]));
  pData[2] = (unsigned short)(((unsigned short)recvCandata[0][5])*256 + ((unsigned short)recvCandata[0][6]));
  pData[3] = (unsigned short)(((unsigned short)recvCandata[0][7])*256 + ((unsigned short)recvCandata[1][0]));
  pData[4] = (unsigned short)(((unsigned short)recvCandata[1][1])*256 + ((unsigned short)recvCandata[1][2]));
  pData[5] = (unsigned short)(((unsigned short)recvCandata[1][3])*256 + ((unsigned short)recvCandata[1][4]));


  if(bias_flag&&(rxframe.can_id == 2)){
    for(int i = 0; i < 6; i++) FT_bias[i] = (float)pData[i];

    bias_flag = false;
  }

  for(int i = 0; i < 3; i++) FT[i] = ((float)pData[i] - FT_bias[i])/50;
  for(int i = 3; i < 6; i++) FT[i] = ((float)pData[i] - FT_bias[i])/2000;
}

void ftsensor_read_can::set_bias(){
  bias_flag = true;
}

void ftsensor_read_can::get_ftdata(const ros::TimerEvent& event){
  std_msgs::Float32MultiArray pub_data;
  for(int i = 0; i < 6; i++) pub_data.data.push_back(FT[i]);

  sensor_pub.publish(pub_data);
}

void ftsensor_read_can::run(){
  while(ros::ok()){
    read_ftsensor();
    ros::spinOnce();
  }
}


