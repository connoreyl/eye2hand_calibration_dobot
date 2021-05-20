#include "ros/ros.h"
#include "ar_convert.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtle_follow");

  ros::NodeHandle nh;

  std::shared_ptr<ARConvert> arConvert(new ARConvert(nh));
  std::thread t(&ARConvert::publishTag, arConvert);
  
  ros::spin();

  ros::shutdown();
  t.join();

  return 0;
}
