#include "joy_control/TeleopJoy.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_controll");
  TeleopJoy joy_control;
  std::cout << "Y/A: linear  +/-" << std::endl 
    << "B/X: angular +/-" << std::endl;

  ros::spin();
  return 0;
}