#include "follow_human/follow_human.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "follow_human");
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
  {
    ros::console::notifyLoggerLevelsChanged();
  }
  follow_human human_follower;

  while (ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}
