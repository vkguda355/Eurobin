#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grid_map_publisher");
  ros::NodeHandle n;
  ros::Publisher grid_pub = n.advertise<nav_msgs::OccupancyGrid>("grid_map", 1);

  nav_msgs::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.resolution = 0.1;
  grid.info.width = 10;
  grid.info.height = 10;
  grid.info.origin.position.x = -1.0;
  grid.info.origin.position.y = -1.0;
  grid.info.origin.orientation.w = 1.0;

  grid.data.resize(grid.info.width * grid.info.height);
  std::fill(grid.data.begin(), grid.data.end(), 0);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    grid.header.stamp = ros::Time::now();
    grid_pub.publish(grid);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
