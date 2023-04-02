#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include <cmath>

ros::NodeHandle nh;
ros::Subscriber sub = nh.subscribe("map", 1, mapCallback);

ros::Publisher grid_pub;
nav_msgs::OccupancyGrid::ConstPtr map_msg;
double map_resolution;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  map_msg = msg;
  map_resolution = msg->info.resolution;
}

//Get the pose
tf::TransformListener tf_listener;
tf::StampedTransform user_transform;

try
{
  tf_listener.lookupTransform("map", "base_link", ros::Time(0), user_transform);
}
catch (tf::TransformException ex)
{
  ROS_ERROR("%s", ex.what());
  return;
}
  
//field of view
const double fov_radius = 5.0; // in meters
const double fov_resolution = 0.1; // in meters
const int fov_size = 2 * fov_radius / fov_resolution;

nav_msgs::OccupancyGrid fov;
fov.info.width = fov_size;
fov.info.height = fov_size;
fov.info.resolution = fov_resolution;
fov.info.origin.position.x = user_transform.getOrigin().x() - fov_radius;
fov.info.origin.position.y = user_transform.getOrigin().y() - fov_radius;

fov.data.resize(fov_size * fov_size);

  // Identify visible points in the occupancy grid
  for (int i = 0; i < fov_size; i++)
{
  for (int j = 0; j < fov_size; j++)
  {
    double x = fov.info.origin.position.x + i * fov_resolution;
    double y = fov.info.origin.position.y + j * fov_resolution;

    if (distance(user_transform, x, y) <= fov_radius)
    {
      if (is_visible(user_transform, x, y))
      {
        fov.data[i * fov_size + j] = 100; // visible
      }
      else
      {
        fov.data[i * fov_size + j] = 0; // not visible
      }
    }
    else
    {
      fov.data[i * fov_size + j] = -1; // out of range
    }
  }
}

ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("fov", 1);
pub.publish(fov);
