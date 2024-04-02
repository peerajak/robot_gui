#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include <numeric>
#include <ros/ros.h>
#include <string>

float triangle_distance(geometry_msgs::Point &a, geometry_msgs::Point &b) {
  return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
};

class PositioningList {
private:
  std::vector<geometry_msgs::Point> _stop_list;

public:
  PositioningList() = default;
  ~PositioningList() = default;

  void append(geometry_msgs::Point sPoint) { _stop_list.push_back(sPoint); };

  void reset() { _stop_list.clear(); }

  float get_travelled_distance() {
    float travelled_distance = 0.0;
    for (auto iter = std::next(_stop_list.begin()); iter < _stop_list.end();
         iter++) {
      travelled_distance += triangle_distance(*std::prev(iter), *iter);
    }

    return travelled_distance;
  };
};

PositioningList position_list;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {

  // robot_odom_x = msg->pose.pose.position.x;
  // robot_odom_y = msg->pose.pose.position.y;
  // robot_odom_z = msg->pose.pose.position.z;
  position_list.append(msg->pose.pose.position);
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_gui_subscriber");
  bool checked = false;
  bool checked2 = true;
  int count = 0;
  double trackbarValue = 0.0;
  char textBuffer[40];

  ros::NodeHandle nh;
  ros::Subscriber sub_o = nh.subscribe("/odom", 1000, odomCallback);
  ros::Rate loop_rate(2);
  while (ros::ok()) {
    ROS_INFO("Robot has traveled %f meters so far",
             position_list.get_travelled_distance());
    loop_rate.sleep();
    ros::spinOnce();
  }
}