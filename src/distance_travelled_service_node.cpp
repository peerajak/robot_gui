#include "geometry_msgs/Point.h"
#include "nav_msgs/Odometry.h"
#include "std_srvs/Trigger.h"
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
    if (_stop_list.size() >= 2) {
      for (auto iter = std::next(_stop_list.begin()); iter < _stop_list.end();
           iter++) {
        travelled_distance += triangle_distance(*std::prev(iter), *iter);
      }
    }

    return travelled_distance;
  };

  float get_travelled_distance_lambda() {
    float travelled_distance = 0.0;

    auto triangle_distance_lambda =
        [&travelled_distance](geometry_msgs::Point &a,
                              geometry_msgs::Point &b) {
          travelled_distance +=
              std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
          return b;
        };
    if (_stop_list.size() >= 2)
      std::accumulate(std::next(_stop_list.begin()), _stop_list.end(),
                      *_stop_list.begin(), triangle_distance_lambda);

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

bool getDistance_callback(std_srvs::Trigger::Request &req,
                          std_srvs::Trigger::Response &res) {

  res.success = true;
  std::string resMessage(
      std::to_string(position_list.get_travelled_distance_lambda()));
  res.message = resMessage;
  ROS_INFO("sending back response:true");
  return res.success;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_gui_get_distance_service");
  bool checked = false;
  bool checked2 = true;
  int count = 0;
  double trackbarValue = 0.0;
  char textBuffer[40];

  ros::NodeHandle nh;
  ros::Subscriber sub_o = nh.subscribe("/odom", 1000, odomCallback);
  ros::ServiceServer getDistance_service =
      nh.advertiseService("/get_distance", getDistance_callback);
  ros::Rate loop_rate(2);
  while (ros::ok()) {
    ROS_DEBUG("Robot has traveled %f meters so far",
              position_list.get_travelled_distance());
    loop_rate.sleep();
    ros::spinOnce();
  }
}