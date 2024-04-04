#include "ros/publisher.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define CVUI_IMPLEMENTATION
#include "nav_msgs/Odometry.h"
#include "odometry_messages_gui/cvui.h"
#include "robot_gui/RobotInfo_msg.h"
#include "std_srvs/Trigger.h"
#include <chrono> // std::chrono::milliseconds
#include <deque>
#include <future> // std::async, std::future
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <signal.h>
#include <string>
#include <tf/tf.h>
#include <tuple>

#define WINDOW_NAME "Info"
#define NUMLINE_ROBOTINFO_MSG 10
#define MAX_Q_SIZE 6

double scaling = 1.0;

class MessageQ {
private:
  std::deque<std::string *> _message_q;
  int _max_q_size;

public:
  MessageQ(int max_q_size) { this->_max_q_size = max_q_size; };
  ~MessageQ() = default;
  void insertMessageQ(std::string *str_in) {
    if (_message_q.size() < _max_q_size) {
      _message_q.push_back(str_in);
    } else {
      _message_q.pop_front();
      _message_q.push_back(str_in);
    }
  };

  void printMessageQ(cv::Mat &frame) {
    for (int i = 0; i < _message_q.size(); i++) {
      for (int j = 0; j < NUMLINE_ROBOTINFO_MSG; j++)
        cvui::printf(
            frame, std::lround(scaling * 55),
            std::lround(scaling *
                        (165 + (10 * NUMLINE_ROBOTINFO_MSG * i) + (10 * j))),
            scaling * cvui::DEFAULT_FONT_SCALE, 0x00ff00,
            _message_q[i][j].c_str());
    }
  };
};
MessageQ window_queue(MAX_Q_SIZE);
float robot_odom_x = 0;
float robot_odom_y = 0;
float robot_odom_z = 0;

void robot_infoCallback(const robot_gui::RobotInfo_msg::ConstPtr &msg) {
  // ROS_DEBUG("%s", msg->data_field_01.c_str());
  std::string *arraySTextBuffer = new std::string[NUMLINE_ROBOTINFO_MSG];

  arraySTextBuffer[0] = std::string(msg->data_field_01.c_str());
  arraySTextBuffer[1] = std::string(msg->data_field_02.c_str());
  arraySTextBuffer[2] = std::string(msg->data_field_03.c_str());
  arraySTextBuffer[3] = std::string(msg->data_field_04.c_str());
  arraySTextBuffer[4] = std::string(msg->data_field_05.c_str());
  arraySTextBuffer[5] = std::string(msg->data_field_06.c_str());
  arraySTextBuffer[6] = std::string(msg->data_field_07.c_str());
  arraySTextBuffer[7] = std::string(msg->data_field_08.c_str());
  arraySTextBuffer[8] = std::string(msg->data_field_09.c_str());
  arraySTextBuffer[9] = std::string(msg->data_field_10.c_str());
  window_queue.insertMessageQ(arraySTextBuffer);
};

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  // ROS_DEBUG("%f, %f, %f", msg->pose.pose.orientation.x,
  // msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  robot_odom_x = msg->pose.pose.position.x;
  robot_odom_y = msg->pose.pose.position.y;
  robot_odom_z = msg->pose.pose.position.z;
};

typedef std::tuple<bool, std::string> TriggerRespTuple;

TriggerRespTuple get_distance_service_client(ros::ServiceClient *serviceClient,
                                             std_srvs::Trigger *srv) {
  serviceClient->call(*srv);
  bool success = srv->response.success;
  ros::Duration(2.5).sleep();
  if (success) {
    ROS_DEBUG("Success: Distance= %s meters.",
              srv->response.message
                  .c_str()); // Print the result given by the service called
  } else {
    ROS_ERROR("Failed to call service /trajectory_by_name");
  }
  return {success, srv->response.message}; //{success, srv.response.message};
}

void mySigintHandler(int sig) {
  ROS_DEBUG("/rotate_robot service stopped");
  ros::shutdown();
}
std::future<TriggerRespTuple> fut;
std::future_status fut_status;
std::chrono::milliseconds span(100);
bool fut_got_initialized = false;
geometry_msgs::Twist twst_msg;
const float botton_teleopt_step = 0.5;
float linear_velocity_up = 0;
float angular_velocity_left = 0;
void setTwistMessage(float lx, float ly, float lz, float ax, float ay,
                     float az) {
  twst_msg.linear.x = lx;
  twst_msg.linear.y = ly;
  twst_msg.linear.z = lz;
  twst_msg.angular.x = ax;
  twst_msg.angular.y = ay;
  twst_msg.angular.z = az;
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_gui_subscriber");
  ros::NodeHandle nh;
  ros::Subscriber sub_r = nh.subscribe<robot_gui::RobotInfo_msg>(
      "robot_info", 1000, robot_infoCallback);
  ros::Subscriber sub_o = nh.subscribe("/odom", 1000, odomCallback);
  ros::service::waitForService(
      "/get_distance"); // wait for service to be running
  ros::ServiceClient getDistance_service =
      nh.serviceClient<std_srvs::Trigger>("/get_distance");
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  std_srvs::Trigger srv;
  // Init cvui
  cvui::init(WINDOW_NAME);
  double currentScaling = -1;
  cv::Mat frame;
  signal(SIGINT, mySigintHandler);
  bool get_distance_success = false;
  std::string get_distance_message;
  setTwistMessage(0, 0, 0, 0, 0, 0);
  while (ros::ok()) {

    if (scaling != currentScaling) {
      frame = cv::Mat(std::lround(scaling * 900), std::lround(scaling * 1000),
                      CV_8UC3);
      currentScaling = scaling;
    }

    frame = cv::Scalar(49, 52, 49);
    cvui::text(frame, std::lround(scaling * 50), std::lround(scaling * 30),
               "Control Robot Speed", scaling * cvui::DEFAULT_FONT_SCALE,
               0xff0000);

    // Divide frame into two rectangles
    cvui::rect(frame, 10, 20, 530, 990, 0xaf55af);
    cvui::rect(frame, 550, 20, 440, 990, 0xaf55af);

    /************************************************
     *                Teleopt Button
     *************************************************/
    const int botton_size = 30;
    const int botton_teleopt_position_x = 80;
    const int botton_teleopt_position_y = 80;

    if (cvui::button(frame, std::lround(scaling * 80),
                     std::lround(scaling * botton_teleopt_position_y),
                     std::lround(scaling * botton_size * 2),
                     std::lround(scaling * botton_size), "Mid",
                     scaling * cvui::DEFAULT_FONT_SCALE)) {
      linear_velocity_up = 0;
      angular_velocity_left = 0;
    }
    if (cvui::button(
            frame, std::lround(scaling * 80),
            std::lround(scaling * (botton_teleopt_position_y - botton_size)),
            std::lround(scaling * botton_size * 2),
            std::lround(scaling * botton_size), "Up",
            scaling * cvui::DEFAULT_FONT_SCALE)) {
      linear_velocity_up = botton_teleopt_step;
      angular_velocity_left = 0;
    }

    if (cvui::button(
            frame, std::lround(scaling * 80),
            std::lround(scaling * (botton_teleopt_position_y + botton_size)),
            std::lround(scaling * botton_size * 2),
            std::lround(scaling * botton_size), "Down",
            scaling * cvui::DEFAULT_FONT_SCALE)) {
      linear_velocity_up = -botton_teleopt_step;
      angular_velocity_left = 0;
    }

    if (cvui::button(frame, std::lround(scaling * (80 - botton_size * 2)),
                     std::lround(scaling * botton_teleopt_position_y),
                     std::lround(scaling * botton_size * 2),
                     std::lround(scaling * botton_size), "Left",
                     scaling * cvui::DEFAULT_FONT_SCALE)) {
      linear_velocity_up = 0;
      angular_velocity_left = botton_teleopt_step;
    }
    if (cvui::button(frame, std::lround(scaling * (80 + botton_size * 2)),
                     std::lround(scaling * botton_teleopt_position_y),
                     std::lround(scaling * botton_size * 2),
                     std::lround(scaling * botton_size), "Right",
                     scaling * cvui::DEFAULT_FONT_SCALE)) {
      linear_velocity_up = 0;
      angular_velocity_left = -botton_teleopt_step;
    }

    setTwistMessage(linear_velocity_up, 0, 0, 0, 0, angular_velocity_left);
    pub.publish(twst_msg);

    /*************************************************
     *          General Area
     *************************************************/
    cvui::window(frame, std::lround(scaling * 20), std::lround(scaling * 150),
                 std::lround(scaling * 500), std::lround(scaling * 400), "Info",
                 scaling * cvui::DEFAULT_FONT_SCALE);

    window_queue.printMessageQ(frame);

    /*************************************************
     *          Current Velocity
     *************************************************/

    cvui::text(frame, std::lround(scaling * 600), std::lround(scaling * 40),
               "Linear Velocity", scaling * cvui::DEFAULT_FONT_SCALE, 0xff0000);

    cvui::text(frame, std::lround(scaling * 720), std::lround(scaling * 40),
               std::to_string(linear_velocity_up).c_str(),
               scaling * cvui::DEFAULT_FONT_SCALE, 0x00ff00);
    cvui::text(frame, std::lround(scaling * 600), std::lround(scaling * 60),
               "Angular Velocity", scaling * cvui::DEFAULT_FONT_SCALE,
               0xff0000);
    cvui::text(frame, std::lround(scaling * 720), std::lround(scaling * 60),
               std::to_string(angular_velocity_left).c_str(),
               scaling * cvui::DEFAULT_FONT_SCALE, 0x00ff00);

    /*************************************************
     *          Robot Position (odometry based)
     *************************************************/
    cvui::text(frame, std::lround(scaling * 600), std::lround(scaling * 140),
               "Estimate robot position based of odometry",
               scaling * cvui::DEFAULT_FONT_SCALE, 0xff0000);
    cvui::text(frame, std::lround(scaling * 600), std::lround(scaling * 150),
               "x", scaling * cvui::DEFAULT_FONT_SCALE * 1.5, 0xff0000);
    char robot_odom_x_buffer[11];
    char robot_odom_y_buffer[11];
    char robot_odom_z_buffer[11];
    sprintf(robot_odom_x_buffer, "%3.6f", robot_odom_x);
    sprintf(robot_odom_y_buffer, "%3.6f", robot_odom_y);
    sprintf(robot_odom_z_buffer, "%3.6f", robot_odom_z);
    cvui::text(frame, std::lround(scaling * 600), std::lround(scaling * 170),
               robot_odom_x_buffer, scaling * cvui::DEFAULT_FONT_SCALE,
               0x00ff00);
    cvui::text(frame, std::lround(scaling * 700), std::lround(scaling * 150),
               "y", scaling * cvui::DEFAULT_FONT_SCALE * 1.5, 0xff0000);
    cvui::text(frame, std::lround(scaling * 700), std::lround(scaling * 170),
               robot_odom_y_buffer, scaling * cvui::DEFAULT_FONT_SCALE,
               0x00ff00);
    cvui::text(frame, std::lround(scaling * 800), std::lround(scaling * 150),
               "z", scaling * cvui::DEFAULT_FONT_SCALE * 1.5, 0xff0000);
    cvui::text(frame, std::lround(scaling * 800), std::lround(scaling * 170),
               robot_odom_z_buffer, scaling * cvui::DEFAULT_FONT_SCALE,
               0x00ff00);

    /*************************************************
     *          Distance Travel Service
     *************************************************/

    cvui::text(frame, std::lround(scaling * 600), std::lround(scaling * 195),
               "Distance Travelled", scaling * cvui::DEFAULT_FONT_SCALE,
               0xff0000);

    if (cvui::button(frame, std::lround(scaling * 600),
                     std::lround(scaling * 210), "Call",
                     scaling * cvui::DEFAULT_FONT_SCALE)) {

      fut = std::async(std::launch::async, get_distance_service_client,
                       &getDistance_service, &srv);
      fut_got_initialized = true;
    }
    cvui::text(frame, std::lround(scaling * 670), std::lround(scaling * 220),
               "Distance in meters", scaling * cvui::DEFAULT_FONT_SCALE,
               0xff0000);
    if (fut_got_initialized) {
      fut_status = fut.wait_for(span);
      if (fut_status == std::future_status::ready) {
        TriggerRespTuple result_getDistance = fut.get();
        get_distance_success = std::get<0>(result_getDistance);
        get_distance_message = std::get<1>(result_getDistance);
        ROS_DEBUG("Future returns %d, %s", get_distance_success,
                  get_distance_message.c_str());
        fut_got_initialized = false;
        // ROS_DEBUG("Future returns");
      }
    }

    if (!get_distance_success)
      get_distance_message = "-------";
    cvui::text(frame, std::lround(scaling * 670), std::lround(scaling * 230),
               get_distance_message.c_str(), scaling * cvui::DEFAULT_FONT_SCALE,
               0x00ff00);
    /***************End UI design******/
    cvui::update();

    // Show everything on the screen
    cv::imshow(WINDOW_NAME, frame);
    ros::spinOnce();
    // Check if ESC key was pressed
    if (cv::waitKey(20) == 27) {
      break;
    }
  }

  return 0;
}
