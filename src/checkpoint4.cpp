#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define CVUI_IMPLEMENTATION
#include "odometry_messages_gui/cvui.h"
#include <deque>
#include <string>

#define WINDOW_NAME "Info"
double scaling = 1.0;

class MessageQ {
private:
  std::deque<std::string> _message_q;
  int _max_q_size;

public:
  MessageQ(int max_q_size) { this->_max_q_size = max_q_size; };
  ~MessageQ() = default;
  void insertMessageQ(std::string &str_in) {
    if (_message_q.size() < _max_q_size) {
      _message_q.push_back(str_in);
    } else {
      _message_q.pop_front();
      _message_q.push_back(str_in);
    }
  };

  void printMessageQ(cv::Mat &frame) {
    for (int i = 0; i < _message_q.size(); i++) {
      cvui::printf(frame, std::lround(scaling * 55),
                   std::lround(scaling * 165 + (12 * i)),
                   scaling * cvui::DEFAULT_FONT_SCALE, 0x00ff00,
                   _message_q[i].c_str());
    }
  };
};

int main(int argc, const char *argv[]) {
  bool checked = false;
  bool checked2 = true;
  int count = 0;
  double trackbarValue = 0.0;
  char textBuffer[40];
  MessageQ window_queue(7);
  // Init cvui and tell it to create a OpenCV window, i.e.
  // cv::namedWindow(WINDOW_NAME).
  cvui::init(WINDOW_NAME);

  double currentScaling = -1;
  cv::Mat frame;

  while (true) {
    if (scaling != currentScaling) {
      frame = cv::Mat(std::lround(scaling * 300), std::lround(scaling * 600),
                      CV_8UC3);
      currentScaling = scaling;
    }

    // Fill the frame with a nice color
    frame = cv::Scalar(49, 52, 49);

    // Show some pieces of text.
    cvui::text(frame, std::lround(scaling * 50), std::lround(scaling * 30),
               "Control Robot Speed", scaling * cvui::DEFAULT_FONT_SCALE,
               0xff0000);

    // You can also specify the size of the text and its color
    // using hex 0xRRGGBB CSS-like style.
    sprintf(textBuffer, "Trackbar  %.2f", trackbarValue);
    std::string STextBuffer(textBuffer);
    window_queue.insertMessageQ(STextBuffer);
    cvui::rect(frame, 10, 20, 230, 260, 0xaf55af);
    cvui::rect(frame, 250, 20, 345, 260, 0xaf55af);
    // cvui::text(frame, std::lround(scaling * 200), std::lround(scaling * 30),
    // textBuffer, scaling*cvui::DEFAULT_FONT_SCALE, 0xff0000);

    // Sometimes you want to show text that is not that simple, e.g. strings +
    // numbers. You can use cvui::printf for that. It accepts a variable number
    // of parameter, pretty much like printf does.

    // Buttons will return true if they were clicked, which makes
    // handling clicks a breeze.
    // if (cvui::button(frame, std::lround(scaling * 50), std::lround(scaling *
    // 60), "Colored Button", scaling*cvui::DEFAULT_FONT_SCALE, 0xa05050)) {
    //	std::cout << "Button clicked" << std::endl;
    //}

    // If you do not specify the button width/height, the size will be
    // automatically adjusted to properly house the label.

    // You can tell the width and height you want
    const int botton_size = 30;
    const int botton_teleopt_position_x = 80;
    const int botton_teleopt_position_y = 80;
    cvui::button(frame, std::lround(scaling * 80),
                 std::lround(scaling * botton_teleopt_position_y),
                 std::lround(scaling * botton_size * 2),
                 std::lround(scaling * botton_size), "Mid",
                 scaling * cvui::DEFAULT_FONT_SCALE);
    cvui::button(
        frame, std::lround(scaling * 80),
        std::lround(scaling * (botton_teleopt_position_y - botton_size)),
        std::lround(scaling * botton_size * 2),
        std::lround(scaling * botton_size), "Up",
        scaling * cvui::DEFAULT_FONT_SCALE);
    cvui::button(
        frame, std::lround(scaling * 80),
        std::lround(scaling * (botton_teleopt_position_y + botton_size)),
        std::lround(scaling * botton_size * 2),
        std::lround(scaling * botton_size), "Down",
        scaling * cvui::DEFAULT_FONT_SCALE);
    cvui::button(frame, std::lround(scaling * (80 - botton_size * 2)),
                 std::lround(scaling * botton_teleopt_position_y),
                 std::lround(scaling * botton_size * 2),
                 std::lround(scaling * botton_size), "Left",
                 scaling * cvui::DEFAULT_FONT_SCALE);
    cvui::button(frame, std::lround(scaling * (80 + botton_size * 2)),
                 std::lround(scaling * botton_teleopt_position_y),
                 std::lround(scaling * botton_size * 2),
                 std::lround(scaling * botton_size), "Right",
                 scaling * cvui::DEFAULT_FONT_SCALE);
    // Window components are useful td similars. At the
    // moment, there is no implementation to constraint content within a
    // a window.

    cvui::window(frame, std::lround(scaling * 50), std::lround(scaling * 150),
                 std::lround(scaling * 120), std::lround(scaling * 100), "Info",
                 scaling * cvui::DEFAULT_FONT_SCALE);

    window_queue.printMessageQ(frame);
    // cvui::printf(frame, std::lround(scaling * 55), std::lround(scaling *
    // 165), scaling*cvui::DEFAULT_FONT_SCALE, 0x00ff00,
    //  "%d + %.2f = %.2f", count, trackbarValue, count*trackbarValue);
    // generalInfoArea(WINDOW_NAME);
    //  cvui::imshow(WINDOW_NAME, frame);
    //  The counter component can be used to alter int variables. Use
    //  the 4th parameter of the function to point it to the variable
    //  to be changed.
    cvui::text(frame, std::lround(scaling * 300), std::lround(scaling * 40),
               "Linear Velocity", scaling * cvui::DEFAULT_FONT_SCALE, 0xff0000);
    cvui::text(frame, std::lround(scaling * 420), std::lround(scaling * 40),
               "0", scaling * cvui::DEFAULT_FONT_SCALE, 0x00ff00);
    cvui::text(frame, std::lround(scaling * 300), std::lround(scaling * 60),
               "Angular Velocity", scaling * cvui::DEFAULT_FONT_SCALE,
               0xff0000);
    cvui::text(frame, std::lround(scaling * 420), std::lround(scaling * 60),
               "0", scaling * cvui::DEFAULT_FONT_SCALE, 0x00ff00);
    // Counter can be used with doubles too. You can also specify
    // the counter's step (how much it should change
    // its value after each button press), as well as the format
    // used to print the value.

    // The trackbar component can be used to create scales.
    // It works with all numerical types (including chars).
    cvui::trackbar(frame, std::lround(scaling * 300), std::lround(scaling * 80),
                   std::lround(scaling * 150), &trackbarValue, 0., 50., 1,
                   "%.1Lf", 0, 1.0, scaling * cvui::DEFAULT_FONT_SCALE);

    // Display the lib version at the bottom of the screen
    // cvui::printf(frame, frame.cols - std::lround(scaling * 80), frame.rows -
    // std::lround(scaling * 20), scaling*cvui::DEFAULT_FONT_SCALE, 0xCECECE,
    // "cvui v.%s", cvui::VERSION);
    cvui::text(frame, std::lround(scaling * 300), std::lround(scaling * 140),
               "Estimate robot position based of odometry",
               scaling * cvui::DEFAULT_FONT_SCALE, 0xff0000);
    cvui::text(frame, std::lround(scaling * 300), std::lround(scaling * 150),
               "x", scaling * cvui::DEFAULT_FONT_SCALE * 1.5, 0xff0000);
    cvui::text(frame, std::lround(scaling * 300), std::lround(scaling * 170),
               "000.0000", scaling * cvui::DEFAULT_FONT_SCALE, 0x00ff00);
    cvui::text(frame, std::lround(scaling * 400), std::lround(scaling * 150),
               "y", scaling * cvui::DEFAULT_FONT_SCALE * 1.5, 0xff0000);
    cvui::text(frame, std::lround(scaling * 400), std::lround(scaling * 170),
               "000.0000", scaling * cvui::DEFAULT_FONT_SCALE, 0x00ff00);
    cvui::text(frame, std::lround(scaling * 500), std::lround(scaling * 150),
               "z", scaling * cvui::DEFAULT_FONT_SCALE * 1.5, 0xff0000);
    cvui::text(frame, std::lround(scaling * 500), std::lround(scaling * 170),
               "000.0000", scaling * cvui::DEFAULT_FONT_SCALE, 0x00ff00);

    cvui::text(frame, std::lround(scaling * 300), std::lround(scaling * 195),
               "Distance Travelled", scaling * cvui::DEFAULT_FONT_SCALE,
               0xff0000);
    if (cvui::button(frame, std::lround(scaling * 300),
                     std::lround(scaling * 210), "Call",
                     scaling * cvui::DEFAULT_FONT_SCALE)) {
      trackbarValue = 0;
      count = 0;
    }
    cvui::text(frame, std::lround(scaling * 370), std::lround(scaling * 220),
               "Distance in meters", scaling * cvui::DEFAULT_FONT_SCALE,
               0xff0000);
    cvui::text(frame, std::lround(scaling * 370), std::lround(scaling * 230),
               "000.00", scaling * cvui::DEFAULT_FONT_SCALE, 0x00ff00);

    cvui::update();

    // Show everything on the screen
    cv::imshow(WINDOW_NAME, frame);

    // Check if ESC key was pressed
    if (cv::waitKey(20) == 27) {
      break;
    }
  }

  return 0;
}
