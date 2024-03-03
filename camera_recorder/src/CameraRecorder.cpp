// created by lijunqi on 2023/5/25
// Submodule of HeliosRobotSystem
// for more see document: https://swjtuhelios.feishu.cn/docx/MfCsdfRxkoYk3oxWaazcfUpTnih?from=from_copylink
/*
 * ██   ██ ███████ ██      ██  ██████  ███████
 * ██   ██ ██      ██      ██ ██    ██ ██
 * ███████ █████   ██      ██ ██    ██ ███████
 * ██   ██ ██      ██      ██ ██    ██      ██
 * ██   ██ ███████ ███████ ██  ██████  ███████
 */
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;
using namespace placeholders;

namespace helios_cv {

class CameraRecorder : public rclcpp::Node {
public:
CameraRecorder(const rclcpp::NodeOptions& options) : Node("recorder_node", options) {
    save_root_ = "/home/jk/Videos";
    tt = time(nullptr);
    localtime_r(&tt, &ttime);
    time_start_sec = ttime.tm_sec;
    time_start_min = ttime.tm_min;
    time_start_hour = ttime.tm_hour;
    strftime(now, 64, "%Y-%m-%d_%H_%M_S", &ttime);
    video_writer = VideoWriter(format("%s/%s.avi", save_root_.c_str(), now), VideoWriter::fourcc('M', 'J', 'P', 'G'), 5.0, cv::Size(1280, 1024));
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>("image_raw", 10, std::bind(&CameraRecorder::CallBack, this, _1));
  }

private:


  VideoWriter video_writer;
  int time_start_sec=0;
  int time_start_min = 0;
  int time_start_hour =0;
  char now[64];
  Mat img;
  std::time_t tt;
  struct tm ttime{};
  string save_root_;
  
  void CallBack(sensor_msgs::msg::Image::SharedPtr msg_ptr) {
    img = cv_bridge::toCvShare(msg_ptr, sensor_msgs::image_encodings::BGR8)->image.clone();
    cv::imshow("DDDDD", img);
    cv::waitKey(1);
    video_writer.write(img);
    tt = time(nullptr);
    localtime_r(&tt, &ttime);
    if((((ttime.tm_hour-time_start_hour)*60+ttime.tm_min-time_start_min)*60+(ttime.tm_sec - time_start_sec))>60){
      video_writer.release();
      strftime(now, 64, "%Y-%m-%d_%H_%M_S", &ttime);
      video_writer = VideoWriter(format("%s/%s.avi", save_root_.c_str(), now), VideoWriter::fourcc('D', 'I', 'V', 'X'), 50.0, cv::Size(1280, 1024));
      time_start_sec=ttime.tm_sec;
      time_start_min=ttime.tm_min;
      time_start_hour=ttime.tm_hour;
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
};

} // namespace helios_cv

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(helios_cv::CameraRecorder)