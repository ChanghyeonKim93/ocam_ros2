#include <chrono>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "ocam_camera.h"

class OcamRos2Node : public rclcpp::Node {
 public:
  explicit OcamRos2Node(const std::string& node_name) : Node(node_name) {
    std::cerr << "node starts\n";

    frame_rate_ = 10;
    ocam_camera_ = std::make_unique<ocam::Camera>(resolution_, frame_rate_);

    ocam::CameraConfig config;
    config.exposure = 25;  // milliseconds
    config.gain = 40;
    config.wb_blue = 50;
    config.wb_red = 50;
    config.auto_exposure = false;
    CallbackControlUvc(config);

    PreparePublishers();
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(100),
                                std::bind(&OcamRos2Node::PollDevice, this));
  }

  ~OcamRos2Node() {}

  void PreparePublishers() {}

  void CallbackControlUvc(ocam::CameraConfig& config) {
    ocam_camera_->ControlUvc(config);
  }

  void PollDevice() {
    ocam::Image camera_image;
    while (true) {
      std::chrono::time_point now = std::chrono::steady_clock::now();

      const auto get_image_status = ocam_camera_->GetImage(&camera_image);
      switch (get_image_status) {
        case ocam::GetImageStatus::SUCCESS:
          break;
        case ocam::GetImageStatus::NO_DATA: {
          usleep(100000);
          std::cerr << "Fail to get image for some reason." << std::endl;
          continue;  // Skip the rest of the loop
        }
        case ocam::GetImageStatus::DISCONNECTED: {
          throw std::runtime_error("The device has been disconnected.");
        }
      }

      // if (camera_image_pub.getNumSubscribers() > 0) {
      //   publishImage(camera_image, camera_image_pub, "camera_frame", now);
      // }

      // if (camera_info_pub.getNumSubscribers() > 0) {
      //   publishCamInfo(camera_info_pub, camera_info, now);
      // }

      // if (show_image_) {
      //   cv::imshow("image", camera_image);
      //   cv::waitKey(10);
      // }

      // r.sleep();
    }
  }

 private:
  // Publishers
  // rclcpp::Publisher<OdometryMsg>::SharedPtr pose_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;

  int resolution_{2};
  int frame_rate_{0};
  bool show_image_{true};
  bool config_changed_{false};
  std::string camera_frame_id_{""};
  std::unique_ptr<ocam::Camera> ocam_camera_{nullptr};
};

#define NODE_NAME "ocam_ros2_node"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<OcamRos2Node>(NODE_NAME));
    rclcpp::shutdown();
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}