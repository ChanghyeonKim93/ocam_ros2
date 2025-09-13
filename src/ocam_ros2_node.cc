#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "ocam_camera.h"

class OcamRos2Node : public rclcpp::Node {
 public:
  OcamRos2Node(const std::string& node_name) : Node(node_name) {
    std::cerr << "node starts\n";
    ocam_camera_ = std::make_unique<ocam::Camera>(resolution_, frame_rate_);
    use_auto_exposure_ = false;
    exposure_ = 25;
    ocam_camera_->ControlUvc(exposure_, gain_, wb_blue_, wb_red_,
                             use_auto_exposure_);

    PreparePublishers();
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(100),
                                std::bind(&OcamRos2Node::PollDevice, this));
  }

  ~OcamRos2Node() {}

  void PreparePublishers() {
    // pose_publisher_ =
    //     this->create_publisher<OdometryMsg>("~/pose",
    //     rclcpp::SensorDataQoS());
  }

  // void callback(ocam::camConfig& config, uint32_t level) {
  //   ocam_->ControlUvc(config.exposure, config.gain, config.wb_blue,
  //                     config.wb_red, config.auto_exposure);
  // }

  void PollDevice() {
    // Reconfigure confidence
    // dynamic_reconfigure::Server<ocam::camConfig> server;
    // dynamic_reconfigure::Server<ocam::camConfig>::CallbackType f;
    // f = boost::bind(&oCamROS::callback, this, _1, _2);
    // server.setCallback(f);

    // setup publisher stuff
    // image_transport::ImageTransport it(nh);
    // image_transport::Publisher camera_image_pub =
    // it.advertise("camera/image_raw", 1);

    // ros::Publisher camera_info_pub =
    //     nh.advertise<sensor_msgs::CameraInfo>("camera/camera_info", 1);

    // sensor_msgs::CameraInfo camera_info;

    // Get config from the left, right.yaml in config
    // ROS_INFO("Loading from ROS calibration files");
    // camera_info_manager::CameraInfoManager info_manager(nh);
    // info_manager.setCameraName("camera");
    // info_manager.loadCameraInfo("package://ocam/config/camera.yaml");
    // camera_info = info_manager.getCameraInfo();
    // camera_info.header.frame_id = camera_frame_id_;

    // loop to publish images;
    ocam::Image camera_image;
    while (true) {
      std::chrono::time_point now = std::chrono::steady_clock::now();

      if (!ocam_camera_->GetImage(&camera_image)) {
        usleep(1000);
        continue;
      } else {
        std::cerr << "Success, found camera" << std::endl;
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
  double frame_rate_;
  int exposure_{10};
  int gain_{40};
  int wb_blue_{50};
  int wb_red_{50};
  bool use_auto_exposure_{true};
  bool show_image_{true};
  bool config_changed_;
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