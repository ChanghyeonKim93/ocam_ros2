#ifndef OCAM_CAMERA_H_
#define OCAM_CAMERA_H_

#include <iostream>
#include <queue>
#include <unordered_map>
#include <unordered_set>

#include "Eigen/Dense"
#include "opencv4/opencv2/core.hpp"
#include "opencv4/opencv2/highgui.hpp"
#include "opencv4/opencv2/imgproc.hpp"

#include "types.h"
#include "withrobot_camera.hpp"

namespace ocam {

struct Parameters {};

struct DeviceInfo {
  std::string path{""};
  int id{-1};
  bool flag0144{false};
};

class Camera {
 public:
  Camera(const int resolution, const double frame_rate) : camera_{nullptr} {
    std::unordered_map<int, ImageSize> resolution_map =  //
        {{0, {1280, 960}},                               //
         {1, {1280, 720}},                               //
         {2, {640, 480}},                                //
         {3, {320, 240}}};
    device_info_ = GetDeviceInfo();

    camera_ = std::make_unique<Withrobot::Camera>(device_info_.path.c_str());

    for (int i = 0; i < device_info_.id; i++) {
      // This flag indicates whether the camera is a specific model
      // (oCam-1CGN-U-T2 or oCam-1MGN-U-T2) that requires different image
      // resolutions.
      if (device_info_.flag0144) {
        if (resolution == 0)
          image_size_ = {1280, 800};
        else if (resolution == 1)
          image_size_ = {1280, 720};
        else if (resolution == 2)
          image_size_ = {640, 480};
        else if (resolution == 3)
          image_size_ = {640, 400};
        else if (resolution == 4)
          image_size_ = {320, 240};
      } else {
        if (resolution == 0)
          image_size_ = {1280, 960};
        else if (resolution == 1)
          image_size_ = {1280, 720};
        else if (resolution == 2)
          image_size_ = {640, 480};
        else if (resolution == 3)
          image_size_ = {320, 240};
      }
    }
    if (image_size_.width == 0 || image_size_.height == 0)
      throw std::runtime_error("Unsupported resolution");

    // Initialize camera format
    camera_->set_format(image_size_.width, image_size_.height,
                        Withrobot::fourcc_to_pixformat('G', 'R', 'B', 'G'), 1,
                        (unsigned int)frame_rate);
    camera_->get_current_format(camera_format_);
    camera_format_.print();

    /* Withrobot camera start */
    camera_->start();
  }

  ~Camera() { camera_->stop(); }

  DeviceInfo GetDeviceInfo() {
    static std::unordered_set<std::string> supported_product_names = {
        "oCam-1CGN-U", "oCam-1CGN-U-T", "oCam-1CGN-U-T2",
        "oCam-1MGN-U", "oCam-1MGN-U-T", "oCam-1MGN-U-T2"};
    static std::unordered_set<std::string> product_names_with_flag0144 = {
        "oCam-1CGN-U-T2", "oCam-1MGN-U-T2"};

    /* enumerate device(UVC compatible devices) list */
    std::vector<Withrobot::usb_device_info> device_info_list;
    DeviceInfo device_info;
    device_info.id = Withrobot::get_usb_device_info_list(device_info_list);
    if (device_info.id < 1) return device_info;

    for (size_t i = 0; i < device_info_list.size(); i++) {
      const auto& product_name = device_info_list[i].product;
      if (supported_product_names.find(product_name) ==
          supported_product_names.end())
        continue;

      device_info.path = device_info_list[i].dev_node;
      if (product_names_with_flag0144.find(product_name) !=
          product_names_with_flag0144.end())
        device_info.flag0144 = true;

      break;
    }
    return device_info;
  }

  void ControlUvc(const int exposure, const int gain, const int blue,
                  const int red, const bool auto_exposure) {
    /* Exposure Setting */
    camera_->set_control("Exposure (Absolute)", exposure);

    /* Gain Setting */
    camera_->set_control("Gain", gain);

    /* White Balance Setting */
    camera_->set_control("White Balance Blue Component", blue);
    camera_->set_control("White Balance Red Component", red);

    /* Auto Exposure Setting */
    auto_exposure ? camera_->set_control("Exposure, Auto", 0x3)
                  : camera_->set_control("Exposure, Auto", 0x1);
  }

  bool GetImage(Image* image) {
    cv::Mat srcImg(cv::Size(camera_format_.width, camera_format_.height),
                   CV_8UC1);
    cv::Mat dstImg;
    // image->data.resize(camFormat.width * camFormat.height);
    // if (camera->get_frame(image->data.data(), camFormat.image_size, 1) != -1)
    // {
    if (camera_->get_frame(srcImg.data, camera_format_.image_size, 1) != -1) {
      cvtColor(srcImg, dstImg, cv::COLOR_BayerGR2RGB);
      cv::imshow("image", dstImg);
      cv::waitKey(5);
      return true;
    } else {
      return false;
    }
  }

 private:
  std::unique_ptr<Withrobot::Camera> camera_{nullptr};
  Withrobot::camera_format camera_format_;
  DeviceInfo device_info_;
  ImageSize image_size_;
};

}  // namespace ocam

#endif  // OCAM_CAMERA_H_
