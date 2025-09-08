#ifndef MADGWICK_FILTER_H_
#define MADGWICK_FILTER_H_

#include <iostream>
#include <queue>
#include <unordered_map>

#include "Eigen/Dense"
#include "opencv4/opencv2/core.hpp"
#include "opencv4/opencv2/highgui.hpp"
#include "opencv4/opencv2/imgproc.hpp"

#include "withrobot_camera.hpp"

namespace ocam {

struct Parameters {};

struct ImageSize {
  int width{0};
  int height{0};
};

struct Image {
  enum class ImageType { GRAY8, RGB8, BGR8, RGBA8, BGRA8 };
  std::vector<uint8_t> data;
  int width{0};
  int height{0};
  int step{0};
};

class Camera {
 private:
  Withrobot::Camera* camera;
  Withrobot::camera_format camFormat;

  ImageSize image_size_;

  int dev_num;
  bool flag0144 = false;
  std::vector<Withrobot::usb_device_info> dev_list;
  std::string devPath_;

 public:
  Camera(int resolution, double frame_rate) : camera(NULL) {
    std::unordered_map<int, ImageSize> res_map =  //
        {{0, {1280, 960}},                        //
         {1, {1280, 720}},                        //
         {2, {640, 480}},
         {3, {320, 240}}};
    GetEnumDevList(dev_list);

    camera = new Withrobot::Camera(devPath_.c_str());

    for (int i = 0; i < dev_num; i++) {
      if (flag0144 == true) {
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
    camera->set_format(image_size_.width, image_size_.height,
                       Withrobot::fourcc_to_pixformat('G', 'R', 'B', 'G'), 1,
                       (unsigned int)frame_rate);
    camera->get_current_format(camFormat);
    camFormat.print();
    /* Withrobot camera start */
    camera->start();
  }

  ~Camera() {
    camera->stop();
    delete camera;
  }

  void GetEnumDevList(std::vector<Withrobot::usb_device_info> dev_list) {
    /* enumerate device(UVC compatible devices) list */
    // std::vector<Withrobot::usb_device_info> dev_list;
    dev_num = Withrobot::get_usb_device_info_list(dev_list);
    if (dev_num < 1) {
      dev_list.clear();
      return;
    }
    for (unsigned int i = 0; i < dev_list.size(); i++) {
      if (dev_list[i].product == "oCam-1CGN-U") {
        devPath_ = dev_list[i].dev_node;
        return;
      } else if (dev_list[i].product == "oCam-1CGN-U-T") {
        devPath_ = dev_list[i].dev_node;
        return;
      } else if (dev_list[i].product == "oCam-1CGN-U-T2") {
        devPath_ = dev_list[i].dev_node;
        flag0144 = true;
        return;
      } else if (dev_list[i].product == "oCam-1MGN-U-T2") {
        devPath_ = dev_list[i].dev_node;
        flag0144 = true;
        return;
      } else if (dev_list[i].product == "oCam-1MGN-U") {
        devPath_ = dev_list[i].dev_node;
        return;
      } else if (dev_list[i].product == "oCam-1MGN-U-T") {
        devPath_ = dev_list[i].dev_node;
        return;
      }
    }
  }

  void ControlUvc(int exposure, int gain, int blue, int red,
                  bool auto_exposure) {
    /* Exposure Setting */
    camera->set_control("Exposure (Absolute)", exposure);

    /* Gain Setting */
    camera->set_control("Gain", gain);

    /* White Balance Setting */
    camera->set_control("White Balance Blue Component", blue);
    camera->set_control("White Balance Red Component", red);

    /* Auto Exposure Setting */
    if (auto_exposure)
      camera->set_control("Exposure, Auto", 0x3);
    else
      camera->set_control("Exposure, Auto", 0x1);
  }

  bool GetImage(Image* image) {
    cv::Mat srcImg(cv::Size(camFormat.width, camFormat.height), CV_8UC1);
    cv::Mat dstImg;
    // image->data.resize(camFormat.width * camFormat.height);
    // if (camera->get_frame(image->data.data(), camFormat.image_size, 1) != -1)
    // {
    if (camera->get_frame(srcImg.data, camFormat.image_size, 1) != -1) {
      cvtColor(srcImg, dstImg, cv::COLOR_BayerGR2RGB);
      cv::imshow("image", dstImg);
      cv::waitKey(5);
      return true;
    } else {
      return false;
    }
  }
};

}  // namespace ocam

#endif  // MADGWICK_FILTER_H_