/****************************************************************************
 *
 *   Copyright (c) 2015-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/*
 * uvc_ros_driver.h
 *
 *  Created on: Jul 5, 2016
 *      Author: nicolas, christoph, simone
 *
 *  The code below is based on the example provided at
 *  https://int80k.com/libuvc/doc/
 */

#ifndef __UVC_ROS_DRIVER_H__
#define __UVC_ROS_DRIVER_H__

#include <valarray>

// local include
#include "calib_yaml_interface.h"
#include "camera_info_helper.h"
#include "fpga_calibration.h"
#include "serial_port.h"
#include "stereo_homography.h"

#include "libuvc/libuvc.h"
#include "uvc_ros_driver/UvcDriverConfig.h"

#include <cuckoo_time_translator/DeviceTimeTranslator.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d/calib3d.hpp>

#include <dynamic_reconfigure/server.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/fill_image.h>
#include <std_msgs/String.h>

#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <algorithm>
#include <string>
#include <utility>  // std::pair
#include <vector>

namespace uvc {

struct CamID {
  size_t left_cam_num;
  size_t right_cam_num;
  bool is_raw_images;
};

class uvcROSDriver {
 private:
  enum ImuElement { COUNT, TEMP, AX, AY, AZ, RX, RY, RZ };

  static constexpr double kWatchdogTimeout = 0.05;
  static constexpr double kWatchdogRestartTime = 1.0;

  bool device_initialized_ = false;
  bool raw_enabled_ = true;
  bool rect_enabled_ = false;
  bool uvc_cb_flag_ = false;
  bool first_imu_received_flag_ = false;
  bool serial_port_open_ = false;
  bool adis_enabled_ = true;
  bool speckle_filter_ = false;
  bool gen_pointcloud_ = false;
  bool debayer_enabled_ = false;
  bool white_balance_enabled_ = false; 

  int n_cameras_ = 2;
  int raw_width_ = 752 + 16;  // 376+16;//
  int raw_height_ = 480;      // 240;//
  int width_ = raw_width_ - 16;
  int height_ = raw_height_;
  int frame_counter_ = 0;
  int modulo_ = 1;
  int calibration_mode_ = 0;
  bool shutdown_ = 0;

  // homography variables
  std::vector<std::pair<int, int>> homography_mapping_;
  std::vector<double> f_;
  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> p_;
  std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> H_;
  std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Vector3d>> R_;

  // Dynamic reconfigure.
  dynamic_reconfigure::Server<uvc_ros_driver::UvcDriverConfig>
      dynamic_reconfigure_;

  // disparity filtering
  int max_speckle_size_;
  int max_speckle_diff_;

  CameraParameters camera_params_;
  // serial port
  Serial_Port sp_;
  // uvc
  uvc_context_t *ctx_;
  uvc_device_t *dev_;
  uvc_device_handle_t *devh_;
  uvc_stream_ctrl_t ctrl_;

  // ros node handle
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  // image publishers
  std::vector<image_transport::Publisher> cam_raw_pubs_;
  std::vector<image_transport::Publisher> cam_rect_pubs_;
  std::vector<image_transport::Publisher> cam_disp_pubs_;
  std::vector<ros::Publisher> cam_info_pubs_;

  std::vector<sensor_msgs::CameraInfo> info_cams_;

  std::vector<ros::Publisher> imu_pubs_;

  // pointcloud publishers
  ros::Publisher pointcloud_pub_;
  ros::Publisher freespace_pointcloud_pub_;

  // tf broadcaster
  tf::TransformBroadcaster br_;

  // time translation
  std::unique_ptr<cuckoo_time_translator::UnwrappedDeviceTimeTranslator>
      device_time_translator_;

  // low level byte helper functions
  static uint8_t *rawDataPtr(const uvc_frame_t *frame, const size_t line,
                             const size_t offset);
  static uint8_t readUInt8(const uvc_frame_t *frame, const size_t line,
                           const size_t offset);
  static int16_t readInt16(const uvc_frame_t *frame, const size_t line,
                           const size_t offset);
  static uint32_t readUInt32(const uvc_frame_t *frame, const size_t line,
                             const size_t offset);

  // extract data from frames
  bool extractAndTranslateTimestamp(size_t line, bool update_translator,
                                    uvc_frame_t *frame, ros::Time *stamp);
  static CamID extractCamId(uvc_frame_t *frame);
  uint8_t extractImuId(uvc_frame_t *frame);
  static uint8_t extractImuCount(size_t line, uvc_frame_t *frame);
  bool extractImuData(size_t line, uvc_frame_t *frame, sensor_msgs::Imu *msg);
  double extractImuElementData(size_t imu_idx, ImuElement element,
                               uvc_frame_t *frame);
  void extractImages(uvc_frame_t *frame, const bool is_raw_images,
                     cv::Mat *images);

  uvc_error_t initAndOpenUvc();
  int setParam(const std::string &name, float val);
  void sendCameraParam(const int camera_number,
                       const uvc_ros_driver::DistortionModelTypes dtype,
                       const double fx, const double fy,
                       const Eigen::Vector2d &p0, const float k1,
                       const float k2, const float r1, const float r2,
                       const Eigen::Matrix3d &H);
  void setCalibration(CameraParameters camParams);

  void dynamicReconfigureCallback(uvc_ros_driver::UvcDriverConfig &config,
                                  uint32_t level);

  inline void selectCameraInfo(int camera, sensor_msgs::CameraInfo **ci);

  static void fillDisparityFromSide(const cv::Mat &input_disparity,
                                    const cv::Mat &valid, const bool &from_left,
                                    cv::Mat *filled_disparity);

  static void bulidFilledDisparityImage(const cv::Mat &input_disparity,
                                        cv::Mat *disparity_filled,
                                        cv::Mat *input_valid);

  void whiteBalance(const cv::Mat &color_image, cv::Mat *white_balanced,
                    double measure_point = 0.20);

  void calcPointCloud(
      const cv::Mat &input_disparity, const cv::Mat &left_image,
      const size_t cam_num, pcl::PointCloud<pcl::PointXYZRGB> *pointcloud,
      pcl::PointCloud<pcl::PointXYZRGB> *freespace_pointcloud) const;

  void watchdogCallback(const ros::TimerEvent &);

 public:
  uvcROSDriver(ros::NodeHandle nh) : nh_(nh), it_(nh_){};
  ~uvcROSDriver();
  void uvc_cb(uvc_frame_t *frame);
  /**
   * initialize device, set up topic publishers and compute homography for
   * the
   * cameras
   */
  void initDevice();
  /**
   * setup uvc stream
   */
  void startDevice();
  // getter and setter for different internal variables
  bool getRawEnabledMode() { return raw_enabled_; }
  void setRawEnabledMode(bool raw_enabled) { raw_enabled_ = raw_enabled; }
  bool getRectEnabledMode() { return rect_enabled_; }
  void setRectEnabledMode(bool rect_enabled) { rect_enabled_ = rect_enabled; }
  bool getCalibrationParam() { return set_calibration_; }
  int getNumberOfCameras() { return n_cameras_; }
  void setNumberOfCameras(int n_cameras) { n_cameras_ = n_cameras; }
  int buildCameraConfig() {
    int camera_config;
    switch (n_cameras_) {
      case 10:
        camera_config = 0x3FF;
        break;
      case 8:
        camera_config = 0x1EF;
        break;
      case 6:
        camera_config = 0x0E7;
        break;
      case 4:
        camera_config = 0x063;
        break;
      case 2:
      default:
        camera_config = 0x021;
        break;
    }

    if (!raw_enabled_) {
      camera_config = camera_config & 0xFE1;
    }
    if (!rect_enabled_) {
      camera_config = camera_config & 0x01F;
    }

    return camera_config;
  }

  CameraParameters getCameraParams() { return camera_params_; };
  void setCameraParams(const CameraParameters &camera_params) {
    camera_params_ = camera_params;
  };
  void getHomographyMapping(
      std::vector<std::pair<int, int>> &homography_mapping) {
    homography_mapping = homography_mapping_;
  };
  void setHomographyMapping(
      const std::vector<std::pair<int, int>> &homography_mapping) {
    homography_mapping_ = homography_mapping;
  };
  int getCalibrationMode() { return calibration_mode_; };
  void setCalibrationMode(int calibration_mode) {
    calibration_mode_ = calibration_mode;

    // update modulo_ variable also
    if (calibration_mode != 0) {
      modulo_ = 4;
    }
  };
};

}  // namespace uvc

#endif /* end of include guard: __UVC_ROS_DRIVER_H__ */
