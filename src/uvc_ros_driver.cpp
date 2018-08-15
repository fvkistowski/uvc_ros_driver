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
 * uvc_ros_driver.cpp
 *
 *  Created on: Jul 5, 2016
 *      Author: nicolas, christoph, simone
 *
 *  The code below is based on the example provided at
 *  https://int80k.com/libuvc/doc/
 */

#include <functional>
#include <iostream>

#include <sensor_msgs/image_encodings.h>

#include "uvc_ros_driver.h"

namespace uvc {

static void callback(uvc_frame *frame, void *arg) {
  uvcROSDriver *obj = (uvcROSDriver *)arg;
  obj->uvc_cb(frame);
}

static bool myPairMax(std::pair<int, int> p, std::pair<int, int> p1) {
  // gratest value is supposed be on the second index
  return p.second < p1.second;
}

uvcROSDriver::~uvcROSDriver() {
  shutdown_ = true;

  if (serial_port_open_) {
    setParam("CAMERA_ENABLE", 0.0f);
  }
  mavlink_message_t message;
  mavlink_param_value_t param;
  bool wait = 1;
  std::string str = "CAMERA_ENABLE";

  if (serial_port_open_) {
    while (wait) {
      int res = sp_.read_message(message);
      if (res == -1) {
        serial_port_open_ = false;
        break;
      }
      if (res != 0) {
        if (message.msgid == 22) {
          mavlink_msg_param_value_decode(&message, &param);
          if (str.compare(param.param_id) == 0 && param.param_value == 0) {
            wait = 0;
          } else {
            setParam("CAMERA_ENABLE", 0.0f);
          }
        }
      }
    }

    printf("Camera Disabled \n");
    // close serial port
    sp_.close_serial();
    usleep(500000);
    uvc_stop_streaming(devh_);

    // close uvc device
    uvc_close(devh_);
    printf("Device closed");
    uvc_unref_device(dev_);
    uvc_exit(ctx_);
  }
}

////////////////////////////////////////////////////////////////////////////////

void uvcROSDriver::initDevice() {
  // initialize serial port
  // sp_ = Serial_Port("/dev/ttyUSB0", 115200);
  sp_ = Serial_Port("/dev/serial/by-id/usb-Cypress_FX3-if02", 115200);

  bool first_fault = true;
  int open = 0;
  while (true) {
    ros::spinOnce();
    if (!nh_.ok()) return;
    open = sp_.open_serial();

    if (open != -1) {
      serial_port_open_ = true;
      break;
    }
    if (first_fault) {
      ROS_ERROR(
          "Couldn't open serialport /dev/serial/by-id/usb-Cypress_FX3-if02. "
          "Will retry every second.");
      first_fault = false;
    }
    sleep(1.0);
  }

  // initialize camera image publisher
  constexpr int kCamQueueSize = 5;
  constexpr int kIMUQueueSize = 20;
  std::string prev_topic;
  for (size_t i = 0; i < n_cameras_; ++i) {
    const std::string topic = "cam_" + std::to_string(i) + "/";
    cam_raw_pubs_.emplace_back(
        it_.advertise(topic + "image_raw", kCamQueueSize));
    cam_info_pubs_.emplace_back(
        nh_.advertise<sensor_msgs::CameraInfo>(topic + "camera_info", 1000));

    if (i % 2) {
      cam_rect_pubs_.emplace_back(
          it_.advertise(prev_topic + "image_rect", kCamQueueSize));
      cam_disp_pubs_.emplace_back(
          it_.advertise(prev_topic + "image_depth", kCamQueueSize));

      imu_pubs_.emplace_back(
          nh_.advertise<sensor_msgs::Imu>(prev_topic + "imu", kIMUQueueSize));
      imu_pubs_.emplace_back(
          nh_.advertise<sensor_msgs::Imu>(topic + "imu", kIMUQueueSize));
    }
    prev_topic = topic;
  }

  imu_pubs_.emplace_back(
      nh_.advertise<sensor_msgs::Imu>("adis_imu", kIMUQueueSize));

  info_cams_.resize(n_cameras_);

  pointcloud_pub_ =
      nh_.advertise<sensor_msgs::PointCloud2>("pointcloud", kCamQueueSize);
  freespace_pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
      "freespace_pointcloud", kCamQueueSize);

  // time translator
  constexpr int kSecondsToMicroSeconds = 1e6;
  constexpr int kTimerBits = 32;
  device_time_translator_.reset(
      new cuckoo_time_translator::UnwrappedDeviceTimeTranslator(
          cuckoo_time_translator::WrappingClockParameters(
              1L << kTimerBits, kSecondsToMicroSeconds),
          nh_.getNamespace(),
          cuckoo_time_translator::Defaults().setFilterAlgorithm(
              cuckoo_time_translator::FilterAlgorithm::ConvexHull)));

  // wait on heartbeat
  std::cout << "Waiting on device.";
  fflush(stdout);
  mavlink_message_t message;
  if (serial_port_open_ && nh_.ok()) {
    int res = sp_.read_message(message);
    if (res == -1) {
      serial_port_open_ = false;
      return;
    }

    mavlink_heartbeat_t heartbeat;

    while (heartbeat.type != 9) {  // check for system type 9 heartbeat

      if (heartbeat.type != 9) {
        printf(".");
        fflush(stdout);
        usleep(50000);
      }
      if (message.msgid == 0) {
        mavlink_msg_heartbeat_decode(&message, &heartbeat);
        if (heartbeat.type == 9) {
          std::cout << std::endl;
          ROS_INFO("Got heartbeat from camera");
        }
      }
      if (serial_port_open_ && nh_.ok()) {
        int res = sp_.read_message(message);
        if (res == -1) {
          serial_port_open_ = false;
          return;
        }
      }
    }

    // set flag for completed initializiation
    device_initialized_ = true;
  }
}

////////////////////////////////////////////////////////////////////////////////

void uvcROSDriver::startDevice() {
  if (device_initialized_) {
    setCalibration(camera_params_);

    // open uvc stream
    uvc_error_t res = initAndOpenUvc();
    if (res < 0) {
      uvc_perror(res, "uvc_init");
      ROS_ERROR("Unable to open uvc device");
      return;
    }

    // start stream
    res = uvc_start_streaming(devh_, &ctrl_, &callback, this, 0);

    setParam("CAMERA_ENABLE", static_cast<float>(buildCameraConfig()));
    setCalibration(camera_params_);

    printf("Waiting on stream");
    while (!uvc_cb_flag_ && ros::ok()) {
      printf(".");
      fflush(stdout);

      if (setParam("CAMERA_ENABLE", static_cast<float>(buildCameraConfig())) ==
          -1) {
        ROS_ERROR("Device not initialized!");
        return;
      }
      usleep(200000);
    }

    ROS_INFO("Enabled Dynamic Reconfigure Callback");
    dynamic_reconfigure_.setCallback(
        std::bind(&uvcROSDriver::dynamicReconfigureCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

  } else {
    ROS_ERROR("Device not initialized!");
  }
}

////////////////////////////////////////////////////////////////////////////////

int uvcROSDriver::setParam(const std::string &name, float val) {
  mavlink_message_t msg;
  char name_buf[16] = {};

  uint8_t local_sys = 1;
  uint8_t local_comp = 1;

  uint8_t target_sys = 99;
  uint8_t target_comp = 55;

  constexpr size_t kAttempts = 2;

  // SETCALIB
  // multiple attempts, just so we maximize chances things actually go through
  for (size_t i = 0; i < kAttempts; ++i) {
    strncpy(name_buf, name.c_str(), MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
    mavlink_msg_param_set_pack(local_sys, local_comp, &msg, target_sys,
                               target_comp, name_buf, val, MAVLINK_TYPE_FLOAT);
    int ret = sp_.write_message(msg);

    if (ret <= 0) {
      printf("ret: %d\n", ret);
      return ret;
    }
  }

  return 0;
}

////////////////////////////////////////////////////////////////////////////////

void uvcROSDriver::sendCameraParam(
    const int camera_number, const uvc_ros_driver::DistortionModelTypes dtype,
    const double fx, const double fy, const Eigen::Vector2d &p0, const float k1,
    const float k2, const float r1, const float r2, const Eigen::Matrix3d &H) {
  std::string camera_name = "CAM" + std::to_string(camera_number);

  setParam("PARAM_DM_" + camera_name, static_cast<float>(dtype));
  setParam("PARAM_CCX_" + camera_name, p0[0]);
  setParam("PARAM_CCY_" + camera_name, p0[1]);
  setParam("PARAM_FCX_" + camera_name, fx);
  setParam("PARAM_FCY_" + camera_name, fy);
  setParam("PARAM_KC1_" + camera_name, k1);
  setParam("PARAM_KC2_" + camera_name, k2);
  setParam("PARAM_KC3_" + camera_name, r1);
  setParam("PARAM_KC4_" + camera_name, r2);
  setParam("PARAM_P1_" + camera_name, r1);
  setParam("PARAM_P2_" + camera_name, r2);
  setParam("PARAM_H11_" + camera_name, H(0, 0));
  setParam("PARAM_H12_" + camera_name, H(0, 1));
  setParam("PARAM_H13_" + camera_name, H(0, 2));
  setParam("PARAM_H21_" + camera_name, H(1, 0));
  setParam("PARAM_H22_" + camera_name, H(1, 1));
  setParam("PARAM_H23_" + camera_name, H(1, 2));
  setParam("PARAM_H31_" + camera_name, H(2, 0));
  setParam("PARAM_H32_" + camera_name, H(2, 1));
  setParam("PARAM_H33_" + camera_name, H(2, 2));
}

////////////////////////////////////////////////////////////////////////////////

void uvcROSDriver::setCalibration(CameraParameters camParams) {
  std::vector<uvc_ros_driver::FPGACalibration> cams;
  int stereo_number = 0;

  if (camParams.isValid) {
    // TODO: find better way for this, general case not only stereo between
    // cam0->cam1, cam2->cam3
    for (int cam = 0; cam < n_cameras_; cam++) {
      uvc_ros_driver::FPGACalibration camera;
      camera.projection_model_.projection_type_ =
          uvc_ros_driver::ProjectionModelTypes::PINHOLE;
      camera.projection_model_.distortion_type_ =
          static_cast<uvc_ros_driver::DistortionModelTypes>(
              camParams.DistortionModel[cam]);
      camera.projection_model_.focal_length_u_ = camParams.FocalLength[cam][0];
      camera.projection_model_.focal_length_v_ = camParams.FocalLength[cam][1];
      camera.projection_model_.principal_point_u_ =
          camParams.PrincipalPoint[cam][0];
      camera.projection_model_.principal_point_v_ =
          camParams.PrincipalPoint[cam][1];
      camera.projection_model_.k1_ = camParams.DistortionCoeffs[cam][0];
      camera.projection_model_.k2_ = camParams.DistortionCoeffs[cam][1];
      camera.projection_model_.r1_ = camParams.DistortionCoeffs[cam][2];
      camera.projection_model_.r2_ = camParams.DistortionCoeffs[cam][3];

      if (cam % 2 != 0) {
        camera.projection_model_.R_[0] =
            camParams.StereoTransformationMatrix[stereo_number][0][0];
        camera.projection_model_.R_[1] =
            camParams.StereoTransformationMatrix[stereo_number][0][1];
        camera.projection_model_.R_[2] =
            camParams.StereoTransformationMatrix[stereo_number][0][2];
        camera.projection_model_.R_[3] =
            camParams.StereoTransformationMatrix[stereo_number][1][0];
        camera.projection_model_.R_[4] =
            camParams.StereoTransformationMatrix[stereo_number][1][1];
        camera.projection_model_.R_[5] =
            camParams.StereoTransformationMatrix[stereo_number][1][2];
        camera.projection_model_.R_[6] =
            camParams.StereoTransformationMatrix[stereo_number][2][0];
        camera.projection_model_.R_[7] =
            camParams.StereoTransformationMatrix[stereo_number][2][1];
        camera.projection_model_.R_[8] =
            camParams.StereoTransformationMatrix[stereo_number][2][2];
        camera.projection_model_.t_[0] =
            camParams.StereoTransformationMatrix[stereo_number][0][3];
        camera.projection_model_.t_[1] =
            camParams.StereoTransformationMatrix[stereo_number][1][3];
        camera.projection_model_.t_[2] =
            camParams.StereoTransformationMatrix[stereo_number][2][3];
        stereo_number++;

      } else {
        camera.projection_model_.R_[0] = 1.0f;
        camera.projection_model_.R_[1] = 0.0f;
        camera.projection_model_.R_[2] = 0.0f;
        camera.projection_model_.R_[3] = 0.0f;
        camera.projection_model_.R_[4] = 1.0f;
        camera.projection_model_.R_[5] = 0.0f;
        camera.projection_model_.R_[6] = 0.0f;
        camera.projection_model_.R_[7] = 0.0f;
        camera.projection_model_.R_[8] = 1.0f;
        camera.projection_model_.t_[0] = 0.0f;
        camera.projection_model_.t_[1] = 0.0f;
        camera.projection_model_.t_[2] = 0.0f;
      }

      cams.push_back(camera);
    }

    // initialize vectors
    f_.resize(n_cameras_);
    p_.resize(n_cameras_);
    H_.resize(n_cameras_);
    // pointer at camera info
    sensor_msgs::CameraInfo *ci;
    size_t homography_size;

    // TODO: reimplment this part for multiple stereo base line based systems
    if (set_calibration_) {
      std::vector<std::pair<int, int> >::iterator it_homography =
          std::max_element(homography_mapping_.begin(),
                           homography_mapping_.end(), myPairMax);

      // set homography number to number of camera pairs for now
      homography_size = n_cameras_ / 2;

      for (size_t i = 0; i < homography_size; i++) {
        // temp structures
        Eigen::Matrix3d H0;
        Eigen::Matrix3d H1;
        double f_new;
        Eigen::Vector2d p0_new, p1_new;

        std::pair<int, int> indx = homography_mapping_[i];

        // hack for now do cleaner later
        double zoom = -100;

        StereoHomography h(cams[indx.first], cams[indx.second]);
        h.getHomography(H0, H1, f_new, p0_new, p1_new, zoom);

        f_[indx.first] = f_new;
        f_[indx.second] = f_new;
        p_[indx.first] = p0_new;
        p_[indx.second] = p1_new;
        // TODO check if matrix is copied or only pointer!!
        H_[indx.first] = H0;
        H_[indx.second] = H1;
      }

      // Set all parameters here
      for (int i = 0; i < n_cameras_; i++) {
        sendCameraParam(i, cams[i].projection_model_.distortion_type_, f_[i],
                        f_[i], p_[i], cams[i].projection_model_.k1_,
                        cams[i].projection_model_.k2_,
                        cams[i].projection_model_.r1_,
                        cams[i].projection_model_.r2_, H_[i]);
        setCameraInfoIntrinsics(info_cams_[i], f_[i], f_[i], p_[i](0),
                                p_[i](1));
        setCameraInfoDistortionMdl(
            info_cams_[i], uvc_ros_driver::ProjectionModelTypes::PINHOLE);
        setCameraInfoDistortionParams(info_cams_[i], 0, 0, 0, 0, 0);
      }

    } else {
      for (int i = 0; i < n_cameras_; i++) {
        setCameraInfoIntrinsics(info_cams_[i], camParams.FocalLength[i][0],
                                camParams.FocalLength[i][1],
                                camParams.PrincipalPoint[i][0],
                                camParams.PrincipalPoint[i][1]);
        setCameraInfoDistortionMdl(
            info_cams_[i], uvc_ros_driver::ProjectionModelTypes::PINHOLE);
        setCameraInfoDistortionParams(
            info_cams_[i], cams[i].projection_model_.k1_,
            cams[i].projection_model_.k2_, cams[i].projection_model_.r1_,
            cams[i].projection_model_.r2_, 0);
      }
    }
  }

  setParam("RESETMT9V034", 1.0f);
  setParam("RESETICM20608", 1.0f);
}

void uvcROSDriver::dynamicReconfigureCallback(
    uvc_ros_driver::UvcDriverConfig &config, uint32_t level) {
  if (!shutdown_) {
    setParam("CAMERA_AUTOEXP", static_cast<float>(config.CAMERA_AUTOEXP));
    setParam("CAMERA_EXP", static_cast<float>(config.CAMERA_EXP));
    setParam("CAMERA_MIN_E", static_cast<float>(config.CAMERA_MIN_E));
    setParam("CAMERA_MAX_E", static_cast<float>(config.CAMERA_MAX_E));
    setParam("CAMERA_AUTOG", static_cast<float>(config.CAMERA_AUTOG));
    setParam("CAMERA_GAIN", static_cast<float>(config.CAMERA_GAIN));
    setParam("STEREO_BAYER_D", static_cast<float>(config.STEREO_BAYER_D));

    primary_camera_mode_ = config.PRIMARY_CAM_MODE;
    setParam("P_MODE", static_cast<float>(config.PRIMARY_CAM_MODE));

    adis_enabled_ = config.ADIS_IMU;
    setParam("ADIS_IMU", static_cast<float>(config.ADIS_IMU));

    raw_enabled_ = config.RAW_ENABLED;
    setParam("CAMERA_ENABLE", static_cast<float>(buildCameraConfig()));

    setParam("IM_H_FLIP_CAM0", static_cast<float>(config.CAMERA_0_HFLIP));
    setParam("IM_V_FLIP_CAM0", static_cast<float>(config.CAMERA_0_VFLIP));
    setParam("IM_H_FLIP_CAM1", static_cast<float>(config.CAMERA_1_HFLIP));
    setParam("IM_V_FLIP_CAM1", static_cast<float>(config.CAMERA_1_VFLIP));
    setParam("IM_H_FLIP_CAM2", static_cast<float>(config.CAMERA_2_HFLIP));
    setParam("IM_V_FLIP_CAM2", static_cast<float>(config.CAMERA_2_VFLIP));
    setParam("IM_H_FLIP_CAM3", static_cast<float>(config.CAMERA_3_HFLIP));
    setParam("IM_V_FLIP_CAM3", static_cast<float>(config.CAMERA_3_VFLIP));
    setParam("IM_H_FLIP_CAM4", static_cast<float>(config.CAMERA_4_HFLIP));
    setParam("IM_V_FLIP_CAM4", static_cast<float>(config.CAMERA_4_VFLIP));
    setParam("IM_H_FLIP_CAM5", static_cast<float>(config.CAMERA_5_HFLIP));
    setParam("IM_V_FLIP_CAM5", static_cast<float>(config.CAMERA_5_VFLIP));
    setParam("IM_H_FLIP_CAM6", static_cast<float>(config.CAMERA_6_HFLIP));
    setParam("IM_V_FLIP_CAM6", static_cast<float>(config.CAMERA_6_VFLIP));
    setParam("IM_H_FLIP_CAM7", static_cast<float>(config.CAMERA_7_HFLIP));
    setParam("IM_V_FLIP_CAM7", static_cast<float>(config.CAMERA_7_VFLIP));
    setParam("IM_H_FLIP_CAM8", static_cast<float>(config.CAMERA_8_HFLIP));
    setParam("IM_V_FLIP_CAM8", static_cast<float>(config.CAMERA_8_VFLIP));
    setParam("IM_H_FLIP_CAM9", static_cast<float>(config.CAMERA_9_HFLIP));
    setParam("IM_V_FLIP_CAM9", static_cast<float>(config.CAMERA_9_VFLIP));

    // update camera parameters in FPGA
    setParam("UPDATEMT9V034", 1.0f);

    setParam("STEREO_FP_CAM1", static_cast<float>(config.STEREO_FP_CAM1));
    setParam("STEREO_RE_CAM1", static_cast<float>(config.STEREO_RE_CAM1));
    setParam("STEREO_CE_CAM1", static_cast<float>(config.STEREO_CE_CAM1));
    setParam("STEREO_TH_CAM1", static_cast<float>(config.STEREO_TH_CAM1));
    setParam("STEREO_LR_CAM1", static_cast<float>(config.STEREO_LR_CAM1));
    setParam("STEREO_OF_CAM1", static_cast<float>(config.STEREO_OF_CAM1));
    setParam("STEREO_P1_CAM1", static_cast<float>(config.STEREO_P1_CAM1));
    setParam("STEREO_P2_CAM1", static_cast<float>(config.STEREO_P2_CAM1));

    setParam("STEREO_FP_CAM3", static_cast<float>(config.STEREO_FP_CAM3));
    setParam("STEREO_RE_CAM3", static_cast<float>(config.STEREO_RE_CAM3));
    setParam("STEREO_CE_CAM3", static_cast<float>(config.STEREO_CE_CAM3));
    setParam("STEREO_TH_CAM3", static_cast<float>(config.STEREO_TH_CAM3));
    setParam("STEREO_LR_CAM3", static_cast<float>(config.STEREO_LR_CAM3));
    setParam("STEREO_OF_CAM3", static_cast<float>(config.STEREO_OF_CAM3));
    setParam("STEREO_P1_CAM3", static_cast<float>(config.STEREO_P1_CAM3));
    setParam("STEREO_P2_CAM3", static_cast<float>(config.STEREO_P2_CAM3));

    setParam("STEREO_FP_CAM5", static_cast<float>(config.STEREO_FP_CAM5));
    setParam("STEREO_RE_CAM5", static_cast<float>(config.STEREO_RE_CAM5));
    setParam("STEREO_CE_CAM5", static_cast<float>(config.STEREO_CE_CAM5));
    setParam("STEREO_TH_CAM5", static_cast<float>(config.STEREO_TH_CAM5));
    setParam("STEREO_LR_CAM5", static_cast<float>(config.STEREO_LR_CAM5));
    setParam("STEREO_OF_CAM5", static_cast<float>(config.STEREO_OF_CAM5));
    setParam("STEREO_P1_CAM5", static_cast<float>(config.STEREO_P1_CAM5));
    setParam("STEREO_P2_CAM5", static_cast<float>(config.STEREO_P2_CAM5));

    setParam("STEREO_FP_CAM7", static_cast<float>(config.STEREO_FP_CAM7));
    setParam("STEREO_RE_CAM7", static_cast<float>(config.STEREO_RE_CAM7));
    setParam("STEREO_CE_CAM7", static_cast<float>(config.STEREO_CE_CAM7));
    setParam("STEREO_TH_CAM7", static_cast<float>(config.STEREO_TH_CAM7));
    setParam("STEREO_LR_CAM7", static_cast<float>(config.STEREO_LR_CAM7));
    setParam("STEREO_OF_CAM7", static_cast<float>(config.STEREO_OF_CAM7));
    setParam("STEREO_P1_CAM7", static_cast<float>(config.STEREO_P1_CAM7));
    setParam("STEREO_P2_CAM7", static_cast<float>(config.STEREO_P2_CAM7));

    setParam("STEREO_FP_CAM9", static_cast<float>(config.STEREO_FP_CAM9));
    setParam("STEREO_RE_CAM9", static_cast<float>(config.STEREO_RE_CAM9));
    setParam("STEREO_CE_CAM9", static_cast<float>(config.STEREO_CE_CAM9));
    setParam("STEREO_TH_CAM9", static_cast<float>(config.STEREO_TH_CAM9));
    setParam("STEREO_LR_CAM9", static_cast<float>(config.STEREO_LR_CAM9));
    setParam("STEREO_OF_CAM9", static_cast<float>(config.STEREO_OF_CAM9));
    setParam("STEREO_P1_CAM9", static_cast<float>(config.STEREO_P1_CAM9));
    setParam("STEREO_P2_CAM9", static_cast<float>(config.STEREO_P2_CAM9));

    // update stereo parameters in FPGA
    setParam("SETCALIB", 1.0f);

    raw_enabled_ = config.RAW_ENABLED;
    setParam("CAMERA_ENABLE", static_cast<float>(buildCameraConfig()));

    setParam("STEREO_RE_CAM1", static_cast<float>(config.STEREO_RE_CAM1));
    setParam("STEREO_CE_CAM1", static_cast<float>(config.STEREO_CE_CAM1));
    setParam("STEREO_TH_CAM1", static_cast<float>(config.STEREO_TH_CAM1));
    setParam("STEREO_LR_CAM1", static_cast<float>(config.STEREO_LR_CAM1));
    setParam("STEREO_OF_CAM1", static_cast<float>(config.STEREO_OF_CAM1));
    setParam("STEREO_P1_CAM1", static_cast<float>(config.STEREO_P1_CAM1));
    setParam("STEREO_P2_CAM1", static_cast<float>(config.STEREO_P2_CAM1));
    setParam("STEREO_BAYER_D", static_cast<float>(config.STEREO_BAYER_D));
    // update stereo parameters in FPGA
    setParam("SETCALIB", 1.0f);

    debayer_enabled_ = config.DEBAYER;
    white_balance_enabled_ = config.WHITE_BALANCE;
    gen_pointcloud_ = config.GEN_POINTCLOUD;
    speckle_filter_ = config.SPECKLE_FILTER;
    max_speckle_size_ = config.MAX_SPECKLE_SIZE;
    max_speckle_diff_ = config.MAX_SPECKLE_DIFF;
  }
}

////////////////////////////////////////////////////////////////////////////////
// NOTE: return error really necessary?
uvc_error_t uvcROSDriver::initAndOpenUvc() {
  uvc_error_t res;
  /* Initialize a UVC service context. Libuvc will set up its own libusb
   * context. Replace NULL with a libusb_context pointer to run libuvc
   * from an existing libusb context. */
  res = uvc_init(&ctx_, NULL);

  if (res < 0) {
    uvc_perror(res, "uvc_init");
    ROS_ERROR("Unable to initialize uvc service context");
    return res;
  }

  /* Locates the first attached UVC device, stores in dev */
  /* filter devices: vendor_id, product_id, "serial_num" */
  res = uvc_find_device(ctx_, &dev_, 0x04b4, 0, NULL);

  if (res < 0) {
    uvc_perror(res, "uvc_find_device"); /* no devices found */
    ROS_ERROR("No devices found");
    return res;
  }

  ROS_INFO("Device found");

  /* Try to open the device: requires exclusive access */
  res = uvc_open(dev_, &devh_);

  if (res < 0) {
    uvc_perror(res, "uvc_open"); /* unable to open device */
    ROS_ERROR("Unable to open the device");
    return res;
  }

  /* Try to negotiate a 640x480 30 fps YUYV stream profile */
  res = uvc_get_stream_ctrl_format_size(
      devh_, &ctrl_,              /* result stored in ctrl */
      UVC_FRAME_FORMAT_YUYV,      /* YUV 422, aka YUV 4:2:2. try _COMPRESSED */
      raw_width_, raw_height_, 30 /* width, height, fps */
  );

  if (res < 0) {
    /* device doesn't provide a matching stream */
    uvc_perror(res, "get_mode");
    ROS_ERROR("Device doesn't provide a matching stream");
    return res;
  }

  return res;
}

uint8_t *uvcROSDriver::rawDataPtr(const uvc_frame_t *frame, const size_t line,
                                  const size_t offset) {
  return &(static_cast<uint8_t *>(
      frame->data)[2 * ((line + 1) * frame->width - offset)]);
}

uint8_t uvcROSDriver::readUInt8(const uvc_frame_t *frame, const size_t line,
                                const size_t offset) {
  return *rawDataPtr(frame, line, offset);
}

int16_t uvcROSDriver::readInt16(const uvc_frame_t *frame, const size_t line,
                                const size_t offset) {
  const uint8_t *data = rawDataPtr(frame, line, offset);
  uint16_t value = static_cast<uint16_t>((data[1] << 0) | (data[0] << 8));
  return *reinterpret_cast<int16_t *>(&value);
}

uint32_t uvcROSDriver::readUInt32(const uvc_frame_t *frame, const size_t line,
                                  const size_t offset) {
  const uint8_t *data = rawDataPtr(frame, line, offset);

  const uint32_t value =
      (data[3] << 0) | (data[2] << 8) | (data[1] << 16) | (data[0] << 24);
  return value;
}

bool uvcROSDriver::extractAndTranslateTimestamp(size_t line,
                                                bool update_translator,
                                                uvc_frame_t *frame,
                                                ros::Time *stamp) {
  // read out micro second timestamp
  constexpr size_t kTimestampOffset = 10;
  const uint32_t fpga_timestamp = readUInt32(frame, line, kTimestampOffset);

  if (fpga_timestamp == 0) {
    return false;
  }

  const uint64_t fpga_timestamp_64 = static_cast<uint64_t>(fpga_timestamp);

  // only update on image timestamps
  if (update_translator) {
    static uint32_t prev_fpga_timestamp;  // only needed for debugging
    try {
      device_time_translator_->update(fpga_timestamp_64, ros::Time::now());
      prev_fpga_timestamp = fpga_timestamp;
    } catch (const std::exception &e) {
      ROS_ERROR_STREAM("Caught exception during time update: " << e.what());
      ROS_ERROR_STREAM("Current fpga timestamp: " << fpga_timestamp);
      ROS_ERROR_STREAM("Previous fpga timestamp: " << prev_fpga_timestamp);

      constexpr int kSecondsToMicroSeconds = 1e6;
      constexpr int kTimerBits = 32;
      device_time_translator_.reset(
          new cuckoo_time_translator::UnwrappedDeviceTimeTranslator(
              cuckoo_time_translator::WrappingClockParameters(
                  1L << kTimerBits, kSecondsToMicroSeconds),
              nh_.getNamespace(),
              cuckoo_time_translator::Defaults().setFilterAlgorithm(
                  cuckoo_time_translator::FilterAlgorithm::ConvexHull)));

      return false;
    }
  }

  if (device_time_translator_->isReadyToTranslate()) {
    *stamp = device_time_translator_->translate(
        device_time_translator_->unwrapEventStamp(fpga_timestamp).getValue());
  } else {
    return false;
  }

  return true;
}

CamID uvcROSDriver::extractCamId(uvc_frame_t *frame) {
  constexpr uint8_t kMaxCams = 10;
  constexpr size_t kCamIdOffset = 8;
  constexpr size_t kCamIdLine = 0;
  constexpr size_t kCamIdShift = 4;
  constexpr size_t kImagesPerFrame = 2;
  constexpr size_t kRectOffset = 8;

  const uint8_t raw_id =
      readUInt8(frame, kCamIdLine, kCamIdOffset) >> kCamIdShift;

  CamID cam_id;
  if (raw_id < kMaxCams / 2) {
    cam_id.left_cam_num = kImagesPerFrame * raw_id;
    cam_id.right_cam_num = cam_id.left_cam_num + 1;
    cam_id.is_raw_images = true;
  } else {
    cam_id.left_cam_num = kImagesPerFrame * (raw_id - kRectOffset);
    cam_id.right_cam_num = cam_id.left_cam_num;
    cam_id.is_raw_images = false;
  }

  return cam_id;
}

uint8_t uvcROSDriver::extractImuId(uvc_frame_t *frame) {
  if (adis_enabled_) {
    return imu_pubs_.size() - 1;
  }

  constexpr size_t kImuIdOffset = 8;
  constexpr size_t kImuIdLine = 0;
  constexpr uint8_t kImuIdMask = 0x0F;

  return readUInt8(frame, kImuIdLine, kImuIdOffset) & kImuIdMask;
}

uint8_t uvcROSDriver::extractImuCount(size_t line, uvc_frame_t *frame) {
  constexpr size_t kImuCountOffset = 9;
  return readUInt8(frame, line, kImuCountOffset);
}

bool uvcROSDriver::extractImuData(size_t line, uvc_frame_t *frame,
                                  sensor_msgs::Imu *msg) {
  if (!extractAndTranslateTimestamp(line, false, frame, &msg->header.stamp)) {
    return false;
  }

  msg->linear_acceleration.x =
      extractImuElementData(line, ImuElement::AX, frame);
  msg->linear_acceleration.y =
      extractImuElementData(line, ImuElement::AY, frame);
  msg->linear_acceleration.z =
      extractImuElementData(line, ImuElement::AZ, frame);

  msg->angular_velocity.x = extractImuElementData(line, ImuElement::RX, frame);
  msg->angular_velocity.y = extractImuElementData(line, ImuElement::RY, frame);
  msg->angular_velocity.z = extractImuElementData(line, ImuElement::RZ, frame);

  return true;
}

double uvcROSDriver::extractImuElementData(size_t line, ImuElement element,
                                           uvc_frame_t *frame) {
  constexpr double kDeg2Rad = M_PI / 180.0;
  constexpr double kGravity = 9.807;

  // Standard IMU
  double acc_scale_factor = kGravity;
  double gyro_scale_factor = kDeg2Rad;

  if (adis_enabled_) {
    acc_scale_factor /= 4000.0;
    gyro_scale_factor /= 100.0;
  } else {
    acc_scale_factor /= 16384.0;
    gyro_scale_factor /= 131.0;
  }

  constexpr size_t kImuDataOffset = 8;

  double data =
      static_cast<double>(readInt16(frame, line, kImuDataOffset - element));

  if (element == ImuElement::AX || element == ImuElement::AY ||
      element == ImuElement::AZ) {
    data *= acc_scale_factor;
  } else if (element == ImuElement::RX || element == ImuElement::RY ||
             element == ImuElement::RZ) {
    data *= gyro_scale_factor;
  }
  return data;
}

void uvcROSDriver::extractImages(uvc_frame_t *frame, const bool is_raw_images,
                                 cv::Mat *images) {
  // read the image data and separate the 2 images
  const cv::Mat input_image(frame->height, frame->width, CV_8UC2, frame->data);

  cv::split(input_image(cv::Rect(0, 0, frame->width - 16, frame->height)),
            images);

  // if second channel is the disparity, apply filtering
  if (!is_raw_images && speckle_filter_) {
    images[1].convertTo(images[1], CV_16SC1);
    cv::filterSpeckles(images[1], 0, max_speckle_size_, max_speckle_diff_);
    images[1].convertTo(images[1], CV_8UC1);
  }
}

// simply replaces invalid disparity values with a valid value found by scanning
// horizontally (note: if disparity values are already valid or if no valid
// value can be found int_max is inserted)
void uvcROSDriver::fillDisparityFromSide(const cv::Mat &input_disparity,
                                         const cv::Mat &valid,
                                         const bool &from_left,
                                         cv::Mat *filled_disparity) {
  *filled_disparity =
      cv::Mat(input_disparity.rows, input_disparity.cols, CV_8UC1);

  for (size_t y_pixels = 0; y_pixels < input_disparity.rows; ++y_pixels) {
    bool prev_valid = false;
    uint8_t prev_value;

    for (size_t x_pixels = 0; x_pixels < input_disparity.cols; ++x_pixels) {
      size_t x_scan;
      if (from_left) {
        x_scan = x_pixels;
      } else {
        x_scan = (input_disparity.cols - x_pixels - 1);
      }

      if (valid.at<uint8_t>(y_pixels, x_scan)) {
        prev_valid = true;
        prev_value = input_disparity.at<uint8_t>(y_pixels, x_scan);
        filled_disparity->at<uint8_t>(y_pixels, x_scan) =
            std::numeric_limits<uint8_t>::max();
      } else if (prev_valid) {
        filled_disparity->at<uint8_t>(y_pixels, x_scan) = prev_value;
      } else {
        filled_disparity->at<uint8_t>(y_pixels, x_scan) =
            std::numeric_limits<uint8_t>::max();
      }
    }
  }
}

void uvcROSDriver::bulidFilledDisparityImage(const cv::Mat &input_disparity,
                                             cv::Mat *disparity_filled,
                                             cv::Mat *input_valid) {
  // mark valid pixels
  *input_valid = cv::Mat(input_disparity.rows, input_disparity.cols, CV_8U);

  for (size_t y_pixels = 0; y_pixels < input_disparity.rows; ++y_pixels) {
    for (size_t x_pixels = 0; x_pixels < input_disparity.cols; ++x_pixels) {
      if (input_disparity.at<uint8_t>(y_pixels, x_pixels) < 32) {
        input_valid->at<uint8_t>(y_pixels, x_pixels) = 0;
      } else {
        input_valid->at<uint8_t>(y_pixels, x_pixels) = 1;
      }
    }
  }

  // take a guess for the depth of the invalid pixels by scanning along the row
  // and giving them the same value as the closest horizontal point.
  cv::Mat disparity_filled_left, disparity_filled_right;
  fillDisparityFromSide(input_disparity, *input_valid, true,
                        &disparity_filled_left);
  fillDisparityFromSide(input_disparity, *input_valid, false,
                        &disparity_filled_right);

  // take the most conservative disparity of the two
  *disparity_filled = cv::max(disparity_filled_left, disparity_filled_right);
}

void uvcROSDriver::whiteBalance(const cv::Mat &color_image,
                                cv::Mat *white_balanced, double measure_point) {
  if (color_image.channels() == 1) {
    ROS_ERROR_STREAM("White balancing requested on a grayscale image");
    return;
  }

  constexpr int kIntenstiyMax = 255;
  constexpr int kIntenstiyMin = 0;
  constexpr int kHistSize = kIntenstiyMax - kIntenstiyMin + 1;

  std::vector<cv::Mat> color_channels(color_image.channels());
  cv::split(color_image, color_channels);

  const int pixels_at_measure_point =
      measure_point * color_image.cols * color_image.rows;

  std::vector<int> min_intensity, max_intensity;
  int white_point = kIntenstiyMin;
  int black_point = kIntenstiyMax;

  for (const cv::Mat &channel : color_channels) {
    cv::Mat hist;
    std::vector<cv::Mat> single_color_channel = {channel};
    cv::calcHist(single_color_channel, {0}, cv::Mat(), hist, {kHistSize},
                 {kIntenstiyMin, kIntenstiyMax}, false);

    // get intensity i of image for which pixels_at_measure_point pixels < i
    int j = 0;
    for (size_t sum = 0; sum < pixels_at_measure_point;
         sum += hist.at<float>(j++))
      ;
    min_intensity.push_back(j);

    // same but for upper
    j = kHistSize - 1;
    for (size_t sum = 0; sum < pixels_at_measure_point;
         sum += hist.at<float>(j--))
      ;
    max_intensity.push_back(j);

    white_point = std::max(max_intensity.back(), white_point);
    black_point = std::min(min_intensity.back(), black_point);
  }

  // strech color channels so the average of all pixels is grey
  for (size_t i = 0; i < max_intensity.size(); ++i) {
    double low = kIntenstiyMin - black_point + min_intensity[i];
    double high = kIntenstiyMax - white_point + max_intensity[i];
    double slope = (kIntenstiyMax - kIntenstiyMin) / (high - low);
    color_channels[i] = slope * color_channels[i] - low;
  }

  cv::merge(color_channels, *white_balanced);
}

void uvcROSDriver::calcPointCloud(
    const cv::Mat &input_disparity, const cv::Mat &left_image,
    const size_t cam_num, pcl::PointCloud<pcl::PointXYZRGB> *pointcloud,
    pcl::PointCloud<pcl::PointXYZRGB> *freespace_pointcloud) const {
  const double focal_length = f_[cam_num];
  const double cx = p_[cam_num].x();
  const double cy = p_[cam_num].y();

  const double baseline = 0.1;

  pointcloud->clear();
  freespace_pointcloud->clear();

  if (left_image.depth() != CV_8U) {
    ROS_ERROR(
        "Pointcloud generation is currently only supported on 8 bit images");
    return;
  }

  cv::Mat disparity_filled, input_valid;
  bulidFilledDisparityImage(input_disparity, &disparity_filled, &input_valid);

  // build pointcloud
  for (int y_pixels = 0; y_pixels < input_disparity.rows; ++y_pixels) {
    for (int x_pixels = 0; x_pixels < input_disparity.cols; ++x_pixels) {
      const uint8_t &is_valid = input_valid.at<uint8_t>(y_pixels, x_pixels);
      const uint8_t &input_value =
          input_disparity.at<uint8_t>(y_pixels, x_pixels);
      const uint8_t &filled_value =
          disparity_filled.at<uint8_t>(y_pixels, x_pixels);

      bool freespace;
      double disparity_value;

      // if the filled disparity is valid it must be a freespace ray
      if (filled_value < std::numeric_limits<uint8_t>::max()) {
        disparity_value = static_cast<double>(filled_value);
        freespace = true;
      }
      // else it is a normal ray
      else if (is_valid) {
        disparity_value = static_cast<double>(input_value);
        freespace = false;
      } else {
        continue;
      }

      pcl::PointXYZRGB point;

      point.z = (8 * focal_length * baseline) / disparity_value;
      point.x = point.z * (x_pixels - cx) / focal_length;
      point.y = point.z * (y_pixels - cy) / focal_length;

      if (left_image.channels() == 3) {
        const cv::Vec3b &color = left_image.at<cv::Vec3b>(y_pixels, x_pixels);
        point.b = color[0];
        point.g = color[1];
        point.r = color[2];
      } else if (left_image.channels() == 4) {
        const cv::Vec4b &color = left_image.at<cv::Vec4b>(y_pixels, x_pixels);
        point.b = color[0];
        point.g = color[1];
        point.r = color[2];
      } else {
        point.b = left_image.at<uint8_t>(y_pixels, x_pixels);
        point.g = point.b;
        point.r = point.b;
      }

      if (freespace) {
        freespace_pointcloud->push_back(point);
      } else {
        pointcloud->push_back(point);
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////

/* This callback function runs once per frame. Use it to perform any
  quick processing you need, or have it put the frame into your application's
  input queue.If this function takes too long, you'll start losing frames. */
void uvcROSDriver::uvc_cb(uvc_frame_t *frame) {
  // check if evrytstartedhing ok
  if (!ros::ok()) {
    return;
  }

  if (!uvc_cb_flag_) {
    std::cout << std::endl;
    ROS_INFO("Stream started");
  }
  uvc_cb_flag_ = true;

  sensor_msgs::Image msg_left_image;
  sensor_msgs::Image msg_right_image;

  const uint8_t imu_id = extractImuId(frame);
  const CamID cam_id = extractCamId(frame);

  const uint16_t frame_counter_cam =
      ((n_cameras_ < 9) || primary_camera_mode_) ? 0 : 8;

  static ros::Time frame_time;
  if ((cam_id.left_cam_num == frame_counter_cam) &&
      (cam_id.is_raw_images == raw_enabled_)) {
    if (!extractAndTranslateTimestamp(0, true, frame, &frame_time)) {
      ROS_ERROR("Invalid timestamp, dropping frame");
    }
    frame_counter_++;
  }

  cv::Mat images[2];
  extractImages(frame, cam_id.is_raw_images, images);

  cv_bridge::CvImage left;

  if (cam_id.is_raw_images && debayer_enabled_) {
    left.encoding = sensor_msgs::image_encodings::BGR8;
    cv::Mat color_image;
    cvtColor(images[0], color_image, cv::COLOR_BayerBG2BGR);

    if (white_balance_enabled_) {
      cv::Mat white_balanced;
      whiteBalance(color_image, &white_balanced);
      left.image = white_balanced;
    } else {
      left.image = color_image;
    }
  } else {
    left.encoding = sensor_msgs::image_encodings::MONO8;
    left.image = images[0];
  }
  msg_left_image = *left.toImageMsg();

  cv_bridge::CvImage right;
  right.encoding = sensor_msgs::image_encodings::MONO8;
  right.image = images[1];
  msg_right_image = *right.toImageMsg();

  if (cam_id.right_cam_num >= n_cameras_) {
    ROS_ERROR_STREAM("Tried to publish to camera " << cam_id.right_cam_num);
    return;
  }

  msg_left_image.header.stamp = frame_time;
  msg_right_image.header.stamp = frame_time;

  if (cam_id.is_raw_images) {
    msg_left_image.header.frame_id =
        "cam_" + std::to_string(cam_id.left_cam_num) + "_optical_frame";
    msg_right_image.header.frame_id =
        "cam_" + std::to_string(cam_id.right_cam_num) + "_optical_frame";

    if (frame_counter_ % modulo_ != 0) {
      return;
    }

    cam_raw_pubs_[cam_id.left_cam_num].publish(msg_left_image);
    cam_raw_pubs_[cam_id.right_cam_num].publish(msg_right_image);
  } else {
    msg_left_image.header.frame_id =
        "cam_" + std::to_string(cam_id.left_cam_num) + "_corrected_frame";
    msg_right_image.header.frame_id =
        "cam_" + std::to_string(cam_id.right_cam_num) + "_disparity_frame";

    // publish images
    cam_rect_pubs_[cam_id.left_cam_num / 2].publish(msg_left_image);
    cam_disp_pubs_[cam_id.right_cam_num / 2].publish(msg_right_image);

    if (gen_pointcloud_) {
      pcl::PointCloud<pcl::PointXYZRGB> pointcloud, freespace_pointcloud;
      calcPointCloud(images[1], images[0], cam_id.left_cam_num, &pointcloud,
                     &freespace_pointcloud);

      sensor_msgs::PointCloud2 pointcloud_msg;
      pcl::toROSMsg(pointcloud, pointcloud_msg);
      pointcloud_msg.header = msg_left_image.header;
      pointcloud_pub_.publish(pointcloud_msg);

      sensor_msgs::PointCloud2 freespace_pointcloud_msg;
      pcl::toROSMsg(freespace_pointcloud, freespace_pointcloud_msg);
      freespace_pointcloud_msg.header = msg_left_image.header;
      freespace_pointcloud_pub_.publish(freespace_pointcloud_msg);
    }
  }

  setCameraInfoHeader(info_cams_[cam_id.left_cam_num], width_, height_,
                      frame_time, msg_left_image.header.frame_id);
  cam_info_pubs_[cam_id.left_cam_num].publish(info_cams_[cam_id.left_cam_num]);

  br_.sendTransform(
      tf::StampedTransform(camera_params_.T_cam_imu[cam_id.left_cam_num],
                           frame_time, "imu", msg_left_image.header.frame_id));

  if (cam_id.is_raw_images) {
    setCameraInfoHeader(info_cams_[cam_id.right_cam_num], width_, height_,
                        frame_time, msg_right_image.header.frame_id);

    cam_info_pubs_[cam_id.right_cam_num].publish(
        info_cams_[cam_id.right_cam_num]);

    br_.sendTransform(tf::StampedTransform(
        camera_params_.T_cam_imu[cam_id.right_cam_num], frame_time, "imu",
        msg_right_image.header.frame_id));
  }

  static uint8_t prev_count = 0;

  // process the IMU data
  bool is_first_imu_msg = true;

  static int imu_msg_skip = 0;
  constexpr int kNumImuToSkip = 18;

  for (size_t i = 0; i < frame->height; ++i) {
    sensor_msgs::Imu msg_imu;

    const uint8_t count = extractImuCount(i, frame);

    if ((count != prev_count) && extractImuData(i, frame, &msg_imu)) {
      // the first imu message of a frame sometimes has a bad timestamp, as we
      // get them at 862 Hz we just throw it out rather then try correct for
      // this
      if (is_first_imu_msg) {
        is_first_imu_msg = false;
        continue;
      }

      if (kNumImuToSkip < imu_msg_skip) {
        imu_pubs_[imu_id].publish(msg_imu);
        imu_msg_skip = 0;
      }

      ++imu_msg_skip;

      prev_count = count;
    }
  }

  ROS_DEBUG("imu id: %d ", imu_id);
}  // namespace uvc

}  // namespace uvc
