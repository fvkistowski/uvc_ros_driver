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

  info_cams_.resize(n_cameras_);

  // time translator
  constexpr int kSecondsToNanoSeconds = 1e9;
  device_time_translator_.reset(
      new cuckoo_time_translator::UnwrappedDeviceTimeTranslator(
          cuckoo_time_translator::ClockParameters(kSecondsToNanoSeconds),
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

      if (setParam("CAMERA_ENABLE", static_cast<float>(buildCameraConfig())) == -1) {
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

  setParam("PARAM_DM_" + camera_name, static_cast<int>(dtype));
  setParam("PARAM_CCX_" + camera_name, p0[0]);
  setParam("PARAM_CCY_" + camera_name, p0[1]);
  setParam("PARAM_FCX_" + camera_name, fx);
  setParam("PARAM_FCY_" + camera_name, fy);
  setParam("PARAM_KC1_" + camera_name, k1);
  setParam("PARAM_KC2_" + camera_name, k2);
  setParam("PARAM_KC3_" + camera_name, r1);
  setParam("PARAM_KC4_" + camera_name, r2);
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

        StereoHomography h(cams[indx.first], cams[indx.second]);
        h.getHomography(H0, H1, f_new, p0_new, p1_new);

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

    // TODO: implement with class variables
    // SGM stereo penalty values p1: discontinuits, p2:
    setParam("STEREO_P1_CAM1", 10.0f);
    setParam("STEREO_P2_CAM1", 250.0f);
    // disparity L->R occlusion in px
    setParam("STEREO_LR_CAM1", 3.0f);
    // threshold 0-255 valid disparity
    setParam("STEREO_TH_CAM1", 140.0f);
    setParam("STEREO_FP_CAM1", 0.0f);
    setParam("STEREO_CE_CAM1", 0.0f);
    setParam("STEREO_RE_CAM1", 0.0f);
    setParam("STEREO_OF_CAM1", 0.0f);

    setParam("STEREO_MP_01", 0.0f);
    setParam("STEREO_BAYER_D", 0.0f);
    setParam("IMU_ENABLE", static_cast<float>(n_cameras_));
    setParam("ADIS_IMU", 0.0f);

    setParam("STEREO_P1_CAM3", 10.0f);
    setParam("STEREO_P2_CAM3", 250.0f);
    setParam("STEREO_LR_CAM3", 3.0f);
    setParam("STEREO_TH_CAM3", 140.0f);
    setParam("STEREO_FP_CAM3", 0.0f);
    setParam("STEREO_CE_CAM3", 0.0f);
    setParam("STEREO_RE_CAM3", 0.0f);
    setParam("STEREO_OF_CAM3", 0.0f);

    setParam("STEREO_P1_CAM5", 16.0f);
    setParam("STEREO_P2_CAM5", 240.0f);
    setParam("STEREO_LR_CAM5", 4.0f);
    setParam("STEREO_TH_CAM5", 120.0f);
    setParam("STEREO_FP_CAM5", 0.0f);
    setParam("STEREO_CE_CAM5", 0.0f);
    setParam("STEREO_RE_CAM5", 0.0f);
    setParam("STEREO_OF_CAM5", 0.0f);

    setParam("STEREO_P1_CAM7", 16.0f);
    setParam("STEREO_P2_CAM7", 240.0f);
    setParam("STEREO_LR_CAM7", 4.0f);
    setParam("STEREO_TH_CAM7", 120.0f);
    setParam("STEREO_FP_CAM7", 0.0f);
    setParam("STEREO_CE_CAM7", 0.0f);
    setParam("STEREO_RE_CAM7", 0.0f);
    setParam("STEREO_OF_CAM7", 0.0f);

    setParam("STEREO_P1_CAM9", 16.0f);
    setParam("STEREO_P2_CAM9", 240.0f);
    setParam("STEREO_LR_CAM9", 4.0f);
    setParam("STEREO_TH_CAM9", 120.0f);
    setParam("STEREO_FP_CAM9", 0.0f);
    setParam("STEREO_CE_CAM9", 0.0f);
    setParam("STEREO_RE_CAM9", 0.0f);
    setParam("STEREO_OF_CAM9", 0.0f);

    setParam("CALIB_GAIN", 4300.0f);

    setParam("CAMERA_H_FLIP", static_cast<float>(flip_));
    setParam("P_MODE", static_cast<float>(primary_camera_mode_));

    if (flip_) {
      setParam("IM_H_FLIP_CAM0", 0.0f);
      setParam("IM_V_FLIP_CAM0", 0.0f);
      setParam("IM_H_FLIP_CAM2", 0.0f);
      setParam("IM_V_FLIP_CAM2", 0.0f);
      setParam("IM_H_FLIP_CAM4", 0.0f);
      setParam("IM_V_FLIP_CAM4", 0.0f);
      setParam("IM_H_FLIP_CAM6", 0.0f);
      setParam("IM_V_FLIP_CAM6", 0.0f);
      setParam("IM_H_FLIP_CAM8", 0.0f);
      setParam("IM_V_FLIP_CAM8", 0.0f);
      setParam("IM_H_FLIP_CAM1", 1.0f);
      setParam("IM_V_FLIP_CAM1", 1.0f);
      setParam("IM_H_FLIP_CAM3", 1.0f);
      setParam("IM_V_FLIP_CAM3", 1.0f);
      setParam("IM_H_FLIP_CAM5", 1.0f);
      setParam("IM_V_FLIP_CAM5", 1.0f);
      setParam("IM_H_FLIP_CAM7", 1.0f);
      setParam("IM_V_FLIP_CAM7", 1.0f);
      setParam("IM_H_FLIP_CAM9", 1.0f);
      setParam("IM_V_FLIP_CAM9", 1.0f);
    } else {
      setParam("IM_H_FLIP_CAM0", 1.0f);
      setParam("IM_V_FLIP_CAM0", 1.0f);
      setParam("IM_H_FLIP_CAM2", 1.0f);
      setParam("IM_V_FLIP_CAM2", 1.0f);
      setParam("IM_H_FLIP_CAM4", 1.0f);
      setParam("IM_V_FLIP_CAM4", 1.0f);
      setParam("IM_H_FLIP_CAM6", 1.0f);
      setParam("IM_V_FLIP_CAM6", 1.0f);
      setParam("IM_H_FLIP_CAM8", 1.0f);
      setParam("IM_V_FLIP_CAM8", 1.0f);
      setParam("IM_H_FLIP_CAM1", 0.0f);
      setParam("IM_V_FLIP_CAM1", 0.0f);
      setParam("IM_H_FLIP_CAM3", 0.0f);
      setParam("IM_V_FLIP_CAM3", 0.0f);
      setParam("IM_H_FLIP_CAM5", 0.0f);
      setParam("IM_V_FLIP_CAM5", 0.0f);
      setParam("IM_H_FLIP_CAM7", 0.0f);
      setParam("IM_V_FLIP_CAM7", 0.0f);
      setParam("IM_H_FLIP_CAM9", 0.0f);
      setParam("IM_V_FLIP_CAM9", 0.0f);
    }
  }

  setParam("SETCALIB", float(set_calibration_));

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
    setParam("P_MODE", static_cast<float>(config.PRIMARY_CAM_MODE));

    setParam("IM_H_FLIP_CAM0", static_cast<float>(config.CAMERA_0_HFLIP));
    setParam("IM_V_FLIP_CAM0", static_cast<float>(config.CAMERA_0_VFLIP));

    setParam("ADIS_IMU", static_cast<float>(config.ADIS_IMU));
    // update camera parameters in FPGA
    setParam("UPDATEMT9V034", 1.0f);

    raw_enabled_ = config.RAW_ENABLED;
    setParam("CAMERA_ENABLE",static_cast<float>(buildCameraConfig()));

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

  constexpr uint64_t kMicroSecondsToNanoSeconds = 1e3;
  const uint64_t fpga_timestamp_64 =
      static_cast<uint64_t>(fpga_timestamp) * kMicroSecondsToNanoSeconds;

  // only update on image timestamps
  if (update_translator) {
    device_time_translator_->update(fpga_timestamp_64, ros::Time::now());
  }

  if (device_time_translator_->isReadyToTranslate()) {
    *stamp = device_time_translator_->translate(fpga_timestamp);
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

  constexpr double kDeg2Rad = 2 * M_PI / 360.0;
  constexpr double kGravity = 9.807;
  
  // Standard IMU
  //constexpr double kAccScaleFactor = kGravity / 16384.0;
  //constexpr double kGyrScaleFactor = kDeg2Rad / 131.0;
  
  // Scaling factors for ADIS.
  constexpr double kAccScaleFactor = kGravity / 4000.0;
  constexpr double kGyrScaleFactor = kDeg2Rad / 100.0;

  constexpr size_t kImuDataOffset = 8;

  double data =
      static_cast<double>(readInt16(frame, line, kImuDataOffset - element));

  if (element == ImuElement::AX || element == ImuElement::AY ||
      element == ImuElement::AZ) {
    data *= kAccScaleFactor;
  } else if (element == ImuElement::RX || element == ImuElement::RY ||
             element == ImuElement::RZ) {
    data *= kGyrScaleFactor;
  }
  return data;
}

void uvcROSDriver::extractImages(uvc_frame_t *frame,
                                 ait_ros_messages::VioSensorMsg *msg_vio) {
  // read the image data and separate the 2 images
  const size_t mixed_size = 2 * (frame->height * frame->width);
  const std::valarray<uint8_t> data(static_cast<uint8_t *>(frame->data),
                                    mixed_size);
  const std::valarray<uint8_t> left_val =
      data[std::slice(0, mixed_size / 2, 2)];
  const std::valarray<uint8_t> right_val =
      data[std::slice(1, mixed_size / 2, 2)];
  std::vector<uint8_t> left(std::begin(left_val), std::end(left_val));
  std::vector<uint8_t> right(std::begin(right_val), std::end(right_val));

  sensor_msgs::fillImage(msg_vio->left_image,
                         sensor_msgs::image_encodings::MONO8,  //
                         frame->height,                        // height
                         frame->width - 16,                    // width
                         frame->width,                         // stepSize
                         left.data());

  sensor_msgs::fillImage(msg_vio->right_image,
                         sensor_msgs::image_encodings::MONO8,  // BAYER_RGGB8,//
                         frame->height,                        // height
                         frame->width - 16,                    // width
                         frame->width,                         // stepSize
                         right.data());
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
  // flag
  uvc_cb_flag_ = true;

  ait_ros_messages::VioSensorMsg msg_vio;

  unsigned frame_size = frame->height * frame->width * 2;

  const uint8_t imu_id = extractImuId(frame);

  static uint8_t prev_count = 0;

  // update timesync
  ros::Time dummy_time;
  extractAndTranslateTimestamp(0, true, frame, &dummy_time);

  // process the IMU data
  bool is_first_imu_msg = true;
  
  static int imu_msg_skip = 0;
  constexpr int kNumImuToSkip = 36;

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

      if(kNumImuToSkip < imu_msg_skip){
        msg_vio.imu.push_back(msg_imu);
        imu_pubs_[imu_id].publish(msg_imu);
        imu_msg_skip = 0;
      }

      ++imu_msg_skip;

      prev_count = count;
    }
  }

  ROS_DEBUG("%lu imu messages", msg_vio.imu.size());
  ROS_DEBUG("imu id: %d ", imu_id);

  extractImages(frame, &msg_vio);

  const CamID cam_id = extractCamId(frame);

  if (cam_id.right_cam_num >= n_cameras_) {
    ROS_ERROR_STREAM("Tried to publish to camera " << cam_id.right_cam_num);
    return;
  }

  const uint16_t frame_counter_cam = n_cameras_ < 9 ? 0 : 8;

  static ros::Time frame_time;
  if ((cam_id.left_cam_num == frame_counter_cam) &&
      (cam_id.is_raw_images == raw_enabled_)) {
    if (!extractAndTranslateTimestamp(0, false, frame, &frame_time)) {
      ROS_ERROR("Invalid timestamp, dropping frame");
    }
    frameCounter_++;
  }

  msg_vio.header.stamp = frame_time;
  msg_vio.left_image.header.stamp = frame_time;
  msg_vio.right_image.header.stamp = frame_time;

  if (cam_id.is_raw_images) {
    msg_vio.left_image.header.frame_id =
        "cam_" + std::to_string(cam_id.left_cam_num) + "_optical_frame";
    msg_vio.right_image.header.frame_id =
        "cam_" + std::to_string(cam_id.right_cam_num) + "_optical_frame";

    if (frameCounter_ % modulo_ != 0) {
      return;
    }

    cam_raw_pubs_[cam_id.left_cam_num].publish(msg_vio.left_image);
    cam_raw_pubs_[cam_id.right_cam_num].publish(msg_vio.right_image);
  } else {
    msg_vio.left_image.header.frame_id =
        "cam_" + std::to_string(cam_id.left_cam_num) + "_corrected_frame";
    msg_vio.right_image.header.frame_id =
        "cam_" + std::to_string(cam_id.right_cam_num) + "_disparity_frame";

    // publish images
    cam_rect_pubs_[cam_id.left_cam_num / 2].publish(msg_vio.left_image);
    cam_disp_pubs_[cam_id.right_cam_num / 2].publish(msg_vio.right_image);
  }

  setCameraInfoHeader(info_cams_[cam_id.left_cam_num], width_, height_,
                      frame_time, msg_vio.left_image.header.frame_id);
  cam_info_pubs_[cam_id.left_cam_num].publish(info_cams_[cam_id.left_cam_num]);

  if (cam_id.is_raw_images) {
    setCameraInfoHeader(info_cams_[cam_id.right_cam_num], width_, height_,
                        frame_time, msg_vio.right_image.header.frame_id);

    cam_info_pubs_[cam_id.right_cam_num].publish(
        info_cams_[cam_id.right_cam_num]);
  }
}

} /* uvc */
