#ifndef CALIB_YAML_INTERFACE_H_
#define CALIB_YAML_INTERFACE_H_

#include <yaml-cpp/yaml.h>
#include <string>

#include <tf/transform_broadcaster.h>

#include "logging.h"

// cameraParameters
// =========================================================
struct CameraParameters {  // parameters of one camera
  bool isValid;
  double FocalLength[10][2];
  double PrincipalPoint[10][2];
  double DistortionCoeffs[10][4];
  int CameraModel[10];
  int DistortionModel[10];

  double StereoTransformationMatrix[5][4][4];

  tf::Transform T_cam_imu[10];

  enum { PINHOLE = 0, OMNI = 1 };
  enum { RADTAN = 0, EQUI = 1 };
};

inline CameraParameters parseYaml(const YAML::Node &node) {
  CameraParameters v;
  v.isValid = true;

  std::string pinhole = "pinhole";
  std::string omni = "omni";

  std::string radtan = "radtan";
  std::string equi = "equidistant";

  //-------------------------camera models----------------------------
  for (int h = 0; h < int(node.size()); h++) {
    YAML::Node camera_model = node["cam" + std::to_string(h)]["camera_model"];

    if (!omni.compare(camera_model.as<std::string>())) {
      v.CameraModel[h] = v.OMNI;

    } else {
      v.CameraModel[h] = v.PINHOLE;
    }
  }

  //-------------------------camera transformation
  // matrix----------------------------
  for (int h = 1; h < int(node.size()); h += 2) {
    YAML::Node cam_tform_mat = node["cam" + std::to_string(h)]["T_cn_cnm1"];

    for (std::size_t i = 0; i < cam_tform_mat.size(); i++) {
      for (std::size_t j = 0; j < cam_tform_mat[i].size(); j++) {
        v.StereoTransformationMatrix[h][i][j] =
            cam_tform_mat[i][j].as<double>();
      }
    }
  }

  //-------------------------distortion models----------------------------
  for (int h = 0; h < int(node.size()); h++) {
    YAML::Node distortion_model =
        node["cam" + std::to_string(h)]["distortion_model"];

    if (!equi.compare(distortion_model.as<std::string>())) {
      v.DistortionModel[h] = v.EQUI;

    } else {
      v.DistortionModel[h] = v.RADTAN;
    }
  }

  //-------------------------distortion coeffs----------------------------
  for (int h = 0; h < int(node.size()); h++) {
    YAML::Node distortion_coeffs =
        node["cam" + std::to_string(h)]["distortion_coeffs"];

    for (std::size_t i = 0; i < distortion_coeffs.size(); i++) {
      v.DistortionCoeffs[h][i] = distortion_coeffs[i].as<double>();
    }
  }

  //-------------------------focal lengths and principal points-----------
  for (int h = 0; h < int(node.size()); h++) {
    YAML::Node intrinsics = node["cam" + std::to_string(h)]["intrinsics"];

    if (v.CameraModel[h] == v.OMNI) {
      v.FocalLength[h][0] = intrinsics[1].as<double>();
      v.FocalLength[h][1] = intrinsics[2].as<double>();
      v.PrincipalPoint[h][0] = intrinsics[3].as<double>();
      v.PrincipalPoint[h][1] = intrinsics[4].as<double>();

    } else {
      v.FocalLength[h][0] = intrinsics[0].as<double>();
      v.FocalLength[h][1] = intrinsics[1].as<double>();
      v.PrincipalPoint[h][0] = intrinsics[2].as<double>();
      v.PrincipalPoint[h][1] = intrinsics[3].as<double>();
    }
  }

  //-------------------------imu transform----------------------------
  for (int h = 0; h < int(node.size()); h++) {
    YAML::Node T_raw = node["cam" + std::to_string(h)]["T_cam_imu"];

    tf::Matrix3x3 rot_mat;
    tf::Vector3 t_vec;

    for (size_t col = 0; col < 3; ++col) {
      for (size_t row = 0; row < 3; ++row) {
        rot_mat[row][col] = T_raw[col][row].as<double>();
      }
      t_vec[col] = T_raw[col][3].as<double>();
    }

    v.T_cam_imu[h] = tf::Transform(rot_mat, t_vec);
  }

  

  return v;
}

#endif
