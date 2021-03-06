#include "stereo_homography.h"

#include <iostream>
#include <iomanip>
#include <cmath>

Eigen::Vector2d compDistortionOulu(const Eigen::Vector2d &xd,
				   const double d[5])
{

	const double k1 = d[0];
	const double k2 = d[1];
	const double k3 = d[4];
	const double p1 = d[2];
	const double p2 = d[3];

	Eigen::Vector2d x = xd;
	std::setprecision(9);

	for (int i = 0; i < 20; i++) {
		double r_2 = x.squaredNorm();
		double k_radial =
			1 + k1 * r_2 + k2 * std::pow(r_2, 2) + k3 * std::pow(r_2, 3);
		Eigen::Vector2d delta_x;
		delta_x(0) = 2 * p1 * x(0) * x(1) + p2 * (r_2 + 2 * std::pow(x(0), 2));
		delta_x(1) = p1 * (r_2 + 2 * std::pow(x(1), 2)) + 2 * p2 * x(0) * x(1);
		x = (xd - delta_x) / k_radial;
	}

	return x;
}

Eigen::Vector2d projectPoint2(const Eigen::Vector3d &X,
			      const Eigen::Matrix3d &R, const double f)
{

	Eigen::Vector3d Y = R * X;
	Eigen::Vector2d x;
	x(0) = Y(0) * f / Y(2);
	x(1) = Y(1) * f / Y(2);
	return x;
}

Eigen::Vector3d normalizePixel(const Eigen::Vector2d &x_kk, const double f[2],
			       const double p[2], const double d[5])
{

	// Subtract principal point, and divide by the focal length:
	Eigen::Vector2d x_distort;
	x_distort(0) = (x_kk(0) - p[0]) / f[0];
	x_distort(1) = (x_kk(1) - p[1]) / f[1];
	x_distort = compDistortionOulu(x_distort, d);
	Eigen::Vector3d x_distort_norm;

	x_distort_norm(0) = x_distort(0);
	x_distort_norm(1) = x_distort(1);
	x_distort_norm(2) = 1.0;
	return x_distort_norm;
}

/*
 * Constructor to compute homography, based to Domink Honeggers Matlab script
 *
 * Input:
 * double r0[9]: Rotation matrix describing relative rotation from imu to cam0.
 *IMPORTANT: r is column-major order!
 * double t0[3]: translation of imu relative to cam0 expressed in cam0 frame
 * double f0[2]: Focal length cam0
 * double p0[2]: Prinicpal point cam0
 * double d0[5]: distortion parameters (radtan model)
 * double r1[9]: Rotation matrix describing relative rotation from imu to cam1.
 *IMPORTANT: r is column-major order!
 * double t1[3]: translation of imu relative to cam1 expressed in cam1 frame
 * double f1[2]: Focal length cam1
 * double p1[2]: Prinicpal point cam1
 * double d1[5]: distortion parameters (radtan model)
 *
 * int image_width: image width in pixels
 * int imake_height: image height in pixels
 */
StereoHomography::StereoHomography(
	const uvc_ros_driver::FPGACalibration &calib_cam0,
	const uvc_ros_driver::FPGACalibration &calib_cam1)
	: image_width_(752), image_height_(480)
{
	if (calib_cam0.projection_model_.projection_type_ ==
	    uvc_ros_driver::ProjectionModelTypes::PINHOLE) {
		const uvc_ros_driver::CameraProjectionModel *cam0_projection_model =
			calib_cam0.getProjectionModel();
		const uvc_ros_driver::CameraProjectionModel *cam1_projection_model =
			calib_cam1.getProjectionModel();
		
		equi_model0_ = cam0_projection_model->distion_type_;
		equi_model1_ = cam1_projection_model->distion_type_;

		f0_[0] = cam0_projection_model->focal_length_u_;
		f0_[1] = cam0_projection_model->focal_length_v_;
		p0_[0] = cam0_projection_model->principal_point_u_;
		p0_[1] = cam0_projection_model->principal_point_v_;
		f1_[0] = cam1_projection_model->focal_length_u_;
		f1_[1] = cam1_projection_model->focal_length_v_;
		p1_[0] = cam1_projection_model->principal_point_u_;
		p1_[1] = cam1_projection_model->principal_point_v_;
		d0_[0] = cam0_projection_model->k1_;
		d0_[1] = cam0_projection_model->k2_;
		d0_[2] = cam0_projection_model->r1_;
		d0_[3] = cam0_projection_model->r2_;
		d1_[0] = cam1_projection_model->k1_;
		d1_[1] = cam1_projection_model->k2_;
		d1_[2] = cam1_projection_model->r1_;
		d1_[3] = cam1_projection_model->r2_;

		for (int i = 0; i < 3; ++i) {
			t0_[i] = cam0_projection_model->t_[i];
			t1_[i] = cam1_projection_model->t_[i];
		}

		for (int i = 0; i < 9; ++i) {
			r0_[i] = cam0_projection_model->R_[i];
			r1_[i] = cam1_projection_model->R_[i];
		}

	} else {
		printf("current projection model not supported\n");
	}
}

// Computes homography for stereo rectification
void StereoHomography::getHomography(Eigen::Matrix3d &H0, Eigen::Matrix3d &H1,
					Eigen::Matrix3d &R0t, Eigen::Matrix &R1t,     
					double &f_new, Eigen::Vector2d &p0_new,
					Eigen::Vector2d &p1_new, double zoom)
{

	Eigen::Map<Eigen::Matrix3d> R0(r0_);
	Eigen::Map<Eigen::Vector3d> t0(t0_);
	Eigen::Map<Eigen::Matrix3d> R1(r1_);
	Eigen::Map<Eigen::Vector3d> t1(t1_);

	Eigen::Matrix4d T0 = Eigen::Matrix4d::Zero();
	Eigen::Matrix4d T1 = Eigen::Matrix4d::Zero();

	T0.block<3, 3>(0, 0) = R0;
	T0.block<3, 1>(0, 3) = t0;
	T0(3, 3) = 1.0;
	T1.block<3, 3>(0, 0) = R1;
	T1.block<3, 1>(0, 3) = t1;
	T1(3, 3) = 1.0;

	Eigen::Matrix4d T_rel = Eigen::Matrix4d::Zero();
	T_rel = T1 * T0.inverse();

	Eigen::Matrix3d Rt = T_rel.block<3, 3>(0, 0);
	Eigen::Matrix3d R = Rt.transpose();
	Eigen::Vector3d T = T_rel.block<3, 1>(0, 3);

	// Bring the 2 cameras in the same orientation by rotating them "minimally"
	Eigen::Vector3d om =
		Eigen::AngleAxisd(R).axis() * Eigen::AngleAxisd(R).angle();

	Eigen::Matrix3d r_1(Eigen::AngleAxisd(om.norm() / (-2.0), om.normalized()));

	if (om.norm() == 0) {
		r_1.setIdentity();
	}

	Eigen::Matrix3d r_0 = r_1.transpose();
	Eigen::Vector3d t_n = r_1 * T;

	// Rotate both cameras so as to bring the translation vector in alignment with
	// the (1;0;0) axis
	Eigen::Vector3d uu = Eigen::Vector3d::Zero();

	if (std::abs(t_n(0)) > std::abs(t_n(1))) {
		uu(0) = 1;  // Horizontal epipolar lines

	} else {
		uu(1) = 1;  // Vertical epipolar lines
	}

	if (uu.dot(t_n) < 0) {
		uu = -uu;
	}

	Eigen::Vector3d ww = t_n.cross(uu);

	ww = ww / ww.norm();
	ww = std::acos(std::abs(t_n.dot(uu)) / (t_n.norm() * uu.norm())) * ww;
	Eigen::Matrix3d R2(Eigen::AngleAxisd(ww.norm(), ww.normalized()));

	if (t_n.norm() == 0) {
		R2.setIdentity();
	}

	// Global rotations to be applied to both views
	Eigen::Matrix3d R_0 =  R2*r_0;
	Eigen::Matrix3d R_1 =  R2*r_1;

	// The resulting rigid motion between the two cameras after image rotations
	// (substitutes of om, R and T)
	Eigen::Matrix3d R_new;
	R_new.setIdentity();

	/////////// Zach's simplicity version /////////////////////////////////
	//// For simplicity, let's pick the same value for the horizontal focal
	//// length as the vertical focal length
	//// (resulting into square pixels)
	//double f0_new = std::round(f0_[0] + zoom);
	//double f1_new = std::round(f0_[0] + zoom);

	/////////// Fynn's version depending on the distortion model //////////
	double f_y_new;

	if(equi_model0_ || equi_model1_){
		f_y_new  = (f0_[0]+f0_[1]+f1_[0]+f1_[1])/4-90;//+zoom;
	}
	else{
		double f0_y_new;

		if (d0_[0] < 0)
			f0_y_new =
				f0_[1] * (1 + d0_[0] * (pow(image_width_, 2) + 
				pow(image_height_, 2)) / (4 * pow(f0_[1], 2)));
		else {
			f0_y_new = f0_[1];
		}

		double f1_y_new;

		if (d1_[0] < 0)
			f1_y_new =
				f1_[1] * (1 + d1_[0] * (pow(image_width_, 2) +
				pow(image_height_, 2)) / (4 * pow(f1_[1], 2)));
		else {
			f1_y_new = f1_[1];
		}

		f_y_new = std::min(f0_y_new, f1_y_new) +
			zoom;  // HACK(gohlp): 40 to zoom in, should be automatically
	}
	// For simplicity, let's pick the same value for the horizontal focal length
	// as the vertical focal length
	// (resulting into square pixels)
	double f0_new = std::round(f_y_new);
	double f1_new = std::round(f_y_new);

	

	// For simplicity, set the principal points for both cameras to be the average
	// of the two principal points
	double py_new = (p0_[1] + p1_[1]) / 2.0;
	p0_new(1) = py_new;
	p1_new(1) = py_new;
	double px_new = (p0_[0] + p1_[0]) / 2.0;
	p0_new(0) = px_new;
	p1_new(0) = px_new;

	// Original Camera matrices
	Eigen::Matrix3d C0 = Eigen::Matrix3d::Zero();
	Eigen::Matrix3d C1 = Eigen::Matrix3d::Zero();

	C0 << f0_[0], 0.0, p0_[0], 0.0, f0_[1], p0_[1], 0.0, 0.0, 1.0;
	C1 << f1_[0], 0.0, p1_[0], 0.0, f1_[1], p1_[1], 0.0, 0.0, 1.0;

	// Resulting new camera matrices
	Eigen::Matrix3d C0_new = Eigen::Matrix3d::Zero();
	Eigen::Matrix3d C1_new = Eigen::Matrix3d::Zero();

	C0_new << f0_new, 0.0, p0_new(0), 0.0, f0_new, p0_new(1), 0.0, 0.0, 1.0;
	C1_new << f1_new, 0.0, p1_new(0), 0.0, f1_new, p1_new(1), 0.0, 0.0, 1.0;

	// Compute homography
	H0 = C0 * R_0.transpose() * C0_new.inverse();
	H1 = C1 * R_1.transpose() * C1_new.inverse();

	R0t = R_0;
	R1t = R_1;

	f_new = f0_new;
}
