#pragma once

#include <hl_communication/labelling.pb.h>
#include <hl_communication/camera.pb.h>
#include <hl_monitoring/field.h>

#include <opencv2/core.hpp>

namespace hl_monitoring
{
/**
 * Uses human labelling on an image to retrieve the pose of the camera
 */
class ManualPoseSolver
{
public:
  ManualPoseSolver(const cv::Mat& img, const hl_communication::IntrinsicParameters& camera_parameters,
                   const Field& field);
  ~ManualPoseSolver();

  bool solve(cv::Mat* rvec, cv::Mat* tvec);
  bool solve(hl_communication::Pose3D* pose, std::vector<hl_communication::Match2D3DMsg>* matches = nullptr);

  static bool solvePose(const std::vector<cv::Point2f>& img_pos, const std::vector<cv::Point3f>& obj_pos,
                        const cv::Mat& camera_matrix, const cv::Mat& distortion_coefficients, cv::Mat* rvec,
                        cv::Mat* tvec);
  static bool solvePose(const std::vector<hl_communication::Match2D3DMsg>& matches,
                        const hl_communication::IntrinsicParameters& camera_parameters, hl_communication::Pose3D* pose);

private:
  void updatePose();
  void exportMatches(std::vector<hl_communication::Match2D3DMsg>* matches);
  void onClick(int event, int x, int y, void* param);

  /**
   * The model of the field used for calibration
   */
  Field field;

  cv::Mat calibration_img;

  /**
   * The name of the points used for calibration
   */
  std::vector<std::string> points_names;

  /**
   * The points of interest of the field
   */
  std::vector<cv::Point3f> points_in_world;

  /**
   * The points manually tagged in the image associated with the index of the point in
   * 'points_in_world' vector
   */
  std::map<int, cv::Point> points_in_img;

  /**
   * Index of the point being calibrated right now
   */
  int point_index;

  cv::Mat camera_matrix;
  cv::Mat distortion_coefficients;
  cv::Size img_size;

  /**
   * The position of field center in camera_basis
   */
  cv::Mat tvec;

  /**
   * The rotation transform from field basis to camera basis
   */
  cv::Mat rvec;
};

}  // namespace hl_monitoring
