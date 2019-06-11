#pragma once

#include "hl_communication/camera.pb.h"

#include <opencv2/core.hpp>

namespace hl_monitoring
{
/**
 * Represents an image along with its intrinsic and extrinsic parameters
 */
class CalibratedImage
{
public:
  CalibratedImage();
  CalibratedImage(const cv::Mat& img, const hl_communication::Pose3D& pose,
                  const hl_communication::IntrinsicParameters& camera_parameters);
  CalibratedImage(const cv::Mat& img, const hl_communication::CameraMetaInformation& camera_meta);

  const cv::Mat& getImg() const;

  const hl_communication::CameraMetaInformation& getCameraInformation() const;

  bool hasCameraParameters() const;
  bool hasPose() const;

  void exportCameraParameters(cv::Mat* camera_matrix, cv::Mat* distortion_coefficients, cv::Size* size) const;
  void exportPose(cv::Mat* rvec, cv::Mat* tvec) const;

  /**
   * Return true if both pose and camera_parameters are specified
   */
  bool isFullySpecified() const;

private:
  cv::Mat img;

  hl_communication::CameraMetaInformation camera_meta;
};

}  // namespace hl_monitoring
