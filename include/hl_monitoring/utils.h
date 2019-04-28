#pragma once

#include <hl_monitoring/camera.pb.h>

#include <hl_communication/position.pb.h>

#include <json/json.h>
#include <opencv2/core.hpp>

/**
 * Contains multiple conversion tools from hl_communication protobuf format to
 * OpenCV classical format and other utilities functions related to jsoncpp
 */

namespace hl_monitoring
{
void intrinsicToCV(const IntrinsicParameters& camera_parameters, cv::Mat* camera_matrix,
                   cv::Mat* distortion_coefficients, cv::Size* img_size);
void cvToIntrinsic(const cv::Mat& camera_matrix, const cv::Mat& distortion_coefficients, const cv::Size& img_size,
                   IntrinsicParameters* camera_parameters);
void pose3DToCV(const Pose3D& pose, cv::Mat* rvec, cv::Mat* tvec);
void cvToPose3D(const cv::Mat& rvec, const cv::Mat& tvec, Pose3D* pose);

/**
 * Build a cv::Point3f at the mean of the position distribution. Z coordinate is set to 0.
 */
cv::Point3f cvtToPoint3f(const hl_communication::PositionDistribution& position);

/**
 * Extract the field position of an object position in the basis of a robot. Do not take into account uncertainty.
 */
cv::Point3f fieldFromSelf(const hl_communication::PositionDistribution& obj_pos_in_self,
                          const hl_communication::PoseDistribution& robot_pose);

cv::Size getImgSize(const CameraMetaInformation& camera_information);

/**
 * Convert a point from the field basis to the camera basis, should be used to check if points are facing the camera
 */
cv::Point3f fieldToCamera(const cv::Point3f& pos_in_field, const cv::Mat& rvec, const cv::Mat& tvec);

cv::Point2f fieldToImg(const cv::Point3f& pos_in_field, const CameraMetaInformation& camera_information);

/**
 * Convert from field position to an image position based on camera_information.
 * @return Is the img position valid (for points behind camera, the underlying implementation return image position of
 * the symetric point), if the point is outside of image, then return false as well
 */
bool fieldToImg(const cv::Point3f& pos_in_field, const CameraMetaInformation& camera_information, cv::Point2f* img_pos);

void checkMember(const Json::Value& v, const std::string& key);

/**
 * Place v[key] in 'dst'
 * Throws an error with an explicit message in case:
 * - v is not an object
 * - v[key] does not contain an object
 * - v[key] has not the required type
 */
template <typename T>
void readVal(const Json::Value& v, const std::string& key, T* dst) = delete;

template <>
void readVal<bool>(const Json::Value& v, const std::string& key, bool* dst);

template <>
void readVal<int>(const Json::Value& v, const std::string& key, int* dst);

template <>
void readVal<float>(const Json::Value& v, const std::string& key, float* dst);

template <>
void readVal<double>(const Json::Value& v, const std::string& key, double* dst);

template <>
void readVal<std::string>(const Json::Value& v, const std::string& key, std::string* dst);

/**
 * Also check that all values of the scalar are in [0,255]
 */
template <>
void readVal<cv::Scalar>(const Json::Value& v, const std::string& key, cv::Scalar* dst);

Json::Value toJson(const cv::Scalar& color);


/**
 * Place v[key] in 'dst', if v is not an object or if v does not contain key, do
 * not change dst and returns
 *
 * Throws an error with an explicit message in case:
 * - v[key] has not the required type
 */
template <typename T>
void tryReadVal(const Json::Value& v, const std::string& key, T* dst)
{
  if (!v.isObject() || !v.isMember(key))
  {
    return;
  }
  readVal(v, key, dst);
}

}  // namespace hl_monitoring
