#pragma once

#include <hl_monitoring/camera.pb.h>

#include <hl_communication/position.pb.h>
#include <hl_communication/labelling.pb.h>

#include <json/json.h>
#include <opencv2/core.hpp>
#include <Eigen/Geometry>

/**
 * Contains multiple conversion tools from hl_communication protobuf format to
 * OpenCV classical format and other utilities functions related to jsoncpp
 */

namespace hl_monitoring
{
/**
 * Uses system_clock to extract a formatted time: format is:
 * - YYYY_MM_DD_HHhMMmSSs Ex: 2018_09_25_17h23m12s
 * Function is reentrant
 */
std::string getFormattedTime();

void intrinsicToCV(const IntrinsicParameters& camera_parameters, cv::Mat* camera_matrix,
                   cv::Mat* distortion_coefficients, cv::Size* img_size);
void cvToIntrinsic(const cv::Mat& camera_matrix, const cv::Mat& distortion_coefficients, const cv::Size& img_size,
                   IntrinsicParameters* camera_parameters);
void pose3DToCV(const Pose3D& pose, cv::Mat* rvec, cv::Mat* tvec);
void cvToPose3D(const cv::Mat& rvec, const cv::Mat& tvec, Pose3D* pose);

/**
 * Convert a protobuf Pose3D to an Affine3D transform
 */
Eigen::Affine3d getAffineFromProtobuf(const hl_monitoring::Pose3D& pose);

/**
 * Export the given Affine3D transform to a pose
 */
void setProtobufFromAffine(const Eigen::Affine3d& affine, hl_monitoring::Pose3D* pose);

std::ostream& operator<<(std::ostream& out, const Pose3D& pose);

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

cv::Point2f protobufToCV(const hl_communication::Point2DMsg& msg);
cv::Point3f protobufToCV(const hl_communication::Point3DMsg& msg);
void cvToProtobuf(const cv::Point2f& pos, hl_communication::Point2DMsg* msg);
void cvToProtobuf(const cv::Point3f& pos, hl_communication::Point3DMsg* msg);

void protobufToCV(const std::vector<hl_communication::Match2D3DMsg>& matches, std::vector<cv::Point2f>* img_pos,
                  std::vector<cv::Point3f>* obj_pos);

/**
 * Convert from field position to an image position based on camera_information.
 * @return Is the img position valid (for points behind camera, the underlying implementation return image position of
 * the symetric point), if the point is outside of image, then return false as well
 */
bool fieldToImg(const cv::Point3f& pos_in_field, const CameraMetaInformation& camera_information, cv::Point2f* img_pos);

Json::Value file2Json(const std::string& path);

/**
 * If human is enabled, then indent the string and uses end of line
 */
std::string json2String(const Json::Value& v, bool human = true);

/**
 * @see json2String
 */
void writeJson(const Json::Value& v, const std::string& path, bool human = true);

void checkMember(const Json::Value& v, const std::string& key);

template <typename T>
Json::Value val2Json(const T& val)
{
  return val.toJson();
}

template <>
Json::Value val2Json<bool>(const bool& val);
template <>
Json::Value val2Json<int>(const int& val);
template <>
Json::Value val2Json<size_t>(const size_t& val);
template <>
Json::Value val2Json<float>(const float& val);
template <>
Json::Value val2Json<double>(const double& val);
template <>
Json::Value val2Json<std::string>(const std::string& val);
template <>
Json::Value val2Json<cv::Scalar>(const cv::Scalar& color);

/**
 * Place v[key] in 'dst'
 * Throws an error with an explicit message in case:
 * - v is not an object
 * - v[key] does not contain an object
 * - v[key] has not the required type
 */
template <typename T>
void readVal(const Json::Value& v, const std::string& key, T* dst)
{
  checkMember(v, key);
  dst->fromJson(v[key]);
}

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

template <typename T>
std::map<uint32_t, T> readMap(const Json::Value& v, const std::string& map_key)
{
  checkMember(v, map_key);
  if (!v[map_key].isObject())
  {
    throw std::runtime_error("readMap(): Value at '" + map_key + "' is not an object");
  }
  // Parse entries:
  const Json::Value& map_v = v[map_key];
  std::map<uint32_t, T> result;
  for (Json::ValueConstIterator it = map_v.begin(); it != map_v.end(); it++)
  {
    const std::string& key = it.name();
    readVal(map_v, key, &(result[std::stoi(key)]));
  }
  return result;
}

template <typename T>
void tryReadMap(const Json::Value& v, const std::string& key, std::map<uint32_t, T>* ptr)
{
  if (v.isObject() && v.isMember(key))
  {
    *ptr = readMap<T>(v, key);
  }
}

template <typename T>
Json::Value map2Json(const std::map<uint32_t, T>& values)
{
  Json::Value v(Json::objectValue);
  for (const auto& pair : values)
  {
    v[std::to_string(pair.first)] = val2Json(pair.second);
  }
  return v;
}

}  // namespace hl_monitoring
