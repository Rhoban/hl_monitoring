#pragma once

#include <hl_monitoring/top_view_drawer.h>
#include <hl_monitoring/utils.h>
#include <hl_communication/wrapper.pb.h>

#include <opencv2/core.hpp>
#include <json/json.h>

#include <functional>

namespace hl_monitoring
{
class CameraMetaInformation;
class TopViewDrawer;

template <class T>
class Drawer
{
public:
  /**
   * The contract of a field to image converter is to convert a field_pos to an img_pos.
   * If the img_pos is outside of the image or if the point is behind the camera, 'false' is returned,
   * on success, 'true' is returned.
   */
  typedef std::function<bool(const cv::Point3f& field_pos, cv::Point2f* img_pos)> FieldToImgConverter;

  Drawer()
  {
  }
  virtual ~Drawer()
  {
  }

  /**
   * Draws the content of data on the 'out' image
   */
  virtual void draw(FieldToImgConverter, const T& data, cv::Mat* out) = 0;

  /**
   * Draws the content of data on the 'out' image, using 'camera_information' to project information in the image basis
   * Might be overrided if behavior is different between TopView and drawing on natural images
   */
  virtual void drawNatural(const CameraMetaInformation& camera_information, const T& data, cv::Mat* out)
  {
    FieldToImgConverter converter = [camera_information](const cv::Point3f& field_pos, cv::Point2f* img_pos) {
      return fieldToImg(field_pos, camera_information, img_pos);
    };
    draw(converter, data, out);
  }

  /**
   * Draws the content of  data on the 'out' image, based on the given top_view_drawer
   * Might be overrided if behavior is different between TopView and drawing on natural images
   */
  virtual void drawTopView(const Field& f, const TopViewDrawer& top_view_drawer, const T& data, cv::Mat* out)
  {
    FieldToImgConverter converter = [f, top_view_drawer](const cv::Point3f& field_pos, cv::Point2f* img_pos) {
      *img_pos = top_view_drawer.getImgFromField(f, field_pos);
      return true;
    };
    draw(converter, data, out);
  }

  /**
   * Convert the internal configuration of the drawer to json
   */
  virtual Json::Value toJson() const
  {
    return Json::Value();
  }

  /**
   * Load internal configuration of a drawer from json content. The function is expected to crash if data is not valid.
   */
  virtual void fromJson(const Json::Value& v)
  {
    (void)v;
  }
};

}  // namespace hl_monitoring
