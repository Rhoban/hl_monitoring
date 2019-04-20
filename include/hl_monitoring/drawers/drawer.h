#pragma once

#include <hl_monitoring/top_view_drawer.h>
#include <hl_communication/wrapper.pb.h>

#include <opencv2/core.hpp>
#include <json/json.h>

namespace hl_monitoring
{
class CameraMetaInformation;
class TopViewDrawer;

template <class T>
class Drawer
{
public:
  Drawer()
  {
  }
  virtual ~Drawer()
  {
  }

  /**
   * Draws the content of data on the 'out' image, using 'camera_information' to project information in the image basis
   */
  virtual void draw(const CameraMetaInformation& camera_information, const T& data, cv::Mat* out) = 0;
  /**
   * Draws the content of  data on the 'out' image, based on the given top_view_drawer
   */
  virtual void draw(const Field& f, const TopViewDrawer& top_view_drawer, const T& data, cv::Mat* out) = 0;

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
  virtual void fromJson(const Json::Value& v){
    (void) v;
  }
};

}  // namespace hl_monitoring
