#include <hl_monitoring/drawers/position_drawer.h>

#include <hl_communication/utils.h>
#include <hl_monitoring/utils.h>

#include <opencv2/imgproc.hpp>

using namespace hl_communication;

namespace hl_monitoring
{
PositionDrawer::PositionDrawer() : circle_radius(5.0), color(0,0,0)
{
}

PositionDrawer::~PositionDrawer()
{
}

void PositionDrawer::draw(const CameraMetaInformation& camera_information, const PositionDistribution& pos, cv::Mat* out)
{
  cv::Point3f field_pos(pos.x(), pos.y(), 0.0);
  cv::Point2f img_pos;
  bool valid = fieldToImg(field_pos,camera_information, &img_pos);
  if (valid)
  {
    cv::circle(*out, img_pos, circle_radius, color, cv::FILLED);
  }
}

void PositionDrawer::draw(const Field& f, const TopViewDrawer& top_view_drawer, const PositionDistribution& pos, cv::Mat* out)
{
  cv::Point img_pos = top_view_drawer.getImgFromField(f, cv::Point3f(pos.x(), pos.y(), 0.0));  
  cv::circle(*out, img_pos, circle_radius, color, cv::FILLED);
}

void PositionDrawer::setColor(const cv::Scalar& new_color)
{
  color = new_color;
}

Json::Value PositionDrawer::toJson() const
{
  Json::Value v = Drawer::toJson();
  v["circle_radius"] = circle_radius;
  v["color"] = hl_monitoring::toJson(color);
  return v;
}

void PositionDrawer::fromJson(const Json::Value& v)
{
  tryReadVal(v, "circle_radius", &circle_radius);
  tryReadVal(v, "color", &color);
}

}  // namespace hl_monitoring
