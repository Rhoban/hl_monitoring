#include <hl_monitoring/drawers/position_drawer.h>

#include <hl_communication/utils.h>

#include <opencv2/imgproc.hpp>

using namespace hl_communication;

namespace hl_monitoring
{
PositionDrawer::PositionDrawer() : circle_radius(5.0), color(0, 0, 0)
{
}

PositionDrawer::~PositionDrawer()
{
}

void PositionDrawer::draw(FieldToImgConverter converter, const PositionDistribution& pos, cv::Mat* out)
{
  cv::Point3f field_pos(pos.x(), pos.y(), 0.0);
  cv::Point2f img_pos;
  bool valid = converter(field_pos, &img_pos);
  if (valid)
  {
    cv::circle(*out, img_pos, circle_radius, color, cv::FILLED, cv::LINE_AA);
  }
}

void PositionDrawer::setColor(const cv::Scalar& new_color)
{
  color = new_color;
}

void PositionDrawer::setRadius(double new_radius)
{
  circle_radius = new_radius;
}

Json::Value PositionDrawer::toJson() const
{
  Json::Value v = Drawer::toJson();
  v["circle_radius"] = circle_radius;
  v["color"] = val2Json(color);
  return v;
}

void PositionDrawer::fromJson(const Json::Value& v)
{
  tryReadVal(v, "circle_radius", &circle_radius);
  tryReadVal(v, "color", &color);
}

}  // namespace hl_monitoring
