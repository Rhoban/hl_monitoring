#include <hl_monitoring/drawers/position_drawer.h>

#include <hl_communication/utils.h>
#include <hl_monitoring/drawers/geometry.h>

#include <opencv2/imgproc.hpp>

using namespace hl_communication;

namespace hl_monitoring
{
PositionDrawer::PositionDrawer() : circle_radius(0.2), color(0, 0, 0)
{
}

PositionDrawer::~PositionDrawer()
{
}

void PositionDrawer::draw(FieldToImgConverter converter, const PositionDistribution& pos, cv::Mat* out)
{
  drawGroundDisk(out, converter, cv::Point2f(pos.x(), pos.y()), circle_radius, color);
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
