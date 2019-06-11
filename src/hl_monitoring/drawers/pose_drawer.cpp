#include <hl_monitoring/drawers/pose_drawer.h>

#include <hl_communication/utils.h>

#include <opencv2/imgproc.hpp>

using namespace hl_communication;

namespace hl_monitoring
{
PoseDrawer::PoseDrawer() : circle_radius(15.0), thickness(3.0), color(0, 0, 0)
{
}

PoseDrawer::~PoseDrawer()
{
}

void PoseDrawer::draw(FieldToImgConverter converter, const hl_communication::PoseDistribution& pose, cv::Mat* out)
{
  if (!pose.has_position())
  {
    return;
  }
  // Tagging pos
  const PositionDistribution& pos = pose.position();
  cv::Point3f field_pos(pos.x(), pos.y(), 0.0);
  cv::Point2f img_pos;
  bool valid_pos = converter(field_pos, &img_pos);
  if (valid_pos)
  {
    cv::circle(*out, img_pos, circle_radius, color, thickness, cv::LINE_AA);
  }
  // Tagging dir
  if (valid_pos && pose.has_dir())
  {
    double dir_rad = pose.dir().mean();
    double dist = 0.1;
    cv::Point3f field_arrow_end(pos.x() + cos(dir_rad) * dist, pos.y() + sin(dir_rad) * dist, 0.0);
    cv::Point2f img_end;
    bool valid_end = converter(field_arrow_end, &img_end);
    if (valid_end)
    {
      cv::Point2f img_dir = img_end - img_pos;
      float img_length = std::sqrt(img_dir.x * img_dir.x + img_dir.y * img_dir.y);
      cv::Point2f img_arrow_end = img_pos + circle_radius * img_dir / img_length;
      cv::line(*out, img_pos, img_arrow_end, color, thickness, cv::LINE_AA);
    }
  }
}

void PoseDrawer::setColor(const cv::Scalar& new_color)
{
  color = new_color;
}

void PoseDrawer::setRadius(double new_radius)
{
  circle_radius = new_radius;
}

Json::Value PoseDrawer::toJson() const
{
  Json::Value v = Drawer::toJson();
  v["circle_radius"] = circle_radius;
  v["thickness"] = thickness;
  v["color"] = val2Json(color);
  return v;
}

void PoseDrawer::fromJson(const Json::Value& v)
{
  tryReadVal(v, "circle_radius", &circle_radius);
  tryReadVal(v, "thickness", &thickness);
  tryReadVal(v, "color", &color);
}

}  // namespace hl_monitoring
