#include <hl_monitoring/drawers/pose_drawer.h>

#include <hl_communication/utils.h>
#include <hl_monitoring/utils.h>

#include <opencv2/imgproc.hpp>

using namespace hl_communication;

namespace hl_monitoring
{
PoseDrawer::PoseDrawer() : circle_radius(4.0), arrow_length(16.0), arrow_thickness(2.0), arrow_tip_ratio(0.3), color(0, 0, 0)
{
}

PoseDrawer::~PoseDrawer()
{
}

void PoseDrawer::draw(const CameraMetaInformation& camera_information, const hl_communication::PoseDistribution& pose,
                      cv::Mat* out)
{
  if (!pose.has_position())
  {
    return;
  }
  // Tagging pos
  const PositionDistribution& pos = pose.position();
  cv::Point3f field_pos(pos.x(), pos.y(), 0.0);
  cv::Point2f img_pos;
  bool valid_pos = fieldToImg(field_pos, camera_information, &img_pos);
  if (valid_pos)
  {
    cv::circle(*out, img_pos, circle_radius, color, cv::FILLED);
  }
  // Tagging dir
  if (valid_pos && pose.has_dir())
  {
    double dir_rad = pose.dir().mean();
    double dist = 0.1;
    cv::Point3f field_arrow_end(pos.x() + cos(dir_rad) * dist, pos.y() + sin(dir_rad) * dist, 0.0);
    cv::Point2f img_end;
    bool valid_end = fieldToImg(field_arrow_end, camera_information, &img_end);
    if (valid_end)
    {
      cv::Point2f img_dir = img_end - img_pos;
      float img_length = std::sqrt(img_dir.x * img_dir.x + img_dir.y * img_dir.y);
      cv::Point2f img_arrow_end = img_pos + arrow_length * img_dir / img_length;
      cv::arrowedLine(*out, img_pos, img_arrow_end, color, arrow_thickness, 0, 0, arrow_tip_ratio);
    }
  }
}

void PoseDrawer::draw(const Field& f, const TopViewDrawer& top_view_drawer,
                      const hl_communication::PoseDistribution& pose, cv::Mat* out)
{
  if (!pose.has_position())
  {
    return;
  }
  // Tagging pos
  const PositionDistribution& pos = pose.position();
  cv::Point2f img_pos = top_view_drawer.getImgFromField(f, cv::Point3f(pos.x(), pos.y(), 0.0));
  cv::circle(*out, img_pos, circle_radius, color, cv::FILLED);
  // Tagging dir
  if (pose.has_dir())
  {
    double dir_rad = pose.dir().mean();
    double dist = 0.1;
    cv::Point3f field_arrow_end(pos.x() + cos(dir_rad) * dist, pos.y() + sin(dir_rad) * dist, 0.0);
    cv::Point2f img_end = top_view_drawer.getImgFromField(f, field_arrow_end);
    cv::Point2f img_dir = img_end - img_pos;
    float img_length = std::sqrt(img_dir.x * img_dir.x + img_dir.y * img_dir.y);
    cv::Point2f img_arrow_end = img_pos + arrow_length * img_dir / img_length;
    cv::arrowedLine(*out, img_pos, img_arrow_end, color, arrow_thickness, 0, 0, arrow_tip_ratio);
  }
}

void PoseDrawer::setColor(const cv::Scalar& new_color)
{
  color = new_color;
}

Json::Value PoseDrawer::toJson() const
{
  Json::Value v = Drawer::toJson();
  v["circle_radius"] = circle_radius;
  v["arrow_length"] = arrow_length;
  v["arrow_thickness"] = arrow_thickness;
  v["arrow_tip_ratio"] = arrow_tip_ratio;
  v["color"] = hl_monitoring::toJson(color);
  return v;
}

void PoseDrawer::fromJson(const Json::Value& v)
{
  tryReadVal(v, "circle_radius", &circle_radius);
  tryReadVal(v, "arrow_length", &arrow_length);
  tryReadVal(v, "arrow_thickness", &arrow_thickness);
  tryReadVal(v, "arrow_tip_ratio", &arrow_tip_ratio);
  tryReadVal(v, "color", &color);
}

}  // namespace hl_monitoring
