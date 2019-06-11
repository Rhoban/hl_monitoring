#include <hl_monitoring/drawers/arrow_drawer.h>

#include <hl_communication/utils.h>

#include <opencv2/imgproc.hpp>

#include <iostream>

using namespace hl_communication;

namespace hl_monitoring
{
ArrowDrawer::ArrowDrawer(ArrowType type, double arrow_thickness)
  : color(0, 0, 0), arrow_thickness(arrow_thickness), arrow_tip_length(20), type(type)
{
}

ArrowDrawer::~ArrowDrawer()
{
}

void ArrowDrawer::draw(FieldToImgConverter converter, const std::pair<cv::Point3f, cv::Point3f>& segment, cv::Mat* out)
{
  cv::Point2f img_src, img_end;
  bool valid_src = converter(segment.first, &img_src);
  bool valid_end = converter(segment.second, &img_end);
  // Tagging dir
  if (valid_src && valid_end)
  {
    double arrow_length = cv::norm(cv::Mat(img_end - img_src));
    double arrow_tip_ratio = arrow_tip_length / arrow_length;

    if (type == ArrowHead)
    {
      cv::arrowedLine(*out, img_src, img_end, color, arrow_thickness, cv::LINE_AA, 0, arrow_tip_ratio);
    }
    else if (type == ArrowCross)
    {
      cv::line(*out, img_src, img_end, color, arrow_thickness, cv::LINE_AA);
      cv::drawMarker(*out, img_end, color, cv::MARKER_TILTED_CROSS, 15, arrow_thickness, cv::LINE_AA);
    }
  }
}

void ArrowDrawer::setColor(const cv::Scalar& new_color)
{
  color = new_color;
}

Json::Value ArrowDrawer::toJson() const
{
  Json::Value v = Drawer::toJson();
  v["arrow_thickness"] = arrow_thickness;
  v["arrow_tip_length"] = arrow_tip_length;
  v["color"] = val2Json(color);
  return v;
}

void ArrowDrawer::fromJson(const Json::Value& v)
{
  tryReadVal(v, "arrow_thickness", &arrow_thickness);
  tryReadVal(v, "arrow_tip_length", &arrow_tip_length);
  tryReadVal(v, "color", &color);
}

}  // namespace hl_monitoring
