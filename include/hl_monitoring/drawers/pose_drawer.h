#pragma once

#include <hl_monitoring/drawers/drawer.h>

namespace hl_monitoring
{
class PoseDrawer : public Drawer<hl_communication::PoseDistribution>
{
public:
  PoseDrawer();
  ~PoseDrawer();
  void draw(FieldToImgConverter converter, const hl_communication::PoseDistribution& data, cv::Mat* out) override;
  void getEllipsePoint(FieldToImgConverter converter, cv::Point3f field_pos, int nbPoints, double angle_ellipse,
                       std::pair<float, float> axes, double start_angle, double end_angle,
                       std::vector<cv::Point>* ellipsePoints);
  Json::Value toJson() const override;
  void fromJson(const Json::Value& v) override;

  void setColor(const cv::Scalar& new_color);

  void setRadius(double new_radius);

private:
  /**
   * Radius of the circle used for the position, also used for direction indicator [px]
   */
  double circle_radius;

  /**
   * Thickness of the pose circle and the direction line [px]
   */
  double thickness;

  cv::Scalar color;
};

}  // namespace hl_monitoring
