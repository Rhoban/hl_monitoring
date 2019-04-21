#pragma once

#include <hl_monitoring/drawers/drawer.h>

namespace hl_monitoring
{
class PositionDrawer : public Drawer<hl_communication::PositionDistribution>
{
public:
  PositionDrawer();
  ~PositionDrawer();
  void draw(FieldToImgConverter converter, const hl_communication::PositionDistribution& data, cv::Mat* out) override;
  Json::Value toJson() const override;
  void fromJson(const Json::Value& v) override;

  void setColor(const cv::Scalar& new_color);
private:
  /**
   * Radius of the circle used for the position [px]
   */
  double circle_radius;

  cv::Scalar color;
};

}  // namespace hl_monitoring
