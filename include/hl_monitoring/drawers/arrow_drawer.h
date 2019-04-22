#pragma once

#include <hl_monitoring/drawers/drawer.h>

namespace hl_monitoring
{

/**
 * Draws an arrow between from the first to the second point of the pair provided
 */
class ArrowDrawer : public Drawer<std::pair<cv::Point3f,cv::Point3f>>
{
public:
  ArrowDrawer();
  ~ArrowDrawer();
  void draw(FieldToImgConverter converter, const std::pair<cv::Point3f,cv::Point3f>& data, cv::Mat* out) override;
  Json::Value toJson() const override;
  void fromJson(const Json::Value& v) override;

  void setColor(const cv::Scalar& new_color);
private:
  /**
   *
   */
  cv::Scalar color;

  /**
   * Arrow width [px]
   */
  double arrow_thickness;

  /**
   * Length of the tip of the arrow drawn [px]
   */
  double arrow_tip_length;
};

}  // namespace hl_monitoring
