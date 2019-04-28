#pragma once

#include <hl_monitoring/drawers/arrow_drawer.h>
#include <hl_monitoring/drawers/pose_drawer.h>
#include <hl_monitoring/drawers/text_drawer.h>

namespace hl_monitoring
{
class PlayerDrawer : public Drawer<hl_communication::RobotMsg>
{
public:
  PlayerDrawer();
  ~PlayerDrawer();
  void draw(FieldToImgConverter converter, const hl_communication::RobotMsg& data, cv::Mat* out) override;
  Json::Value toJson() const override;
  void fromJson(const Json::Value& v) override;

  void setColor(const cv::Scalar& new_color);
private:
  /**
   * Drawer used to tag robot position and direction
   */
  PoseDrawer pose_drawer;

  /**
   * Draws name of the robot
   */
  TextDrawer name_drawer;

  ArrowDrawer target_drawer;

  ArrowDrawer kick_drawer;

  cv::Scalar color;
};

}  // namespace hl_monitoring
