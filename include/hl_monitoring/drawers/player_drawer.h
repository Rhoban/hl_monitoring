#pragma once

#include <hl_monitoring/drawers/pose_drawer.h>

namespace hl_monitoring
{
class PlayerDrawer : public Drawer<hl_communication::RobotMsg>
{
public:
  PlayerDrawer();
  ~PlayerDrawer();
  void draw(const CameraMetaInformation& camera_information, const hl_communication::RobotMsg& data,
            cv::Mat* out) override;
  void draw(const Field& f, const TopViewDrawer& top_view_drawer, const hl_communication::RobotMsg& data,
            cv::Mat* out) override;
  Json::Value toJson() const override;
  void fromJson(const Json::Value& v) override;

  void setColor(const cv::Scalar& new_color);
private:
  /**
   * Drawer used to tag robot position and direction
   */
  PoseDrawer pose_drawer;

  cv::Scalar color;
};

}  // namespace hl_monitoring
