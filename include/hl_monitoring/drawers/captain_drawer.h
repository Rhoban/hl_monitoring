#pragma once

#include <hl_monitoring/drawers/position_drawer.h>
#include <hl_monitoring/drawers/pose_drawer.h>
#include <hl_communication/message_manager.h>

namespace hl_monitoring
{

class CaptainDrawer : public Drawer<hl_communication::Captain>
{
public:
  CaptainDrawer();
  ~CaptainDrawer();
  void draw(FieldToImgConverter converter, const hl_communication::Captain& data, cv::Mat* out) override;
  Json::Value toJson() const override;
  void fromJson(const Json::Value& v) override;

private:
  PositionDrawer ball_drawer;
  PoseDrawer opponent_drawer;
};

}  // namespace hl_monitoring
