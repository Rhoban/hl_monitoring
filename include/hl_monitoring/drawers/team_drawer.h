#pragma once

#include <hl_monitoring/drawers/player_drawer.h>
#include <hl_monitoring/drawers/captain_drawer.h>
#include <hl_communication/message_manager.h>

namespace hl_monitoring
{
class TeamDrawer : public Drawer<hl_communication::MessageManager::Status>
{
public:
  TeamDrawer();
  ~TeamDrawer();
  void draw(FieldToImgConverter converter, const hl_communication::MessageManager::Status& data, cv::Mat* out) override;
  Json::Value toJson() const override;
  void fromJson(const Json::Value& v) override;

private:
  /**
   * Drawer used to tag players
   */
  PlayerDrawer player_drawer;

  /**
   * Drawer used for captain information
   */
  CaptainDrawer captain_drawer;

  std::map<uint32_t,cv::Scalar> color_by_team_id;

  void updateColorsById(const hl_communication::GCMsg& gc_msg);
};

}  // namespace hl_monitoring
