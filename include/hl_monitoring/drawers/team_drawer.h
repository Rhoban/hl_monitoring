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

  void setTeamFocus(int new_team_focus);
  /**
   * Player is numbered from 1
   */
  void setPlayerFocus(int new_player_focus);

private:
  /**
   * Drawer used to tag players
   */
  PlayerDrawer player_drawer;

  /**
   * Drawer used for captain information
   */
  CaptainDrawer captain_drawer;

  /**
   * If negative, no focus, if positive, only team with id 'team_focus' is drawn
   */
  int team_focus;

  /**
   * If negative, no focus, if positive, only player with id 'player_focus' is drawn
   */
  int player_focus;

  std::map<uint32_t,cv::Scalar> color_by_team_id;

  void updateColorsById(const hl_communication::GCMsg& gc_msg);
};

}  // namespace hl_monitoring
