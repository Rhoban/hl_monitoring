#include <hl_monitoring/drawers/team_drawer.h>

#include <hl_communication/utils.h>
#include <hl_monitoring/utils.h>

#include <opencv2/imgproc.hpp>

using namespace hl_communication;

namespace hl_monitoring
{
TeamDrawer::TeamDrawer()
{
}

TeamDrawer::~TeamDrawer()
{
}

void TeamDrawer::draw(FieldToImgConverter converter, const hl_communication::MessageManager::Status& status,
                      cv::Mat* out)
{
  updateColorsById(status.gc_message);
  for (const auto& entry : status.getRobotsByTeam())
  {
    uint32_t team_id = entry.first;
    if (team_focus >= 0 && (int)team_id != team_focus)
    {
      continue;
    }
    cv::Scalar color(0,0,0);// Default color for team is black
    if (color_by_team_id.count(team_id))
    {
      color = color_by_team_id[team_id];
    }
    player_drawer.setColor(color);
    captain_drawer.setColor(color);
    for (const RobotMsg& msg : entry.second)
    {
      uint32_t robot_id = msg.robot_id().robot_id();
      if (player_focus >= 0 && (int)robot_id != player_focus)
      {
        continue;
      }
      if (!isPenalized(status.gc_message, team_id, robot_id))
      {
        player_drawer.draw(converter, msg, out);
        if (msg.has_captain())
        {
          captain_drawer.draw(converter, msg.captain(), out);
        }
      }
    }
  }
}

Json::Value TeamDrawer::toJson() const
{
  Json::Value v = Drawer::toJson();
  v["player"] = player_drawer.toJson();
  v["captain"] = captain_drawer.toJson();
  return v;
}

void TeamDrawer::fromJson(const Json::Value& v)
{
  Drawer::fromJson(v);
  if (v.isMember("player"))
  {
    player_drawer.fromJson(v["player"]);
  }
  if (v.isMember("captain"))
  {
    captain_drawer.fromJson(v["captain"]);
  }
}

void TeamDrawer::updateColorsById(const GCMsg& gc_msg)
{
  color_by_team_id.clear();
  std::vector<cv::Scalar> team_colors = { cv::Scalar(240, 0, 0), cv::Scalar(0, 0, 200) };
  for (int idx = 0; idx < gc_msg.teams_size(); idx++)
  {
    const GCTeamMsg& team_msg = gc_msg.teams(idx);
    if (team_msg.has_team_number() && team_msg.has_team_color())
    {
      uint32_t team_number = team_msg.team_number();
      uint32_t team_color = team_msg.team_color();
      color_by_team_id[team_number] = team_colors[team_color];
    }
  }
}

void TeamDrawer::setTeamFocus(int new_team_focus)
{
  team_focus = new_team_focus;
}

void TeamDrawer::setPlayerFocus(int new_player_focus)
{
  player_focus = new_player_focus;
}

}  // namespace hl_monitoring
