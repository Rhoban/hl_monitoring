#include <hl_monitoring/team_manager.h>

#include <hl_communication/utils.h>

using namespace hl_communication;

namespace hl_monitoring
{
TeamManager::TeamManager() : nb_players_per_team(0)
{
}

int TeamManager::getNbPlayersPerTeam() const
{
  return nb_players_per_team;
}

bool TeamManager::hasTeam(uint32_t team_id) const
{
  return teams.count(team_id) > 0;
}

const TeamConfig& TeamManager::getTeam(uint32_t team_id) const
{
  return teams.at(team_id);
}

std::string TeamManager::getTeamName(uint32_t team_id) const
{
  if (hasTeam(team_id))
  {
    return getTeam(team_id).getName();
  }
  else
  {
    return "Team " + std::to_string(team_id);
  }
}

std::string TeamManager::getPlayerName(uint32_t team_id, uint32_t player_id) const
{
  std::string result;
  if (hasTeam(team_id) && getTeam(team_id).hasPlayer(player_id))
  {
    result = std::to_string(player_id) + " - " + getTeam(team_id).getName(player_id);
  }
  else
  {
    result = "Player " + std::to_string(player_id);
  }
  return result;
}

void TeamManager::fromJson(const Json::Value& v)
{
  readVal(v, "players", &nb_players_per_team);
  teams = readMap<TeamConfig>(v, "teams");
}

Json::Value TeamManager::toJson() const
{
  Json::Value v;
  v["players"] = nb_players_per_team;
  v["teams"] = map2Json(teams);
  return v;
}

}  // namespace hl_monitoring
