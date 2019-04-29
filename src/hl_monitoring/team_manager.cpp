#include <hl_monitoring/team_manager.h>

#include <hl_monitoring/utils.h>

#include <iostream>

namespace hl_monitoring
{
TeamManager::TeamManager()
{
}

bool TeamManager::hasTeam(uint32_t team_id) const
{
  return teams.count(team_id) > 0;
}

const TeamConfig& TeamManager::getTeam(uint32_t team_id) const
{
  return teams.at(team_id);
}

void TeamManager::fromJson(const Json::Value& v)
{
  teams = readMap<TeamConfig>(v, "teams");
}

Json::Value TeamManager::toJson() const
{
  Json::Value v;
  v["teams"] = map2Json(teams);
  return v;
}

}  // namespace hl_monitoring
