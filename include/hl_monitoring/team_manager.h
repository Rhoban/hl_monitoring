#pragma once

#include <hl_monitoring/team_config.h>

namespace hl_monitoring
{
/**
 * Stores the configuration of all the teams
 */
class TeamManager
{
public:
  TeamManager();

  bool hasTeam(uint32_t team_id) const;
  const TeamConfig& getTeam(uint32_t team_id) const;

  void fromJson(const Json::Value& v);
  Json::Value toJson() const;

private:
  std::map<uint32_t, TeamConfig> teams;
};

}  // namespace hl_monitoring
