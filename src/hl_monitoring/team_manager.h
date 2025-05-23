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

  int getNbPlayersPerTeam() const;

  bool hasTeam(uint32_t team_id) const;
  const TeamConfig& getTeam(uint32_t team_id) const;
  /**
   * If team is present in config, retrieve its name, otherwise generate a string 'Team XX'
   */
  std::string getTeamName(uint32_t team_id) const;
  /**
   * If given player can be found, returns 'X - name', otherwise returns 'Player X'
   */
  std::string getPlayerName(uint32_t team_id, uint32_t player_id) const;

  void fromJson(const Json::Value& v);
  Json::Value toJson() const;

private:
  std::map<uint32_t, TeamConfig> teams;

  int nb_players_per_team;
};

}  // namespace hl_monitoring
