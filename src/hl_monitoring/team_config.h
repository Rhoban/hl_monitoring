#pragma once

#include <hl_communication/robot_estimation.pb.h>
#include <json/json.h>

namespace hl_monitoring
{
/**
 * Stores the name of all the players of the teams
 *
 * Players are stored by id (1-...) rather than by index (0-...)
 */
class TeamConfig
{
public:
  TeamConfig();

  bool hasPlayer(uint32_t player_id) const;
  const std::string& getName() const;
  /**
   * Throws std::out_of_range if no players with the given player_id can be found
   */
  const std::string& getName(uint32_t player_id) const;

  void fromJson(const Json::Value& v);
  Json::Value toJson() const;

private:
  std::string name;
  std::map<uint32_t, std::string> players;
};

}  // namespace hl_monitoring
