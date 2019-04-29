#include <hl_monitoring/team_config.h>

#include <hl_monitoring/utils.h>

namespace hl_monitoring
{
TeamConfig::TeamConfig()
{
}

bool TeamConfig::hasPlayer(uint32_t player_id) const
{
  return players.count(player_id) > 0;
}

const std::string& TeamConfig::getName() const
{
  return name;
}

const std::string& TeamConfig::getName(uint32_t player_id) const
{
  return players.at(player_id);
}

void TeamConfig::fromJson(const Json::Value& v)
{
  readVal(v, "name", &name);
  tryReadMap(v, "players", &players);
}

Json::Value TeamConfig::toJson() const
{
  Json::Value v;
  v["name"] = name;
  v["players"] = map2Json(players);
  return v;
}

}  // namespace hl_monitoring
