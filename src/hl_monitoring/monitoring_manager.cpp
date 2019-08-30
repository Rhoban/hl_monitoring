#include "hl_monitoring/monitoring_manager.h"

#include <hl_communication/utils.h>
#include <hl_communication/game_controller_utils.h>
#include <hl_monitoring/opencv_image_provider.h>
#include <hl_monitoring/replay_image_provider.h>

#include <fstream>
#include <iostream>

#include <sys/stat.h>
#include <sys/types.h>

#ifdef HL_MONITORING_USES_FLYCAPTURE
#include <hl_monitoring/flycap_image_provider.h>
#endif

using namespace hl_communication;

namespace hl_monitoring
{
MonitoringManager::MonitoringManager() : live(false)
{
}

MonitoringManager::~MonitoringManager()
{
  if (output_prefix != "")
  {
    message_manager->saveMessages(output_prefix + "messages.bin");
  }
}

void MonitoringManager::autoLiveStart()
{
  live = true;
  external_providers.clear();
  message_manager.reset(new MessageManager(getGCDefaultPort(), true));
  field = Field();
  team_manager = TeamManager();
  setupOutput();
  dumpReplayConfig();
}

void MonitoringManager::loadConfig(const std::string& path)
{
  // Reading Json file
  Json::Value root = file2Json(path);
  // Parsing json content
  readVal(root, "live", &live);
  tryReadVal(root, "output_prefix", &output_prefix);
  setupOutput();
  checkMember(root, "image_providers");
  checkMember(root, "message_manager");
  checkMember(root, "field");
  checkMember(root, "team_manager");
  field.fromJson(root["field"]);
  team_manager.fromJson(root["team_manager"]);
  loadImageProviders(root["image_providers"]);
  loadMessageManager(root["message_manager"]);
  dumpReplayConfig();
}

void MonitoringManager::setupOutput()
{
  if (live)
  {
    // If user forgot to add a '/' at the end of the file, add it automatically
    if (output_prefix != "" && output_prefix[output_prefix.size() - 1] != '/')
    {
      output_prefix += "/";
    }
    output_prefix += getFormattedTime() + "/";

    if (mkdir(output_prefix.c_str(), 0755))
    {
      throw std::runtime_error(HL_DEBUG + "Failed to create directory at '" + output_prefix + "'");
    }
  }
}

void MonitoringManager::dumpReplayConfig()
{
  // Only dump if config is live
  if (!live)
    return;

  // TODO: need update for image providers
  //  Json::Value v;
  //  v["live"] = false;
  //  v["image_providers"] = Json::Value(Json::ValueType::objectValue);
  //  for (const auto& entry : image_providers)
  //  {
  //    v["image_providers"][entry.first]["class_name"] = "ReplayImageProvider";
  //    v["image_providers"][entry.first]["input_path"] = entry.first + ".avi";
  //    v["image_providers"][entry.first]["meta_information_path"] = entry.first + ".bin";
  //  }
  //  v["message_manager"]["file_path"] = "messages.bin";
  //  v["field"] = field.toJson();
  //  v["team_manager"] = team_manager.toJson();
  //  writeJson(v, output_prefix + "replay.json", true);
}

std::unique_ptr<ImageProvider> MonitoringManager::buildImageProvider(const Json::Value& v, const std::string& name)
{
  checkMember(v, "class_name");
  std::unique_ptr<ImageProvider> result;
  std::string class_name, input_path, intrinsic_path, default_pose_path;
  readVal(v, "class_name", &class_name);
  tryReadVal(v, "intrinsic_path", &intrinsic_path);
  tryReadVal(v, "default_pose_path", &default_pose_path);
  std::string image_provider_prefix = output_prefix + name;
  if (class_name == "OpenCVImageProvider")
  {
    checkMember(v, "input_path");
    readVal(v, "input_path", &input_path);
    result.reset(new OpenCVImageProvider(input_path, image_provider_prefix));
  }
  else if (class_name == "ReplayImageProvider")
  {
    checkMember(v, "input_path");
    readVal(v, "input_path", &input_path);
    std::string meta_information_path;
    if (v.isMember("meta_information_path"))
    {
      result.reset(new ReplayImageProvider(input_path, v["meta_information_path"].asString()));
    }
    else
    {
      result.reset(new ReplayImageProvider(input_path));
    }
  }
#ifdef HL_MONITORING_USES_FLYCAPTURE
  else if (class_name == "FlyCapImageProvider")
  {
    checkMember(v, "parameters");
    const Json::Value& parameters = v["parameters"];
    result.reset(new FlyCapImageProvider(parameters, image_provider_prefix));
  }
#endif
  else
  {
    throw std::runtime_error(HL_DEBUG + "unknown class: '" + class_name + "'");
  }
  if (intrinsic_path != "")
  {
    IntrinsicParameters intrinsic_params;
    readFromFile(intrinsic_path, &intrinsic_params);
    result->setIntrinsic(intrinsic_params);
  }
  if (default_pose_path != "")
  {
    Pose3D pose;
    readFromFile(default_pose_path, &pose);
    result->setDefaultPose(pose);
  }
  return result;
}

void MonitoringManager::loadImageProviders(const Json::Value& v)
{
  if (!v.isObject())
  {
    throw std::runtime_error(HL_DEBUG + " invalid type for v, expecting an object");
  }
  for (Json::ValueConstIterator it = v.begin(); it != v.end(); it++)
  {
    const std::string& key = it.name();
    addImageProvider(key, buildImageProvider(v[key], key));
  }
}

void MonitoringManager::loadMessageManager(const Json::Value& v)
{
  if (!v.isObject())
  {
    throw std::runtime_error(HL_DEBUG + " invalid type for v, expecting an object");
  }
  std::string file_path;
  std::vector<int> ports;
  tryReadVal(v, "file_path", &file_path);
  if (v.isMember("ports"))
  {
    if (!v["ports"].isArray())
    {
      throw std::runtime_error(HL_DEBUG + " expecting an array of int for ports");
    }
    for (Json::ArrayIndex idx = 0; idx < v["ports"].size(); idx++)
    {
      if (!v["ports"][idx].isInt())
      {
        throw std::runtime_error(HL_DEBUG + " value at index " + std::to_string(idx) + " of ports is not an int");
      }
      ports.push_back(v["ports"][idx].asInt());
    }
  }
  bool ports_set = ports.size() != 0;
  bool file_path_set = file_path != "";
  if (!ports_set && !file_path_set)
  {
    throw std::runtime_error(HL_DEBUG + " neither 'ports' nor 'file_path' provided");
  }
  else if (ports_set && file_path_set)
  {
    throw std::runtime_error(HL_DEBUG + " both 'ports' and 'file_path' provided");
  }
  if (ports_set)
  {
    message_manager.reset(new MessageManager(ports));
  }
  else
  {
    message_manager.reset(new MessageManager(file_path));
  }
}

void MonitoringManager::setMessageManager(std::unique_ptr<MessageManager> new_message_manager)
{
  message_manager = std::move(new_message_manager);
}

void MonitoringManager::addImageProvider(const std::string& name, std::unique_ptr<ImageProvider> image_provider)
{
  uint64_t ip_start = image_provider->getStart();
  if (external_providers.count(name) > 0 && external_providers.at(name).count(ip_start))
    throw std::logic_error(HL_DEBUG + "Failed to add External Image Provider: '" + name + "' starting at " +
                           std::to_string(ip_start) + " is already in collection");
  if (live)
    image_provider->setExternalName(name);
  external_providers[name][ip_start] = std::move(image_provider);
}

void MonitoringManager::update()
{
  for (const auto& entry : external_providers)
  {
    for (const auto& provider_entry : entry.second)
    {
      provider_entry.second->update();
    }
  }
  for (const auto& entry : robot_providers)
  {
    for (const auto& provider_entry : entry.second)
    {
      provider_entry.second->update();
    }
  }
  message_manager->update();
}

// TODO: getCalibratedImage const should be available for image providers, then code would get a lot simpler (can use
// getImageProvider)
CalibratedImage MonitoringManager::getCalibratedImage(const std::string& provider_name, uint64_t time_stamp)
{
  if (external_providers.count(provider_name) == 0)
    throw std::out_of_range(HL_DEBUG + " no image provider named '" + provider_name + "'");
  if (external_providers.at(provider_name).begin()->second->getStart() > time_stamp)
    return external_providers.at(provider_name).begin()->second->getCalibratedImage(time_stamp);
  if (external_providers.at(provider_name).rbegin()->second->getStart() <= time_stamp)
    return external_providers.at(provider_name).rbegin()->second->getCalibratedImage(time_stamp);
  auto it = external_providers.at(provider_name).upper_bound(time_stamp);
  it--;
  return it->second->getCalibratedImage(time_stamp);
}

std::map<std::string, CalibratedImage> MonitoringManager::getCalibratedImages(uint64_t time_stamp)
{
  std::map<std::string, CalibratedImage> images;
  for (const std::string& provider_name : getImageProvidersNames())
  {
    try
    {
      images[provider_name] = getCalibratedImage(provider_name, time_stamp);
    }
    catch (const std::out_of_range& exc)
    {
      std::cerr << "Can't get image: " << exc.what() << std::endl;
    }
  }
  return images;
}

const hl_communication::MessageManager& MonitoringManager::getMessageManager() const
{
  if (!message_manager)
  {
    throw std::runtime_error(HL_DEBUG + " no message manager in MonitoringManager");
  }
  return *message_manager;
}

const ImageProvider& MonitoringManager::getImageProvider(const std::string& name) const
{
  if (external_providers.count(name) == 0)
    throw std::out_of_range(HL_DEBUG + " no image provider named '" + name + "'");
  if (external_providers.at(name).size() > 0)
    throw std::out_of_range(HL_DEBUG + " multiple image providers named '" + name +
                            "' timestamp should also be provided");
  return *(external_providers.at(name).begin()->second);
}

const ImageProvider& MonitoringManager::getImageProvider(const std::string& name, uint64_t time_stamp) const
{
  if (external_providers.count(name) == 0)
    throw std::out_of_range(HL_DEBUG + " no image provider named '" + name + "'");
  if (external_providers.at(name).begin()->second->getStart() > time_stamp)
    return *(external_providers.at(name).begin()->second);
  if (external_providers.at(name).rbegin()->second->getStart() <= time_stamp)
    return *(external_providers.at(name).rbegin()->second);
  auto it = external_providers.at(name).upper_bound(time_stamp);
  it--;
  return *(it->second);
}

std::set<std::string> MonitoringManager::getImageProvidersNames() const
{
  std::set<std::string> names;
  for (const auto& entry : external_providers)
  {
    names.insert(entry.first);
  }
  return names;
}

uint64_t MonitoringManager::getStart() const
{
  uint64_t min_ts = std::numeric_limits<uint64_t>::max();
  if (message_manager)
    min_ts = std::min(min_ts, message_manager->getStart());
  for (const auto& entry : external_providers)
  {
    min_ts = std::min(min_ts, entry.second.begin()->second->getStart());
  }
  return min_ts;
}

uint64_t MonitoringManager::getEnd() const
{
  uint64_t max_ts = 0;
  if (message_manager)
    max_ts = std::max(max_ts, message_manager->getEnd());
  for (const auto& entry : external_providers)
  {
    max_ts = std::max(max_ts, entry.second.rbegin()->second->getEnd());
  }
  return max_ts;
}

bool MonitoringManager::isGood() const
{
  for (const auto& entry : external_providers)
  {
    for (const auto& provider_entry : entry.second)
      if (provider_entry.second->isStreamFinished())
        return false;
  }
  return true;
}

bool MonitoringManager::isLive() const
{
  return live;
}

void MonitoringManager::setOffset(int64 offset)
{
  if (message_manager)
  {
    message_manager->setOffset(offset);
  }
  for (auto& entry : external_providers)
  {
    for (auto& ip : entry.second)
      ip.second->setOffset(offset);
  }
}

int64 MonitoringManager::getOffset() const
{
  std::vector<int64_t> offsets;
  if (message_manager)
  {
    offsets.push_back(message_manager->getOffset());
  }
  for (const auto& entry : external_providers)
    for (const auto& ip : entry.second)
      offsets.push_back(ip.second->getOffset());
  if (offsets.size() == 0)
  {
    return 0;
  }
  int64_t sum_offset = 0;
  int nb_offsets = 0;
  for (int64_t offset : offsets)
  {
    sum_offset += offset;
  }
  // TODO: add a mechanism to watch potential overflows on sum_offset;
  int64_t mean_offset = sum_offset / nb_offsets;
  return mean_offset;
}

const Field& MonitoringManager::getField() const
{
  return field;
}

const TeamManager& MonitoringManager::getTeamManager() const
{
  return team_manager;
}

void MonitoringManager::setPose(const std::string& provider_name, int frame_idx, const Pose3D& pose)
{
  if (external_providers.count(provider_name) == 0 || external_providers.at(provider_name).size() > 1)
    throw std::out_of_range(HL_DEBUG + " provider name: " + provider_name);
  external_providers.at(provider_name).begin()->second->setPose(frame_idx, pose);
}

}  // namespace hl_monitoring
