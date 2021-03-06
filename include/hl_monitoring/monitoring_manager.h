#pragma once

#include <hl_monitoring/field.h>
#include <hl_monitoring/image_provider.h>
#include <hl_monitoring/team_manager.h>
#include <hl_communication/message_manager.h>

#include <json/json.h>
#include <memory>

namespace hl_monitoring
{
/**
 * Manage the monitoring of a game or a replay for the RoboCup humanoid league
 *
 * Provide access to the following elements:
 * - Status message sent by the robot
 * - Game Controller status
 * - Named video streams with parameters of the cameras (intrinsic+extrinsic)
 */
class MonitoringManager
{
public:
  MonitoringManager();
  ~MonitoringManager();

  /**
   * Starts with a default configuration in live mode:
   * - No image providers
   * - Message manager listening on GameController port with auto_discovery
   * - Default Field
   * - Empty team manager
   */
  void autoLiveStart();

  void loadConfig(const std::string& path);
  /**
   * Creates the appropriate output directory if live mode is enabled.
   */
  void setupOutput();
  void dumpReplayConfig();

  std::unique_ptr<ImageProvider> buildImageProvider(const Json::Value& v, const std::string& name);
  void loadImageProviders(const Json::Value& v);
  void loadMessageManager(const Json::Value& v);

  void setMessageManager(std::unique_ptr<hl_communication::MessageManager> message_manager);
  void addImageProvider(const std::string& name, std::unique_ptr<ImageProvider> image_provider);

  void update();

  CalibratedImage getCalibratedImage(const std::string& provider_name, uint64_t time_stamp);
  std::map<std::string, CalibratedImage> getCalibratedImages(uint64_t time_stamp);

  const hl_communication::MessageManager& getMessageManager() const;

  /**
   * Returns non-mutable access to the given image provider if it exists.
   * throws std::out_of_range if name is not valid.
   */
  const ImageProvider& getImageProvider(const std::string& name) const;

  std::set<std::string> getImageProvidersNames() const;

  /**
   * Return the first time_stamp found in messages and video streams
   */
  uint64_t getStart() const;

  bool isGood() const;

  bool isLive() const;

  /**
   * Set the offset in us between steady_clock and system_clock [us] (time_since_epoch)
   */
  void setOffset(int64 offset);

  /**
   * Get the offset in us between steady_clock and system_clock [us] (time_since_epoch)
   * - return 0 if there are no elements with offsets
   * - Might display an error message if an overflow occured
   */
  int64_t getOffset() const;

  const Field& getField() const;
  const TeamManager& getTeamManager() const;

  void setPose(const std::string& provider_name, int frame_idx, const hl_communication::Pose3D& pose);

private:
  /**
   * Access to message from both, robots and GameController
   */
  std::unique_ptr<hl_communication::MessageManager> message_manager;

  /**
   * Access to all the channels allowing to retrieve images
   */
  std::map<std::string, std::unique_ptr<ImageProvider>> image_providers;

  /**
   * Dimensions of the field
   */
  Field field;

  /**
   * Access to team names and field names
   */
  TeamManager team_manager;

  /**
   * Is the monitoring session live or not?
   */
  bool live;

  /**
   * The path of the directory to which datas will be exported (ending by a /).
   * if empty, then nothing is saved
   */
  std::string output_prefix;
};

}  // namespace hl_monitoring
