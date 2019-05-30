#pragma once

#include <hl_monitoring/field.h>
#include <hl_monitoring/replay_image_provider.h>

#include <functional>

namespace hl_monitoring
{
class ReplayViewer
{
public:
  ReplayViewer(std::unique_ptr<ReplayImageProvider> image_provider, const std::string& window_name,
               bool playing = false, const Field& field = Field());
  virtual ~ReplayViewer();

  void run();

  /**
   * Update timestamp and retrieve image + calibration information
   */
  virtual void step();

  /**
   * Draw elements on the display Img, default is drawing nothing
   */
  virtual void paintImg();

protected:
  void updateTime();
  /**
   * Throws a logic_error if the key is already included
   */
  void addBinding(int key, const std::string& help_msg, std::function<void()> callback);
  void quit();
  void setSpeed(double speed);
  void printHelp();

  struct Action
  {
    std::function<void()> callback;
    std::string help_msg;
  };

  /**
   * The source of the images
   */
  std::unique_ptr<ReplayImageProvider> provider;

  /**
   * The model of the field used for the replay
   */
  Field field;

  std::map<int, Action> key_bindings;

  /**
   * The name of the cv window used for display
   */
  std::string window_name;

  /**
   * The image of the provider with calibration parameters
   */
  CalibratedImage calibrated_img;

  /**
   * The img currently displayed
   */
  cv::Mat display_img;

  /**
   * The actual timestamp of the video in the camera
   */
  uint64_t now;

  /**
   * Duration of a time step
   */
  int step_ms;

  /**
   * Is the replay in progress or not
   */
  bool playing;

  /**
   * Current reading speed
   */
  double speed;

  /**
   * When set to true, end run after the next step
   */
  bool end;
};

}  // namespace hl_monitoring
