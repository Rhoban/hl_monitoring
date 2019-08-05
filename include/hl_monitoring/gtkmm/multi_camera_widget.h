#pragma once

#include <hl_monitoring/field.h>
#include <hl_monitoring/monitoring_manager.h>
#include <hl_monitoring/gtkmm/image_widget.h>
#include <hl_monitoring/gtkmm/video_controller.h>

namespace hl_monitoring
{
/**
 * A widget with multiple cameras allowed
 */
class MultiCameraWidget : public Gtk::VBox
{
public:
  typedef std::function<void(const std::string& name, const cv::Point2f& click_pos)> MouseClickHandler;
  MultiCameraWidget();
  virtual ~MultiCameraWidget();

  bool tick();

  void step();

  /**
   * Returns a list of all the active sources
   */
  std::set<std::string> getActiveSources() const;

  /**
   * Draws annotation on the image specified by the given name
   */
  virtual void annotateImg(const std::string& name);

  static std::string getName(const hl_communication::VideoSourceID& id);

  void registerClickHandler(MouseClickHandler handler);

protected:
  struct SourceStatus
  {
    hl_communication::VideoSourceID source_id;
    /**
     * Is image currently displayed
     */
    bool activated = false;
    ImageWidget* display_area;
    Gtk::ToggleButton* activation_button;
    /**
     * Raw image from the monitoring manager
     */
    CalibratedImage calibrated_image;
    /**
     * Image which is displayed, can potentially be annotated
     */
    cv::Mat display_image;
    uint64_t timestamp;
  };

  /**
   * Add the given provider to the manager and updates internal representation if required
   */
  void addProvider(std::unique_ptr<ImageProvider> provider);

  /**
   * Opens dialog box and load a window
   */
  void on_load_replay();

  /**
   * Guess and extract all available logs from a folder
   */
  void on_load_folder();

  /**
   * Provide access to all the images and messages
   */
  MonitoringManager manager;
  /**
   * Buttons to load replay or message
   */
  Gtk::HBox load_buttons;
  /**
   * All the sources which are not included yet
   */
  Gtk::HBox available_sources;
  /**
   * Container for images shown
   */
  Gtk::Table image_tables;

  Gtk::Button load_replay_button;
  Gtk::Button load_folder_button;

  std::map<std::string, SourceStatus> sources;

  VideoController video_ctrl;

  uint64_t last_tick;
  /**
   * Generic handlers for click inside the areas
   */
  std::vector<MouseClickHandler> handlers;
};

}  // namespace hl_monitoring
