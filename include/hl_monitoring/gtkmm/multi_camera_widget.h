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
  MultiCameraWidget();
  virtual ~MultiCameraWidget();

  bool tick();

  void step();

  /**
   * Draws annotation on the image specified by the given name
   */
  virtual void annotateImg(const std::string& name);

protected:
  /**
   * Opens dialog box and load a window
   */
  void on_load_replay();

  /**
   * Guess and extract all available logs from a folder
   */
  void on_load_folder();

protected:
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

  std::set<std::string> active_sources;
  std::map<std::string, ImageWidget*> display_areas;
  std::map<std::string, hl_communication::VideoSourceID> source_ids;
  std::map<std::string, Gtk::ToggleButton*> activation_buttons;
  std::map<std::string, CalibratedImage> calibrated_images;
  std::map<std::string, cv::Mat> display_images;
  std::map<std::string, uint64_t> timestamp_by_image;

  VideoController video_ctrl;

  uint64_t last_tick;
};

}  // namespace hl_monitoring
