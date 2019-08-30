#pragma once

#include <hl_monitoring/monitoring_manager.h>
#include <hl_monitoring/top_view_drawer.h>
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
  typedef std::function<void(const hl_communication::VideoSourceID& source_id, const cv::Point2f& click_pos)>
      MouseClickHandler;
  MultiCameraWidget();
  virtual ~MultiCameraWidget();

  bool tick();

  /**
   * If lazy annotations is enabled, update annotations only if the timestamp of the videoControl has changed since last
   * annotation's update
   */
  void step(bool lazy_annotations = true);

  /**
   * Returns a list of all the active sources
   */
  std::set<std::string> getActiveSources() const;

  /**
   * Check if buttons toggling display of the videos have been used. Updates layout if necessary
   */
  void checkActivity();

  /**
   * Reload the source images from the videos based on current timestamp in a lazy way (i.e. images are loaded only if
   * their timestamp differs from previous one).
   */
  void updateCalibratedImages();
  /**
   * Update the annotations on all the images displayed using the calibrated images as last source.
   */
  void updateAnnotations();
  /**
   * Draws annotation on the image specified by the given name
   */
  virtual void annotateImg(const std::string& name);

  static std::string getName(const hl_communication::VideoSourceID& id);

  const hl_communication::VideoSourceID& getDetailedSourceID(const hl_communication::VideoSourceID& id);

  uint32_t getFrameIndex(const hl_communication::VideoSourceID& id);

  void registerClickHandler(MouseClickHandler handler);

  static bool isTopViewID(const hl_communication::VideoSourceID& id);

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
   * Update the number of rows and columns and updates the display areas according to current state of the buttons
   */
  void refreshTables();

  /**
   * Add the given provider to the manager and updates internal representation if required
   */
  virtual void addProvider(std::unique_ptr<ImageProvider> provider);

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
  /**
   * The list of sources actives since last refreshTables
   */
  std::set<std::string> last_active_sources;

  VideoController video_ctrl;

  /**
   * Generic handlers for click inside the areas
   */
  std::vector<MouseClickHandler> handlers;

  /**
   * The drawer used to provide a top view
   */
  TopViewDrawer top_view_drawer;

  /**
   * The timestamp for which annotation was performed last time
   */
  uint64_t last_annotation;
};

}  // namespace hl_monitoring
