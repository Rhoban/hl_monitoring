#pragma once

#include <hl_monitoring/field.h>
#include <hl_monitoring/replay_image_provider.h>
#include <hl_monitoring/gtkmm/image_widget.h>
#include <hl_monitoring/gtkmm/video_controller.h>

#include <functional>

namespace hl_monitoring
{
class ReplayWidget : public Gtk::VBox
{
public:
  ReplayWidget();
  virtual ~ReplayWidget();

  void setImageProvider(std::unique_ptr<ReplayImageProvider> image_provider);
  void setField(const Field& field);

  bool tick();

  /**
   * Update timestamp and retrieve image + calibration information
   */
  virtual void step();

  /**
   * Draw elements on the display Img, default is drawing nothing
   */
  virtual void paintImg();

  hl_communication::VideoMetaInformation getMetaInformation() const;
  hl_communication::VideoSourceID getSourceId() const;

  /**
   * The source of the images
   */
  std::unique_ptr<ReplayImageProvider> provider;

  ImageWidget img_widget;
  VideoController video_ctrl;

  /**
   * The model of the field used for the replay
   */
  Field field;

  /**
   * The image of the provider with calibration parameters
   */
  CalibratedImage calibrated_img;

  /**
   * The img currently displayed
   */
  cv::Mat display_img;

  /**
   * The color of text painted in the img
   */
  cv::Scalar text_color;

  uint64_t last_tick;
};

}  // namespace hl_monitoring
