#pragma once

#include <gtkmm.h>

namespace hl_monitoring
{
/**
 * A widget allowing to control time over a video, it provides a trackbar with a slider as well as play/pause and fast
 * forward/backward buttons
 *
 * It is the responsability of the parent to tick the VideoController regularly to update current time
 *
 * TODO: Code associated with the play/pause button could be extracted
 */
class VideoController : public Gtk::VBox
{
public:
  VideoController();
  virtual ~VideoController();

  /**
   * start and end are in microseconds
   */
  void setTimeLimits(uint64_t start, uint64_t end);

  /**
   * elapsed_us -> nb microseconds elapsed since last kick
   */
  void tickTime();

  void on_play_toggle();
  Glib::ustring on_timebar_format_value(double value);
  bool on_timebar_change_value(Gtk::ScrollType scroll, double new_value);

private:
  /**
   * Return t bounded by start and end of the current time frame
   */
  uint64_t boundTime(uint64_t t) const;

  /**
   * Update image of play button to match current state
   */
  void updatePlayButton();
  /**
   * Updates the position of the time_bar to match current time
   */
  void updateTimeBar();

  Gtk::HScale time_bar;

  Gtk::ToggleButton play_button;

  Gtk::Image play_img;

  /**
   * Time limits
   */
  uint64_t start, end;

  /**
   * Current value for the timer
   */
  uint64_t now;

  /**
   * Time stamp for last tick
   */
  uint64_t last_tick;

  /**
   * Current reading speed
   */
  double speed;

  /**
   * Is the video playing or is it paused?
   */
  bool is_playing;
};

}  // namespace hl_monitoring
