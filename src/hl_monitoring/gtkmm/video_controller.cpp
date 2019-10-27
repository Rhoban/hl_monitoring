#include <hl_monitoring/gtkmm/video_controller.h>

#include <hl_communication/utils.h>
#include <iomanip>
#include <iostream>

using namespace std;

namespace hl_monitoring
{
VideoController::VideoController()
  : rewind_button(Gtk::Stock::MEDIA_REWIND)
  , forward_button(Gtk::Stock::MEDIA_FORWARD)
  , start(0)
  , end(0)
  , now(0)
  , speed(1.0)
  , is_playing(false)
{
  updatePlayButton();
  time_bar.signal_format_value().connect(sigc::mem_fun(*this, &VideoController::on_timebar_format_value));
  time_bar.signal_change_value().connect(sigc::mem_fun(*this, &VideoController::on_timebar_change_value));
  // Custom steps: basic increment -> 20ms, page_up/page_down -> 5 sec
  time_bar.set_increments(0.02, 5.0);
  // Connecting buttons signals
  rewind_button.signal_clicked().connect(sigc::mem_fun(*this, &VideoController::on_rewind));
  play_button.signal_clicked().connect(sigc::mem_fun(*this, &VideoController::on_play_toggle));
  forward_button.signal_clicked().connect(sigc::mem_fun(*this, &VideoController::on_forward));
  // Setting position of different elements
  resize(2, 3);
  attach(time_bar, 0, 3, 0, 1);
  attach(rewind_button, 0, 1, 1, 2);
  attach(play_button, 1, 2, 1, 2);
  attach(forward_button, 2, 3, 1, 2);
  show_all_children();
  // Initialize with current time as default
  last_tick = hl_communication::getTimeStamp();
}

VideoController::~VideoController()
{
}

void VideoController::setTimeLimits(uint64_t new_start, uint64_t new_end)
{
  if (new_end < new_start)
    throw std::logic_error(HL_DEBUG + "end < start (" + std::to_string(new_end) + ", " + std::to_string(new_start) +
                           ")");
  start = new_start;
  end = new_end;
  now = boundTime(now);
  time_bar.set_range(start / std::pow(10, 6), end / std::pow(10, 6));
}

void VideoController::tickTime()
{
  uint64_t new_tick = hl_communication::getTimeStamp();
  uint64_t elapsed_us = new_tick - last_tick;
  if (is_playing)
  {
    int64_t dt = elapsed_us * speed;
    now = boundTime(now + dt);
  }
  last_tick = new_tick;
  updateTimeBar();
}

uint64_t VideoController::getTime() const
{
  return now;
}

void VideoController::on_play_toggle()
{
  is_playing = play_button.get_active();
  if (is_playing)
    speed = 1;
  updatePlayButton();
}

void VideoController::on_rewind()
{
  if (speed >= 0)
    speed = -2;
  else
    speed *= 2;
  play_button.set_active();
  updatePlayButton();
}

void VideoController::on_forward()
{
  if (speed <= 0)
    speed = 2;
  else
    speed *= 2;
  play_button.set_active();
  updatePlayButton();
}

Glib::ustring VideoController::on_timebar_format_value(double value)
{
  int64_t elapsed_ms = value * std::pow(10, 3) - start / std::pow(10, 3);
  int ms = elapsed_ms % 1000;
  int sec = (elapsed_ms / 1000) % 60;
  int min = (elapsed_ms / 60000) % 60;
  int hour = (elapsed_ms / (60 * 60 * 1000));
  std::ostringstream oss;
  oss << setfill('0');
  if (hour > 0)
    oss << setw(2) << hour << "h";
  if (min > 0)
    oss << setw(2) << min << "m";
  oss << setw(2) << sec << ":" << setw(3) << ms;
  return oss.str();
}

bool VideoController::on_timebar_change_value(Gtk::ScrollType scroll, double new_value)
{
  now = new_value * std::pow(10, 6);
  return true;
}

void VideoController::force_pause()
{
  is_playing = false;
  updatePlayButton();
}

uint64_t VideoController::boundTime(uint64_t t) const
{
  return std::min(end, std::max(start, t));
}

void VideoController::updatePlayButton()
{
  if (is_playing)
    Gtk::Stock::lookup(Gtk::Stock::MEDIA_PAUSE, Gtk::BuiltinIconSize::ICON_SIZE_BUTTON, play_img);
  else
    Gtk::Stock::lookup(Gtk::Stock::MEDIA_PLAY, Gtk::BuiltinIconSize::ICON_SIZE_BUTTON, play_img);
  play_button.set_image(play_img);
}

void VideoController::updateTimeBar()
{
  time_bar.set_value(now / std::pow(10, 6));
}

}  // namespace hl_monitoring
