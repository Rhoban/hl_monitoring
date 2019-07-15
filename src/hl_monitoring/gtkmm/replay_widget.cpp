#include <hl_monitoring/replay_viewer.h>

#include <hl_communication/utils.h>
#include <hl_monitoring/field.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>

using namespace hl_communication;

namespace hl_monitoring
{
ReplayWidget::ReplayWidget(std::unique_ptr<ReplayImageProvider> image_provider, bool playing, const Field& field)
  : provider(std::move(image_provider))
  , field(field)
  , now(0)
  , step_ms(33)  // Default frequency -> ~30Hz
  , playing(playing)
  , speed(1.0)
  , end(false)
  , text_color(255, 255, 255)
{
  now = provider->getStart();
  addBinding('h', "Print help", [this]() { this->printHelp(); });
  addBinding(' ', "Toggle play/pause", [this]() { this->playing = !this->playing; });
  addBinding('q', "Quit replay", [this]() { this->quit(); });
  addBinding('+', "Double speed", [this]() { this->setSpeed(2 * this->speed); });
  addBinding('-', "Divide speed by 2", [this]() { this->setSpeed(this->speed / 2); });
  addBinding('b', "Set playing direction to backward", [this]() { this->setSpeed(-std::fabs(this->speed)); });
  addBinding('f', "Set playing direction to forward", [this]() { this->setSpeed(std::fabs(this->speed)); });
}

ReplayWidget::~ReplayWidget()
{
  cv::destroyWindow(window_name);
}

void ReplayWidget::run()
{
  while (!end)
  {
    uint64_t start = getTimeStamp();
    updateTime();
    step();
    paintImg();
    cv::imshow(window_name, display_img);
    uint64_t end = getTimeStamp();
    int elapsed_ms = (end - start) / 1000;
    int wait_time_ms = std::max(5, 33 - elapsed_ms);  // 30 fps as default display
    int key(255);                                     // Default value when no key is pressed
    key = cv::waitKey(wait_time_ms);
    if (key != 255)
    {
      try
      {
        key_bindings.at(key).callback();
      }
      catch (const std::out_of_range& o)
      {
        printHelp();
      }
    }
  }
}

void ReplayWidget::printHelp()
{
  for (const auto& entry : key_bindings)
  {
    std::cout << "'" << keyCode2Str(entry.first) << "':\t" << entry.second.help_msg << std::endl;
  }
}

std::string ReplayWidget::keyCode2Str(int key)
{
  if (key <= 255)
  {
    std::string result;
    result = (char)key;
    return result;
  }
  return std::to_string(key);
}

hl_communication::VideoMetaInformation ReplayWidget::getMetaInformation() const
{
  return provider->getMetaInformation();
}

hl_communication::VideoSourceID ReplayWidget::getSourceId() const
{
  return getMetaInformation().source_id();
}

void ReplayWidget::step()
{
  calibrated_img = provider->getCalibratedImage(now);
  display_img = calibrated_img.getImg().clone();
}

void ReplayWidget::paintImg()
{
  std::string speed_status;
  if (playing)
  {
    if (speed > 0)
    {
      speed_status = "> " + std::to_string(speed) + "x";
    }
    else
    {
      speed_status = "< " + std::to_string(speed) + "x";
    }
  }
  else
  {
    speed_status = "pause";
  }
  int offset = 5;
  cv::Point bottom_left(offset, display_img.rows - offset);
  cv::putText(display_img, speed_status, bottom_left, cv::HersheyFonts::FONT_HERSHEY_SIMPLEX, 1.0, text_color);
}

void ReplayWidget::updateTime()
{
  if (playing)
  {
    now += step_ms * 1000 * speed;
    if (now > provider->getEnd())
    {
      playing = false;
      now = provider->getEnd();
      std::cerr << "End of stream reached" << std::endl;
    }
    else if (now < provider->getStart())
    {
      playing = false;
      now = provider->getStart();
      std::cerr << "Start of stream reached" << std::endl;
    }
  }
}

void ReplayWidget::addBinding(int key, const std::string& help_msg, std::function<void()> callback)
{
  if (key_bindings.count(key))
  {
    throw std::logic_error(HL_DEBUG + " key " + std::to_string(key) + " is already binded");
  }
  Action a;
  a.callback = callback;
  a.help_msg = help_msg;
  key_bindings[key] = a;
}

void ReplayWidget::quit()
{
  end = true;
}

void ReplayWidget::setSpeed(double new_speed)
{
  speed = new_speed;
}

}  // namespace hl_monitoring
