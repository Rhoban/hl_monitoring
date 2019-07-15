#include <hl_monitoring/gtkmm/replay_widget.h>

#include <hl_communication/utils.h>
#include <hl_monitoring/field.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>

using namespace hl_communication;

namespace hl_monitoring
{
ReplayWidget::ReplayWidget() : text_color(255, 255, 255), last_tick(0)
{
  add(img_widget);
  img_widget.show();
  add(video_ctrl);
  video_ctrl.show();
  int tick_period_ms = 50;
  sigc::slot<bool> time_slot(sigc::mem_fun(*this, &ReplayWidget::tick));
  Glib::signal_timeout().connect(time_slot, tick_period_ms);
}

ReplayWidget::~ReplayWidget()
{
}

void ReplayWidget::setImageProvider(std::unique_ptr<ReplayImageProvider> image_provider)
{
  provider = std::move(image_provider);
  video_ctrl.setTimeLimits(provider->getStart(), provider->getEnd());
}

void ReplayWidget::setField(const Field& new_field)
{
  field = new_field;
}

bool ReplayWidget::tick()
{
  if (!provider)
    return true;
  video_ctrl.tickTime();
  uint64_t now = video_ctrl.getTime();
  // Skip updates if time hasn't changed since last tick
  if (now == last_tick)
    return true;
  step();
  paintImg();
  img_widget.updateImage(display_img);
  return true;
}

void ReplayWidget::step()
{
  calibrated_img = provider->getCalibratedImage(video_ctrl.getTime());
  display_img = calibrated_img.getImg().clone();
}

void ReplayWidget::paintImg()
{
}

hl_communication::VideoMetaInformation ReplayWidget::getMetaInformation() const
{
  return provider->getMetaInformation();
}

hl_communication::VideoSourceID ReplayWidget::getSourceId() const
{
  return getMetaInformation().source_id();
}

}  // namespace hl_monitoring
