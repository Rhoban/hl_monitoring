#include <hl_monitoring/gtkmm/multi_camera_widget.h>

#include <hl_monitoring/gtkmm/dialogs.h>

namespace hl_monitoring
{
MultiCameraWidget::MultiCameraWidget() : load_replay_button("Load replay")
{
  load_replay_button.signal_clicked().connect(sigc::mem_fun(*this, &TestWindow::on_button_clicked));
  load_buttons.add(load_replay_button);
  load_replay_button.show();
  add(load_buttons);
  load_buttons.show();
  add(available_sources);
  available_sources.show();
  add(image_tables);
  image_tables.show();
  add(video_ctrl);
  video_ctrl.show();
  // Starting timer
  int tick_period_ms = 30;
  sigc::slot<bool> time_slot(sigc::mem_fun(*this, &ReplayWidget::tick));
  Glib::signal_timeout().connect(time_slot, tick_period_ms);
}

MultiCameraWidget::~MultiCameraWidget()
{
}

void MultiCameraWidget::on_load_replay()
{
  std::string video_file;
  if (requestVideoFile(this, &video_file))
  {
    std::string protobuf_file;
    std::unique_ptr<ReplayImageProvider> provider;
    if (requestProtobufFile(this, &protobuf_file))
      provider.reset(new ReplayImageProvider(video_file, protobuf_file));
    else
      provider.reset(new ReplayImageProvider(video_file));
    const VideoSourceID& source_id = provider->getMetaInformation().source_id();
    std::ostringstream source_oss;
    if (source_id.has_robot_source())
      source_oss << source_id.robot_source();
    else if (source_id.has_external_source())
      source_oss << source_id.external_source();
    else
      throw std::logic_error(HL_DEBUG + "Unknown type of source");
    std::string source_name = source_oss.str();

    manager.addImageProvider(source_name, std::move(provider));
    video_ctrl.setTimeLimits(manager.getStart(), manager.getEnd());
    if (display_areas.count(source_name) == 0)
    {
      display_areas[source_name] = ImageWidget();
      activation_buttons[source_name] = Gtk::ToggleButton(source_name);
      std::cout << "Adding button " << source_name << std::endl;
    }
  }
}

bool MultiCameraWidget::tick()
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
  last_tick = now;
  return true;
}

}  // namespace hl_monitoring
