#include <hl_monitoring/gtkmm/multi_camera_widget.h>

#include <hl_monitoring/replay_image_provider.h>
#include <hl_monitoring/gtkmm/dialogs.h>

#include <iostream>

using namespace hl_communication;

namespace hl_monitoring
{
MultiCameraWidget::MultiCameraWidget() : load_replay_button("Load replay"), load_folder_button("Load folder")
{
  load_replay_button.signal_clicked().connect(sigc::mem_fun(*this, &MultiCameraWidget::on_load_replay));
  load_buttons.add(load_replay_button);
  load_replay_button.show();
  load_folder_button.signal_clicked().connect(sigc::mem_fun(*this, &MultiCameraWidget::on_load_folder));
  load_buttons.add(load_folder_button);
  load_folder_button.show();
  add(load_buttons);
  load_buttons.show();
  add(available_sources);
  available_sources.show();
  add(image_tables);
  image_tables.set_homogeneous(true);
  image_tables.show();
  add(video_ctrl);
  video_ctrl.show();
  // Starting timer
  int tick_period_ms = 30;
  sigc::slot<bool> time_slot(sigc::mem_fun(*this, &MultiCameraWidget::tick));
  Glib::signal_timeout().connect(time_slot, tick_period_ms);
}

MultiCameraWidget::~MultiCameraWidget()
{
  for (auto& entry : display_areas)
    delete (entry.second);
  for (auto& entry : activation_buttons)
    delete (entry.second);
}

void MultiCameraWidget::on_load_replay()
{
  std::string video_file;
  Gtk::Window* window = (Gtk::Window*)get_toplevel();
  if (requestVideoFile(window, &video_file))
  {
    std::string protobuf_file;
    std::unique_ptr<ReplayImageProvider> provider;
    if (requestProtobufFile(window, &protobuf_file))
      provider.reset(new ReplayImageProvider(video_file, protobuf_file));
    else
      provider.reset(new ReplayImageProvider(video_file));
    std::cout << "Getting SourceId" << std::endl;
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
      display_areas[source_name] = new ImageWidget();
      activation_buttons[source_name] = new Gtk::ToggleButton(source_name);
      available_sources.add(*activation_buttons[source_name]);
      activation_buttons[source_name]->show();
      activation_buttons[source_name]->set_active(true);
    }
  }
}

void MultiCameraWidget::on_load_folder()
{
  std::string log_folder;
  Gtk::Window* window = (Gtk::Window*)get_toplevel();
  if (requestFolder(window, &log_folder))
  {
    std::vector<std::unique_ptr<ImageProvider>> logs = ReplayImageProvider::loadReplays(log_folder);
    for (std::unique_ptr<ImageProvider>& provider : logs)
    {
      // TODO: avoid code duplication (with on_load_replay)
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
      if (display_areas.count(source_name) == 0)
      {
        display_areas[source_name] = new ImageWidget();
        activation_buttons[source_name] = new Gtk::ToggleButton(source_name);
        available_sources.add(*activation_buttons[source_name]);
        activation_buttons[source_name]->show();
        activation_buttons[source_name]->set_active(true);
      }
    }
    video_ctrl.setTimeLimits(manager.getStart(), manager.getEnd());
  }
}

bool MultiCameraWidget::tick()
{
  //  if (!provider)
  //    return true;
  video_ctrl.tickTime();
  uint64_t now = video_ctrl.getTime();
  // Skip updates if time hasn't changed since last tick
  // if (now == last_tick)
  //  return true;
  step();
  last_tick = now;
  return true;
}

void MultiCameraWidget::step()
{
  bool has_changed = false;
  std::set<std::string> last_active_sources = active_sources;
  for (const auto& entry : activation_buttons)
  {
    const std::string& name = entry.first;
    ImageWidget* display_area = display_areas[name];
    bool was_active = active_sources.count(name) > 0;
    if (!entry.second->get_active())
    {
      // TODO: test if display area is present
      if (was_active)
      {
        active_sources.erase(name);
        has_changed = true;
        std::cout << "Hidding area for : " << name << std::endl;
      }
      continue;
    }

    try
    {
      calibrated_images[name] = manager.getCalibratedImage(name, video_ctrl.getTime());
      display_images[name] = calibrated_images[name].getImg().clone();
      display_area->updateImage(display_images[name]);
    }
    catch (const std::out_of_range& exc)
    {
    }
    if (!was_active)
    {
      display_area->show();
      active_sources.insert(name);
      has_changed = true;
      std::cout << "Displaying area for : " << name << std::endl;
    }
  }
  if (has_changed)
  {
    for (const std::string& source : last_active_sources)
      image_tables.remove(*display_areas[source]);
    int row(0), col(0);
    int nb_rows(1), nb_cols(2);
    image_tables.resize(nb_rows, nb_cols);
    for (const std::string& source : active_sources)
    {
      image_tables.attach(*display_areas[source], col, col + 1, row, row + 1);
      col++;
      if (col == nb_cols)
      {
        col = 0;
        row++;
      }
    }
  }
}

}  // namespace hl_monitoring
