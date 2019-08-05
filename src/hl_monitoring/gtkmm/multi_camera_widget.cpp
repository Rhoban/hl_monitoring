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
  for (auto& entry : sources)
  {
    delete (entry.second.display_area);
    delete (entry.second.activation_button);
  }
}

void MultiCameraWidget::addProvider(std::unique_ptr<ImageProvider> provider)
{
  const VideoSourceID& source_id = provider->getMetaInformation().source_id();
  std::string source_name = getName(source_id);
  manager.addImageProvider(source_name, std::move(provider));
  video_ctrl.setTimeLimits(manager.getStart(), manager.getEnd());
  if (sources.count(source_name) == 0)
  {
    sources[source_name].source_id = source_id;
    sources[source_name].activated = false;
    sources[source_name].display_area = new ImageWidget();
    sources[source_name].activation_button = new Gtk::ToggleButton(source_name);
    sources[source_name].activation_button->show();
    sources[source_name].activation_button->set_active(true);
    for (auto& handler : handlers)
    {
      sources[source_name].display_area->registerClickHandler(
          [source_name, handler](const cv::Point2f& pos) { handler(source_name, pos); });
    }
  }
}

void MultiCameraWidget::on_load_replay()
{
  std::string video_file;
  Gtk::Window* window = (Gtk::Window*)get_toplevel();
  if (requestVideoFile(window, &video_file))
  {
    std::string protobuf_file;
    std::unique_ptr<ImageProvider> provider;
    if (requestProtobufFile(window, &protobuf_file))
      provider.reset(new ReplayImageProvider(video_file, protobuf_file));
    else
      provider.reset(new ReplayImageProvider(video_file));
    addProvider(std::move(provider));
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
      addProvider(std::move(provider));
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
  bool has_window_changed = false;
  std::set<std::string> last_active_sources = getActiveSources();
  for (auto& entry : sources)
  {
    const std::string& name = entry.first;
    SourceStatus& status = entry.second;
    if (!status.activation_button->get_active())
    {
      if (status.activated)
      {
        status.activated = false;
        has_window_changed = true;
        std::cout << "Hidding area for : " << name << std::endl;
      }
      continue;
    }

    try
    {
      uint64_t now = video_ctrl.getTime();
      uint64_t source_ts = manager.getImageProvider(name, now).getFrameEntry(now).utc_ts();
      // Update image only if source_ts has changed
      if (source_ts != status.timestamp)
      {
        status.calibrated_image = manager.getCalibratedImage(name, source_ts);
        status.display_image = status.calibrated_image.getImg().clone();
        annotateImg(name);
        status.display_area->updateImage(status.display_image);
        status.timestamp = source_ts;
      }
    }
    catch (const std::out_of_range& exc)
    {
      std::cout << exc.what() << std::endl;
    }
    if (!status.activated)
    {
      status.display_area->show();
      status.activated = true;
      has_window_changed = true;
      std::cout << "Displaying area for : " << name << std::endl;
    }
  }
  if (has_window_changed)
  {
    for (const std::string& source : last_active_sources)
      image_tables.remove(*(sources[source].display_area));
    int row(0), col(0);
    int nb_rows(1), nb_cols(2);
    image_tables.resize(nb_rows, nb_cols);
    for (auto& entry : sources)
    {
      image_tables.attach(*entry.second.display_area, col, col + 1, row, row + 1);
      col++;
      if (col == nb_cols)
      {
        col = 0;
        row++;
      }
    }
  }
}

std::set<std::string> MultiCameraWidget::getActiveSources() const
{
  std::set<std::string> result;
  for (const auto& entry : sources)
    if (entry.second.activated)
      result.insert(entry.first);
  return result;
}

void MultiCameraWidget::annotateImg(const std::string& name)
{
  (void)name;
}

std::string MultiCameraWidget::getName(const hl_communication::VideoSourceID& source_id)
{
  std::ostringstream source_oss;
  if (source_id.has_robot_source())
    source_oss << source_id.robot_source();
  else if (source_id.has_external_source())
    source_oss << source_id.external_source();
  else
    throw std::logic_error(HL_DEBUG + "Unknown type of source");
  return source_oss.str();
}

void MultiCameraWidget::registerClickHandler(MouseClickHandler handler)
{
  handlers.push_back(handler);
  for (auto& entry : sources)
  {
    std::string name = entry.first;
    entry.second.display_area->registerClickHandler([name, handler](const cv::Point2f& pos) { handler(name, pos); });
  }
}

}  // namespace hl_monitoring
