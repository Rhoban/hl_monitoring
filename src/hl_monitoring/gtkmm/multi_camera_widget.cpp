#include <hl_monitoring/gtkmm/multi_camera_widget.h>

#include <hl_monitoring/replay_image_provider.h>
#include <hl_monitoring/gtkmm/dialogs.h>

#include <opencv2/imgproc.hpp>

#include <iostream>

using namespace hl_communication;

namespace hl_monitoring
{
MultiCameraWidget::MultiCameraWidget()
  : load_replay_button("Load replay")
  , load_folder_button("Load folder")
  , top_view_drawer(cv::Size(640, 480))
  , last_annotation(0)
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
  // Adding the TopView specific source
  std::string top_view_name = "TopView";
  VideoSourceID top_view_id;
  top_view_id.set_external_source(top_view_name);
  sources[top_view_name].source_id = top_view_id;
  sources[top_view_name].activated = true;
  sources[top_view_name].display_area = new ImageWidget();
  sources[top_view_name].activation_button = new Gtk::ToggleButton(top_view_name);
  sources[top_view_name].activation_button->show();
  sources[top_view_name].activation_button->set_active(true);
  for (auto& handler : handlers)
  {
    sources[top_view_name].display_area->registerClickHandler(
        [top_view_id, handler](const cv::Point2f& pos) { handler(top_view_id, pos); });
  }
  available_sources.add(*sources[top_view_name].activation_button);
  refreshTables();
}

MultiCameraWidget::~MultiCameraWidget()
{
  for (auto& entry : sources)
  {
    delete (entry.second.display_area);
    delete (entry.second.activation_button);
  }
}
void MultiCameraWidget::refreshTables()
{
  for (const std::string& source : last_active_sources)
    image_tables.remove(*(sources[source].display_area));
  std::set<std::string> new_active_sources = getActiveSources();
  int nb_active_sources = new_active_sources.size();
  // TODO: ideally, nb rows and nb cols should depend on the size of widget
  int nb_rows = std::max(1, (int)std::floor(std::sqrt(nb_active_sources)));
  int nb_cols = std::max(1, (int)std::ceil(std::sqrt(nb_active_sources)));
  image_tables.resize(nb_rows, nb_cols);
  // Filling table
  int row(0), col(0);
  for (auto& entry : sources)
  {
    if (!entry.second.activation_button->get_active())
      continue;
    image_tables.attach(*entry.second.display_area, col, col + 1, row, row + 1);
    col++;
    if (col == nb_cols)
    {
      col = 0;
      row++;
    }
  }
  last_active_sources = new_active_sources;
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
          [source_id, handler](const cv::Point2f& pos) { handler(source_id, pos); });
    }
    // To keep alphabetical order, each time a new source is added, all elements are removed and added
    for (auto& entry : sources)
    {
      if (entry.first != source_name)
        available_sources.remove(*entry.second.activation_button);
      available_sources.add(*entry.second.activation_button);
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
  // Loading a replay might require to update annotations
  step(false);
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
  // Loading a folder might require to update annotations
  step(false);
}

bool MultiCameraWidget::tick()
{
  video_ctrl.tickTime();
  step();
  return true;
}

void MultiCameraWidget::step(bool lazy_annotations)
{
  checkActivity();
  updateCalibratedImages();
  if (!lazy_annotations || last_annotation != video_ctrl.getTime())
  {
    updateAnnotations();
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
void MultiCameraWidget::checkActivity()
{
  bool has_window_changed = false;
  std::set<std::string> last_active_sources = getActiveSources();
  for (auto& entry : sources)
  {
    SourceStatus& status = entry.second;
    bool button_active = status.activation_button->get_active();
    if (status.activated != button_active)
    {
      status.activated = button_active;
      has_window_changed = true;
      if (button_active)
        status.display_area->show();
      else
        status.display_area->hide();
    }
  }
  if (has_window_changed)
  {
    refreshTables();
  }
}

void MultiCameraWidget::updateCalibratedImages()
{
  uint64_t now = video_ctrl.getTime();
  for (auto& entry : sources)
  {
    const std::string& name = entry.first;
    SourceStatus& status = entry.second;
    if (!status.activated)
      continue;
    bool is_top_view = isTopViewID(status.source_id);
    uint64_t source_ts = now;
    try
    {
      if (!is_top_view)
        source_ts = manager.getImageProvider(name, now).getFrameEntry(now).utc_ts();
      if (source_ts != status.timestamp)
      {
        if (is_top_view)
        {
          status.calibrated_image =
              CalibratedImage(top_view_drawer.getImg(manager.getField()), hl_communication::CameraMetaInformation());
        }
        else
        {
          status.calibrated_image = manager.getCalibratedImage(name, source_ts);
        }
        status.timestamp = source_ts;
      }
    }
    catch (const std::out_of_range& exc)
    {
      std::cout << exc.what() << std::endl;
    }
  }
}

void MultiCameraWidget::updateAnnotations()
{
  for (auto& entry : sources)
  {
    const std::string& name = entry.first;
    SourceStatus& status = entry.second;
    if (!status.activated)
      continue;
    const cv::Mat& raw_img = status.calibrated_image.getImg();
    if (raw_img.empty())
      continue;
    status.display_image = raw_img.clone();
    annotateImg(name);
    status.display_area->updateImage(status.display_image);
  }
  last_annotation = video_ctrl.getTime();
}

void MultiCameraWidget::annotateImg(const std::string& name)
{
  (void)name;
  uint64_t now = video_ctrl.getTime();
  int64_t frame_age = now - (int64_t)sources[name].timestamp;
  cv::Mat& display_img = sources[name].display_image;
  std::string msg;
  if (frame_age < 0)
  {
    // TODO: timestamp to human?
    msg = "Next frame in " + std::to_string(-frame_age / 1000) + " ms";
  }
  else if (frame_age > 500 * 1000)
  {
    msg = "Outdated frame: " + std::to_string(frame_age / 1000) + " ms";
  }
  if (msg != "")
  {
    int font_face = cv::FONT_HERSHEY_PLAIN;
    double font_scale = 2.0;
    int thickness = 2;
    int baseline = 0;
    cv::Point center(display_img.cols / 2, display_img.rows / 2);
    cv::Size text_size = cv::getTextSize(msg, font_face, font_scale, thickness, &baseline);
    cv::Point pos(center.x - text_size.width / 2, center.y - text_size.height / 2);
    std::cout << "Drawing message to " << pos << std::endl;
    cv::putText(display_img, msg, pos, font_face, font_scale, cv::Scalar(255, 0, 255), thickness, cv::LINE_AA);
  }
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
uint32_t MultiCameraWidget::getFrameIndex(const hl_communication::VideoSourceID& id)
{
  std::string name = getName(id);
  uint64_t time_stamp = sources[name].timestamp;
  return manager.getImageProvider(name, time_stamp).getIndex(time_stamp);
}

void MultiCameraWidget::registerClickHandler(MouseClickHandler handler)
{
  handlers.push_back(handler);
  for (auto& entry : sources)
  {
    hl_communication::VideoSourceID source_id = entry.second.source_id;
    entry.second.display_area->registerClickHandler(
        [source_id, handler](const cv::Point2f& pos) { handler(source_id, pos); });
  }
}

bool MultiCameraWidget::isTopViewID(const hl_communication::VideoSourceID& id)
{
  return id.has_external_source() && id.external_source() == "TopView";
}

}  // namespace hl_monitoring
