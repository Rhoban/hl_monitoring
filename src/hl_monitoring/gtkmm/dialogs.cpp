#include <hl_monitoring/gtkmm/dialogs.h>

#include <hl_communication/utils.h>

#include <iostream>

namespace hl_monitoring
{
bool requestFile(Gtk::Window* window, const std::string& pattern_name, const std::vector<std::string>& patterns,
                 std::string* path)
{
  Gtk::FileChooserDialog dialog("Request file", Gtk::FILE_CHOOSER_ACTION_OPEN);
  dialog.set_transient_for(*window);
  dialog.add_button(Gtk::Stock::CANCEL, Gtk::RESPONSE_CANCEL);
  dialog.add_button(Gtk::Stock::OPEN, Gtk::RESPONSE_OK);
  Gtk::FileFilter filter;
  filter.set_name(pattern_name);
  for (const std::string& pattern : patterns)
    filter.add_pattern(pattern);
  dialog.add_filter(filter);
  int result = dialog.run();
  switch (result)
  {
    case (Gtk::RESPONSE_OK):
    {
      *path = std::string(dialog.get_filename());
      return true;
    }
    case (Gtk::RESPONSE_CANCEL):
    {
      std::cout << HL_DEBUG << "request file canceled by user" << std::endl;
      break;
    }
    default:
      std::cout << HL_DEBUG << "Unexpected button pressed" << std::endl;
  }
  return false;
}

bool requestVideoFile(Gtk::Window* window, std::string* path)
{
  return requestFile(window, "video file", { "*.avi" }, path);
}

bool requestProtobufFile(Gtk::Window* window, std::string* path)
{
  return requestFile(window, "protobuf file", { "*.pb", "*.bin" }, path);
}

}  // namespace hl_monitoring
