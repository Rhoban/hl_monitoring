#include <hl_monitoring/gtkmm/dialogs.h>

#include <hl_communication/utils.h>

#include <iostream>

namespace hl_monitoring
{
Gtk::Window* getWindow(Gtk::Widget* widget)
{
  Gtk::Window* top = dynamic_cast<Gtk::Window*>(widget->get_toplevel());
  if (top == nullptr)
    throw std::runtime_error("Trying to get window of an isolated widget");
  return top;
}

bool requestSavePath(Gtk::Window* window, std::string* path)
{
  Gtk::FileChooserDialog dialog("Choose log folder", Gtk::FILE_CHOOSER_ACTION_SAVE);
  dialog.set_transient_for(*window);
  dialog.add_button(Gtk::Stock::CANCEL, Gtk::RESPONSE_CANCEL);
  dialog.add_button(Gtk::Stock::SAVE, Gtk::RESPONSE_OK);
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
bool requestFolder(Gtk::Window* window, std::string* path)
{
  Gtk::FileChooserDialog dialog("Choose log folder", Gtk::FILE_CHOOSER_ACTION_SELECT_FOLDER);
  dialog.set_transient_for(*window);
  dialog.add_button(Gtk::Stock::CANCEL, Gtk::RESPONSE_CANCEL);
  dialog.add_button(Gtk::Stock::OPEN, Gtk::RESPONSE_OK);
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

void showMessage(Gtk::Window* window, const std::string& msg, const std::string& detailed_msg, Gtk::MessageType type)
{
  Gtk::MessageDialog dialog(*window, msg, false, type);
  if (detailed_msg != "")
  {
    dialog.set_secondary_text(detailed_msg);
  }
  dialog.run();
}

}  // namespace hl_monitoring
