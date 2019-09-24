#pragma once

#include <gtkmm.h>

namespace hl_monitoring
{
/**
 * Return the window which is the top-level of this widget.
 * If the toplevel is not a window, throws a runtime_error
 */
Gtk::Window* getWindow(Gtk::Widget* widget);

/**
 * Request a path to save a file from window, the folder path is written to 'path'
 * Return true on success.
 */
bool requestSavePath(Gtk::Window* window, std::string* path);
/**
 * Request a folder from window, the folder path is written to 'path'
 * Return true on success.
 */
bool requestFolder(Gtk::Window* window, std::string* path);
/**
 * Request a file from window with the given pattern, the file path is written to 'path'
 * Return true on success.
 *
 * e.g. requestFile(window, "csv file", {"*.csv"}, output_path)
 */
bool requestFile(Gtk::Window* window, const std::string& pattern_name, const std::vector<std::string>& patterns,
                 std::string* path);
/**
 * Currently only supports avi
 * @see requestFile
 */
bool requestVideoFile(Gtk::Window* window, std::string* path);
/**
 * @see requestFile
 */
bool requestProtobufFile(Gtk::Window* window, std::string* path);
/**
 * Display a message box with a simple OK button.
 */
void showMessage(Gtk::Window* window, const std::string& msg, const std::string& detailed_msg = "",
                 Gtk::MessageType type = Gtk::MessageType::MESSAGE_INFO);

}  // namespace hl_monitoring
