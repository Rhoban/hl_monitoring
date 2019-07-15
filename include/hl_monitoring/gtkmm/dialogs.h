#pragma once

#include <gtkmm.h>

namespace hl_monitoring
{
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

}  // namespace hl_monitoring
