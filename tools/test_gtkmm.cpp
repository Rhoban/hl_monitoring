#include <hl_monitoring/gtkmm/multi_camera_widget.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>

using namespace hl_monitoring;

class TestWindow : public Gtk::Window
{
public:
  TestWindow()
  {
    set_border_width(10);
    add(multi_camera_widget);
    multi_camera_widget.show();
    multi_camera_widget.set_size_request(800, 600);
  }

private:
  MultiCameraWidget multi_camera_widget;
};

int main(int argc, char* argv[])
{
  Gtk::Main kit(argc, argv);
  TestWindow window;
  window.set_default_size(200, 200);
  Gtk::Main::run(window);
}
