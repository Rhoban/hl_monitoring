#include <hl_monitoring/gtkmm/image_widget.h>
#include <hl_monitoring/gtkmm/video_controller.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>

using namespace hl_monitoring;

class TestWindow : public Gtk::Window
{
public:
  TestWindow() : button("Choose image")
  {
    set_border_width(10);
    button.signal_clicked().connect(sigc::mem_fun(*this, &TestWindow::on_button_clicked));
    layout.add(button);
    button.show();
    layout.add(img_widget);
    img_widget.show();
    layout.add(video_controller);
    video_controller.show();
    add(layout);
    layout.show();
    layout.set_size_request(500, 600);
    int tick_period_ms = 50;
    sigc::slot<bool> time_slot(sigc::bind(sigc::mem_fun(*this, &TestWindow::on_time_tick), 0));
    Glib::signal_timeout().connect(time_slot, tick_period_ms);
    video_controller.setTimeLimits(std::pow(10, 6), std::pow(10, 8));
  }

private:
  void on_button_clicked()
  {
    Gtk::FileChooserDialog dialog("Please choose a file", Gtk::FILE_CHOOSER_ACTION_OPEN);
    dialog.set_transient_for(*this);
    dialog.add_button("_Cancel", Gtk::RESPONSE_CANCEL);
    dialog.add_button("Select", Gtk::RESPONSE_OK);
    int result = dialog.run();
    switch (result)
    {
      case (Gtk::RESPONSE_OK):
      {
        std::cout << "Select clicked." << std::endl;
        std::cout << "File selected: " << dialog.get_filename() << std::endl;
        cv::Mat mat = cv::imread(std::string(dialog.get_filename()));
        img_widget.updateImage(mat);
        break;
      }
      case (Gtk::RESPONSE_CANCEL):
      {
        std::cout << "Cancel clicked." << std::endl;
        break;
      }
      default:
      {
        std::cout << "Unexpected button clicked." << std::endl;
        break;
      }
    }
  }

  bool on_time_tick(int timer)
  {
    (void)timer;
    video_controller.tickTime();
    return true;
  }

  Gtk::Button button;
  ImageWidget img_widget;
  VideoController video_controller;
  Gtk::VBox layout;
};

int main(int argc, char* argv[])
{
  Gtk::Main kit(argc, argv);
  TestWindow window;
  window.set_default_size(200, 200);
  Gtk::Main::run(window);
}
