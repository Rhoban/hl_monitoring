#include <hl_monitoring/gtkmm/dialogs.h>
#include <hl_monitoring/gtkmm/replay_widget.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>

using namespace hl_monitoring;

class TestWindow : public Gtk::Window
{
public:
  TestWindow() : button("Choose Log")
  {
    set_border_width(10);
    button.signal_clicked().connect(sigc::mem_fun(*this, &TestWindow::on_button_clicked));
    layout.add(button);
    button.show();
    layout.add(replay_widget);
    replay_widget.show();
    add(layout);
    layout.show();
    layout.set_size_request(800, 600);
  }

private:
  void on_button_clicked()
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
      replay_widget.setImageProvider(std::move(provider));
    }
  }

  Gtk::Button button;
  ReplayWidget replay_widget;
  Gtk::VBox layout;
};

int main(int argc, char* argv[])
{
  Gtk::Main kit(argc, argv);
  TestWindow window;
  window.set_default_size(200, 200);
  Gtk::Main::run(window);
}
