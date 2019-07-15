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
    Gtk::FileChooserDialog dialog("Please choose video", Gtk::FILE_CHOOSER_ACTION_OPEN);
    dialog.set_transient_for(*this);
    dialog.add_button("_Cancel", Gtk::RESPONSE_CANCEL);
    dialog.add_button("Select", Gtk::RESPONSE_OK);
    int result = dialog.run();
    switch (result)
    {
      case (Gtk::RESPONSE_OK):
      {
        std::string file = dialog.get_filename();
        // TODO: add_metadata
        std::unique_ptr<ReplayImageProvider> provider(new ReplayImageProvider(file));
        replay_widget.setImageProvider(std::move(provider));
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
