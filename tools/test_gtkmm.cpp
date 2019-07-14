#include <hl_monitoring/gtkmm/image_widget.h>

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
    layout.add(img_widget);
    add(layout);
    button.show();
    img_widget.show();
    layout.show();
    layout.set_size_request(500, 600);
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

  Gtk::Button button;
  ImageWidget img_widget;
  Gtk::VBox layout;
};

int main(int argc, char* argv[])
{
  Gtk::Main kit(argc, argv);
  TestWindow window;
  window.set_default_size(200, 200);
  Gtk::Main::run(window);
}
