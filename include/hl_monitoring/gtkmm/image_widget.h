#include <gtkmm.h>

#include <opencv2/core.hpp>

namespace hl_monitoring
{
class ImageWidget : public Gtk::Image
{
public:
  ImageWidget();

  void updateImage(const cv::Mat& img);
  void on_size_allocate(Gtk::Allocation& allocation) override;

private:
  void updatePixbuf();

  // Stores memory
  cv::Mat rgb_img;
  // Scaled img
  cv::Mat scaled_img;
  Glib::RefPtr<Gdk::Pixbuf> pixbuf;
};

}  // namespace hl_monitoring
