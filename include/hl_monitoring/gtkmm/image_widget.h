#include <gtkmm.h>

#include <opencv2/core.hpp>

namespace hl_monitoring
{
class ImageWidget : public Gtk::EventBox
{
public:
  ImageWidget();

  bool hasImage() const;
  void updateImage(const cv::Mat& img);
  void on_size_allocate(Gtk::Allocation& allocation) override;

  bool on_mouse_button_press(GdkEventButton* event);

private:
  void updatePixbuf();

  Gtk::Image img_widget;

  // Stores memory
  cv::Mat rgb_img;
  /**
   * Original image scaled to fit the area
   */
  cv::Mat scaled_img;
  /**
   * The pixel buffer shown
   */
  Glib::RefPtr<Gdk::Pixbuf> pixbuf;
  /**
   * Scale of the display with respect to original size
   */
  double display_scale;
  /**
   * Offset along x-axis between widget coordinates and img_start
   */
  int offset_x;
  /**
   * Offset along y-axis between widget coordinates and img_start
   */
  int offset_y;
};

}  // namespace hl_monitoring
