#include <gtkmm.h>

#include <opencv2/core.hpp>

namespace hl_monitoring
{
class ImageWidget : public Gtk::EventBox
{
public:
  /**
   * Event handler receiving position of click inside image referential
   */
  typedef std::function<void(cv::Point2f)> MouseClickHandler;

  ImageWidget();

  bool hasImage() const;
  void updateImage(const cv::Mat& img);

  void registerClickHandler(MouseClickHandler handler);

  void on_size_allocate(Gtk::Allocation& allocation) override;

  bool on_mouse_button_press(GdkEventButton* event);

  /**
   * Return the size of the display image
   */
  cv::Size getImgSize() const;

  /**
   * Updates the scale ratio for the display image based on allocated size
   */
  void updateDisplayScale(int allocated_width, int allocated_height);

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

  std::vector<MouseClickHandler> click_handlers;
};

}  // namespace hl_monitoring
