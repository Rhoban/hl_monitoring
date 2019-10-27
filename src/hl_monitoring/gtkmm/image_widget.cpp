#include <hl_monitoring/gtkmm/image_widget.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

namespace hl_monitoring
{
ImageWidget::ImageWidget() : display_scale(1.0), offset_x(0), offset_y(0)
{
  add(img_widget);
  img_widget.show();
  signal_button_press_event().connect(sigc::mem_fun(*this, &ImageWidget::on_mouse_button_press));
}

void ImageWidget::on_size_allocate(Gtk::Allocation& allocation)
{
  Gtk::EventBox::on_size_allocate(allocation);
  set_allocation(allocation);
  if (hasImage())
  {
    updateDisplayScale(allocation.get_width(), allocation.get_height());
    cv::Size dst_size = getImgSize();
    cv::Size current_size(scaled_img.cols, scaled_img.rows);
    // Note: there seems to be a 1 px incompressible margin which is not related to:
    // - border from parent
    // - padding from Gtk::Image
    if (dst_size != current_size)
    {
      cv::resize(rgb_img, scaled_img, dst_size);
      updatePixbuf();
    }
  }
}

bool ImageWidget::on_mouse_button_press(GdkEventButton* event)
{
  if (event->button != 0)
  {
    double img_x = (event->x - offset_x) / display_scale;
    double img_y = (event->y - offset_y) / display_scale;
    std::cout << "Button " << event->button << " pressed at (" << event->x << "," << event->y << ") -> img: (" << img_x
              << ", " << img_y << ")" << std::endl;
    for (const MouseClickHandler& handler : click_handlers)
    {
      handler(event->button, cv::Point2f(img_x, img_y));
    }
  }
  return true;
}

bool ImageWidget::hasImage() const
{
  return !scaled_img.empty();
}

void ImageWidget::updateImage(const cv::Mat& img)
{
  if (img.channels() != 3)
    throw std::logic_error("Only 3 channels mat are accepted currently");
  cv::cvtColor(img, rgb_img, CV_BGR2RGB);
  updateDisplayScale(get_width(), get_height());
  cv::Size current_size(rgb_img.cols, rgb_img.rows);
  cv::Size dst_size = getImgSize();
  cv::resize(rgb_img, scaled_img, dst_size);
  updatePixbuf();
}

void ImageWidget::registerClickHandler(MouseClickHandler handler)
{
  click_handlers.push_back(handler);
}

cv::Size ImageWidget::getImgSize() const
{
  int dst_width = std::max(1, (int)(display_scale * rgb_img.cols));
  int dst_height = std::max(1, (int)(display_scale * rgb_img.rows));
  return cv::Size(dst_width, dst_height);
}

void ImageWidget::updateDisplayScale(int allocated_width, int allocated_height)
{
  double height_ratio = allocated_height / (double)(rgb_img.rows);
  double width_ratio = allocated_width / (double)(rgb_img.cols);
  display_scale = std::min(width_ratio, height_ratio);
  cv::Size dst_size = getImgSize();
  offset_x = (allocated_width - dst_size.width) / 2;
  offset_y = (allocated_height - dst_size.height) / 2;
}

void ImageWidget::updatePixbuf()
{
  if (hasImage())
  {
    pixbuf = Gdk::Pixbuf::create_from_data(scaled_img.data, Gdk::COLORSPACE_RGB, false, 8, scaled_img.cols,
                                           scaled_img.rows, scaled_img.step);
    img_widget.set(pixbuf);
  }
}

}  // namespace hl_monitoring
