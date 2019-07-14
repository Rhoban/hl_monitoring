#include <hl_monitoring/gtkmm/image_widget.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

namespace hl_monitoring
{
ImageWidget::ImageWidget()
{
}

void ImageWidget::on_size_allocate(Gtk::Allocation& allocation)
{
  Gtk::Image::on_size_allocate(allocation);
  if (!rgb_img.empty())
  {
    double height_ratio = allocation.get_height() / (double)(rgb_img.rows);
    double width_ratio = allocation.get_width() / (double)(rgb_img.cols);
    double ratio = std::min(width_ratio, height_ratio);
    cv::Size dst_size(ratio * rgb_img.cols, ratio * rgb_img.rows);
    cv::Size current_size(scaled_img.cols, scaled_img.rows);
    if (dst_size != current_size)
    {
      cv::resize(rgb_img, scaled_img, dst_size);
      updatePixbuf();
    }
  }
}

void ImageWidget::updateImage(const cv::Mat& img)
{
  if (img.channels() != 3)
    throw std::logic_error("Only 3 channels mat are accepted currently");
  cv::cvtColor(img, rgb_img, CV_BGR2RGB);
  scaled_img = rgb_img;
  updatePixbuf();
}

void ImageWidget::updatePixbuf()
{
  if (!scaled_img.empty())
  {
    pixbuf = Gdk::Pixbuf::create_from_data(scaled_img.data, Gdk::COLORSPACE_RGB, false, 8, scaled_img.cols,
                                           scaled_img.rows, scaled_img.step);
    set(pixbuf);
  }
}

}  // namespace hl_monitoring
