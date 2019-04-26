#pragma once

#include <hl_monitoring/drawers/drawer.h>

namespace hl_monitoring
{

/**
 * Can be used to draw text at a given position in field
 */
class TextDrawer : public Drawer<std::pair<cv::Point3f,std::string>>
{
public:
  TextDrawer();
  ~TextDrawer();
  void draw(FieldToImgConverter converter, const std::pair<cv::Point3f,std::string>& data, cv::Mat* out) override;
  Json::Value toJson() const override;
  void fromJson(const Json::Value& v) override;

  void setColor(const cv::Scalar& new_color);
  void setImgOffset(const cv::Point2f& new_offset);

private:
  /**
   * Can be used to have an offset in the image in order to avoid writing the text over an existing feature
   */
  cv::Point2f img_offset;

  /**
   * Multiplies based font size
   */
  double font_scale;

  double font_thickness;

  cv::Scalar color;
};

}  // namespace hl_monitoring
