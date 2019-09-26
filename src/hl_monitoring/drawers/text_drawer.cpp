#include <hl_monitoring/drawers/text_drawer.h>

#include <hl_communication/utils.h>

#include <opencv2/imgproc.hpp>

using namespace hl_communication;

namespace hl_monitoring
{
TextDrawer::TextDrawer() : img_offset(0, 0), font_scale(1.0), font_thickness(2.0), color(0, 0, 0)
{
}

TextDrawer::~TextDrawer()
{
}

void TextDrawer::draw(FieldToImgConverter converter, const cv::Point3f& field_position, const std::string& msg,
                      cv::Mat* out)
{
  cv::Point2f img_pos;
  if (converter(field_position, &img_pos))
    drawCenteredText(out, msg, img_pos + img_offset, color, font_thickness, font_scale);
}

void TextDrawer::draw(FieldToImgConverter converter, const std::pair<cv::Point3f, std::string>& data, cv::Mat* out)
{
  draw(converter, data.first, data.second, out);
}

void TextDrawer::drawCenteredText(cv::Mat* out, const std::string& msg, const cv::Point2f& img_pos,
                                  const cv::Scalar& color, int thickness, double font_scale, int font_face)
{
  int unused_baseline;
  cv::Size text_size = cv::getTextSize(msg, font_face, font_scale, thickness, &unused_baseline);
  cv::Point text_pos = img_pos + cv::Point2f(-text_size.width, text_size.height) / 2;
  cv::putText(*out, msg, text_pos, font_face, font_scale, color * 0.7, thickness, cv::LINE_AA);
}

void TextDrawer::setFontScale(double new_font_scale)
{
  font_scale = new_font_scale;
}

void TextDrawer::setColor(const cv::Scalar& new_color)
{
  color = new_color;
}

void TextDrawer::setImgOffset(const cv::Point2f& new_offset)
{
  img_offset = new_offset;
}

Json::Value TextDrawer::toJson() const
{
  Json::Value v = Drawer::toJson();
  v["img_offset_x"] = img_offset.x;
  v["img_offset_y"] = img_offset.y;
  v["font_scale"] = font_scale;
  v["font_thickness"] = font_thickness;
  v["color"] = val2Json(color);
  return v;
}

void TextDrawer::fromJson(const Json::Value& v)
{
  tryReadVal(v, "img_offset_x", &img_offset.x);
  tryReadVal(v, "img_offset_y", &img_offset.y);
  tryReadVal(v, "font_scale", &font_scale);
  tryReadVal(v, "font_thickness", &font_thickness);
  tryReadVal(v, "color", &color);
}

}  // namespace hl_monitoring
