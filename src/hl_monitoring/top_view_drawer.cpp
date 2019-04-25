#include <hl_monitoring/top_view_drawer.h>

#include <opencv2/imgproc.hpp>

namespace hl_monitoring
{
TopViewDrawer::TopViewDrawer() : TopViewDrawer(cv::Size(600, 400))
{
}
TopViewDrawer::TopViewDrawer(const cv::Size& img_size)
  : img_size(img_size)
  , background_color(0, 0, 0)
  , field_color(0, 180, 0)
  , lines_color(255, 255, 255)
  , goals_color(0, 0, 255)
{
}

void TopViewDrawer::setImgSize(const cv::Size& new_img_size)
{
  img_size = new_img_size;
}

cv::Mat TopViewDrawer::getImg(const Field& f) const
{
  cv::Mat result(img_size, CV_8UC3, background_color);
  drawTurf(f, &result);
  drawLines(f, &result);
  drawCenter(f, &result);
  drawPenaltyMarks(f, &result);
  drawGoals(f, &result);
  return result;
}

double TopViewDrawer::getScale(const Field& f) const
{
  double scale_x = img_size.width / f.getArenaLength();
  double scale_y = img_size.height / f.getArenaWidth();
  return std::min(scale_x, scale_y);
}

cv::Point TopViewDrawer::getImgFromField(const Field& f, const cv::Point3f& pos_in_field) const
{
  return getImgFromField(f, cv::Point2f(pos_in_field.x, pos_in_field.y));
}

cv::Point TopViewDrawer::getImgFromField(const Field& f, const cv::Point2f& pos_in_field) const
{
  cv::Point2f center(img_size.width / 2, img_size.height / 2);
  cv::Point2f img_offset = pos_in_field * getScale(f);
  img_offset.y *= -1;//Y axis is inverted on image
  return center + img_offset;
}

int TopViewDrawer::getLineWidth(const Field& f) const
{
  return (int)(f.line_width * getScale(f));
}

int TopViewDrawer::getGoalWidth(const Field& f) const
{
  return (int)(2 * f.line_width * getScale(f));
}

int TopViewDrawer::getMarkLength(const Field& f) const
{
  return (int)(f.penalty_mark_length * getScale(f));
}

void TopViewDrawer::drawTurf(const Field& f, cv::Mat* dst) const
{
  cv::Point pt1 = getImgFromField(f, f.getPoint("arena_corner--"));
  cv::Point pt2 = getImgFromField(f, f.getPoint("arena_corner++"));
  cv::rectangle(*dst, pt1, pt2, field_color, CV_FILLED);
}

void TopViewDrawer::drawLines(const Field& f, cv::Mat* dst) const
{
  for (const Field::Segment& line : f.getWhiteLines())
  {
    cv::Point pt1 = getImgFromField(f, line.first);
    cv::Point pt2 = getImgFromField(f, line.second);
    cv::line(*dst, pt1, pt2, lines_color, getLineWidth(f));
  }
}

void TopViewDrawer::drawPenaltyMarks(const Field& f, cv::Mat* dst) const
{
  for (const cv::Point3f& p : f.getPenaltyMarks())
  {
    drawMark(f, p, dst);
  }
}

void TopViewDrawer::drawGoals(const Field& f, cv::Mat* dst) const
{
  for (const Field::Segment& goal : f.getGoals())
  {
    cv::Point pt1 = getImgFromField(f, goal.first);
    cv::Point pt2 = getImgFromField(f, goal.second);
    cv::line(*dst, pt1, pt2, goals_color, getGoalWidth(f));
  }
}

void TopViewDrawer::drawCenter(const Field& f, cv::Mat* dst) const
{
  cv::Point3f center_in_field(0,0,0);
  drawMark(f, center_in_field, dst);
  cv::Point center_in_img = getImgFromField(f, center_in_field);
  int radius_in_img = (int)(f.center_radius * getScale(f));
  cv::circle(*dst, center_in_img, radius_in_img, lines_color, getLineWidth(f));
}

void TopViewDrawer::drawMark(const Field& f, const cv::Point3f& mark_pos_in_field, cv::Mat* dst) const
{
  int half_length = getMarkLength(f) / 2;
  cv::Point pos_in_img = getImgFromField(f, mark_pos_in_field);
  for (const cv::Point& offset : { cv::Point(half_length, 0), cv::Point(0, half_length) })
  {
    cv::line(*dst, pos_in_img - offset, pos_in_img + offset, lines_color, getLineWidth(f));
  }
}

}  // namespace hl_monitoring
