#include <hl_monitoring/drawers/geometry.h>

#include <opencv2/imgproc.hpp>

namespace hl_monitoring
{
// TODO: refactor code here -> duplication

void drawGroundDisk(cv::Mat* out, FieldToImgConverter converter, const cv::Point2f& ground_center, double radius,
                    const cv::Scalar& color, double alpha, double arc_step)
{
  std::vector<cv::Point> img_points;
  for (double angle = 0; angle < 2 * M_PI; angle += arc_step)
  {
    cv::Point3f obj_pos(ground_center.x + radius * cos(angle), ground_center.y + radius * sin(angle), 0);
    cv::Point2f img_pos;
    if (converter(obj_pos, &img_pos))
      img_points.push_back(img_pos);
  }
  if (img_points.size() > 0)
  {
    if (alpha >= 1.0)
    {
      cv::fillConvexPoly(*out, img_points, color, cv::LINE_AA);
    }
    else
    {
      cv::Mat cpy = out->clone();
      cv::fillConvexPoly(cpy, img_points, color, cv::LINE_AA);
      cv::addWeighted(cpy, alpha, *out, 1 - alpha, 0, *out);
    }
  }
}

void drawGroundCircle(cv::Mat* out, FieldToImgConverter converter, const cv::Point2f& ground_center, double radius,
                      const cv::Scalar& color, double thickness, double arc_step)
{
  std::vector<cv::Point> img_points;
  for (double angle = 0; angle < 2 * M_PI; angle += arc_step)
  {
    cv::Point3f obj_pos(ground_center.x + radius * cos(angle), ground_center.y + radius * sin(angle), 0);
    cv::Point2f img_pos;
    if (converter(obj_pos, &img_pos))
      img_points.push_back(img_pos);
  }
  if (img_points.size() > 0)
  {
    cv::polylines(*out, img_points, true, color, thickness, cv::LINE_AA);
  }
}

}  // namespace hl_monitoring
