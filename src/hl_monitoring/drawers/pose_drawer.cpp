#include <hl_monitoring/drawers/pose_drawer.h>

#include <hl_communication/utils.h>
#include <hl_monitoring/drawers/geometry.h>

#include <iostream>

#include <cmath>

#include <opencv2/imgproc.hpp>

using namespace hl_communication;

namespace hl_monitoring
{
PoseDrawer::PoseDrawer() : circle_radius(0.1), thickness(2.0), color(0, 0, 0)
{
}

PoseDrawer::~PoseDrawer()
{
}

void PoseDrawer::draw(FieldToImgConverter converter, const hl_communication::PoseDistribution& pose, cv::Mat* out)
{
  if (!pose.has_position())
  {
    return;
  }
  // Tagging pos
  const PositionDistribution& pos = pose.position();
  cv::Point3f field_pos(pos.x(), pos.y(), 0.0);
  cv::Point2f img_pos;
  bool valid_pos = converter(field_pos, &img_pos);

  if (!valid_pos)
    return;

  cv::Mat covMat;
  bool has_covariance = exportUncertainty(pos, &covMat);
  if (has_covariance)
  {
    // TODO: refactor
    // overlay for opacity
    cv::Mat overlay;
    overlay = out->clone();

    cv::Mat eigenvalues, eigenvectors;
    cv::eigen(covMat, eigenvalues, eigenvectors);

    // Calculate the angle between the largest eigenvector and the x-axis
    double angle = atan2(eigenvectors.at<float>(0, 1), eigenvectors.at<float>(0, 0));

    // Shift the angle to the [0, 2pi] interval instead of [-pi, pi]
    if (angle < 0)
      angle += M_PI * 2;

    // 95% interf=val confiance
    float intervalConfianceRatio = 1.96;

    // Calculate the size of the minor and major axes
    float halfmajoraxissize = intervalConfianceRatio * sqrt(eigenvalues.at<float>(0));
    float halfminoraxissize = intervalConfianceRatio * sqrt(eigenvalues.at<float>(1));
    std::pair<float, float> axes(halfmajoraxissize, halfminoraxissize);
    int nbPoint = 100;

    std::vector<cv::Point> ellipsePoints;

    getEllipsePoint(converter, field_pos, nbPoint, angle, axes, 0, 2 * M_PI, &ellipsePoints);

    cv::RotatedRect ellipse = fitEllipse(ellipsePoints);
    cv::ellipse(overlay, ellipse, color, cv::FILLED, cv::LINE_AA);

    if (pose.has_dir() && pose.dir().has_std_dev())
    {
      double dir_rad = pose.dir().mean();
      double std_dev_rad = pose.dir().std_dev();

      if (std_dev_rad < M_PI / 2)
      {
        double offset = intervalConfianceRatio * std_dev_rad;
        float cone_dir_min = dir_rad - offset;
        float cone_dir_max = dir_rad + offset;

        std::vector<cv::Point> dirPoints;
        getEllipsePoint(converter, field_pos, nbPoint, angle, axes, cone_dir_min, cone_dir_max, &dirPoints);

        dirPoints.push_back(img_pos);

        cv::fillConvexPoly(overlay, dirPoints, color * 1.5, cv::LINE_AA);
      }
    }

    double max_opacity = 0.9;
    double min_opacity = 0.05;

    double opacity =
        (max_opacity - min_opacity) * exp(-5 * eigenvalues.at<float>(0) * eigenvalues.at<float>(1)) + min_opacity;

    cv::addWeighted(overlay, opacity, *out, 1 - opacity, 0, *out);
  }
  else
  {
    cv::Point3f pos_in_field(pos.x(), pos.y(), 0);
    drawGroundCircle(out, converter, cv::Point2f(pos.x(), pos.y()), circle_radius, color, thickness);
    if (pose.has_dir())
    {
      // Tagging dir
      double dir_rad = pose.dir().mean();
      cv::Point3f field_arrow_end(pos.x() + cos(dir_rad) * circle_radius, pos.y() + sin(dir_rad) * circle_radius, 0.0);
      cv::Point2f img_end;
      bool valid_end = converter(field_arrow_end, &img_end);
      if (valid_end)
        cv::line(*out, img_pos, img_end, color, thickness, cv::LINE_AA);
    }
    else
    {
      // Drawing a projected cross
      for (double angle : { M_PI / 4, 3 * M_PI / 4 })
      {
        cv::Point3f offset = cv::Point3f(cos(angle), sin(angle), 0) * circle_radius;
        cv::Point3f start_field = pos_in_field - offset;
        cv::Point3f end_field = pos_in_field + offset;
        cv::Point2f img_start, img_end;
        bool success = converter(start_field, &img_start) && converter(end_field, &img_end);
        if (success)
          cv::line(*out, img_start, img_end, color, thickness, cv::LINE_AA);
      }
    }
  }
}

void PoseDrawer::getEllipsePoint(FieldToImgConverter converter, cv::Point3f field_pos, int nbPoints,
                                 double angle_ellipse, std::pair<float, float> axes, double start_angle,
                                 double end_angle, std::vector<cv::Point>* ellipsePoints)
{
  for (float i = start_angle - angle_ellipse; i < end_angle - angle_ellipse; i += (M_PI * 2) / nbPoints)
  {
    float x, y;
    x = axes.first * cos(i) * cos(angle_ellipse) - axes.second * sin(i) * sin(angle_ellipse) + field_pos.x;
    y = axes.first * cos(i) * sin(angle_ellipse) + axes.second * sin(i) * cos(angle_ellipse) + field_pos.y;

    cv::Point2f img_pos_ellipse;
    cv::Point3f ellipse_pos(x, y, 0);
    bool valid_pos_ellipse = converter(ellipse_pos, &img_pos_ellipse);

    if (valid_pos_ellipse)
    {
      ellipsePoints->push_back(img_pos_ellipse);
    }
  }
}

void PoseDrawer::setColor(const cv::Scalar& new_color)
{
  color = new_color;
}

void PoseDrawer::setRadius(double new_radius)
{
  circle_radius = new_radius;
}

Json::Value PoseDrawer::toJson() const
{
  Json::Value v = Drawer::toJson();
  v["circle_radius"] = circle_radius;
  v["thickness"] = thickness;
  v["color"] = val2Json(color);
  return v;
}

void PoseDrawer::fromJson(const Json::Value& v)
{
  tryReadVal(v, "circle_radius", &circle_radius);
  tryReadVal(v, "thickness", &thickness);
  tryReadVal(v, "color", &color);
}

}  // namespace hl_monitoring
