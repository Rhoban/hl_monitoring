#include <hl_monitoring/drawers/pose_drawer.h>

#include <hl_communication/utils.h>
#include <hl_monitoring/utils.h>

#include <iostream>

#include <cmath>

#include <opencv2/imgproc.hpp>

using namespace hl_communication;

namespace hl_monitoring
{
PoseDrawer::PoseDrawer() : circle_radius(15.0), thickness(3.0), color(0, 0, 0)
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

  if (valid_pos)
  {
    if (pos.uncertainty_size() == 3)
    {
      // overlay for opacity
      cv::Mat overlay;
      overlay = out->clone();

      float data[4] = { pos.uncertainty(0), pos.uncertainty(1), pos.uncertainty(1), pos.uncertainty(2) };
      cv::Mat covMat = cv::Mat(2, 2, CV_32F, data);

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

      std::vector<cv::Point> ellipsePoints;
      double maxDistAxe = 0;
      double minDistAxe = 0;

      int nbPoint = 36;
      for (int i = 0; i < nbPoint; i++)
      {
        float x, y;
        x = halfmajoraxissize * cos(360 / nbPoint * i) * cos(angle) -
            halfminoraxissize * sin(360 / nbPoint * i) * sin(angle) + pos.x();
        y = halfmajoraxissize * cos(360 / nbPoint * i) * sin(angle) +
            halfminoraxissize * sin(360 / nbPoint * i) * cos(angle) + pos.y();

        cv::Point2f img_pos_ellipse;
        cv::Point3f ellipse_pos(x, y, 0);
        bool valid_pos_ellipse = converter(ellipse_pos, &img_pos_ellipse);

        if (valid_pos_ellipse)
        {
          ellipsePoints.push_back(img_pos_ellipse);

          // finding major and minor axes for our ellipse after projection on the image
          if (cv::norm(img_pos_ellipse - img_pos) > maxDistAxe)
            maxDistAxe = cv::norm(img_pos_ellipse - img_pos);
          if (cv::norm(img_pos_ellipse - img_pos) < minDistAxe || minDistAxe == 0)
            minDistAxe = cv::norm(img_pos_ellipse - img_pos);
        }
      }

      cv::RotatedRect ellipse = fitEllipse(ellipsePoints);
      cv::ellipse(overlay, ellipse, color, CV_FILLED, cv::LINE_AA);

      if (pose.has_dir() && pose.dir().has_std_dev())
      {
        double dir_rad = pose.dir().mean();
        if (dir_rad < 0)
          dir_rad += M_PI * 2;
        double dir_deg = 180 * dir_rad / M_PI;

        double std_dev_rad = pose.dir().std_dev();
        double std_dev_deg = 180 * std_dev_rad / M_PI;

        double offset = intervalConfianceRatio * std_dev_deg;

        cv::ellipse(overlay, img_pos, cv::Size2f(maxDistAxe, minDistAxe), angle, -dir_deg - offset, -dir_deg + offset,
                    color * 1.5, CV_FILLED, cv::LINE_AA);
      }

      double max_opacity = 0.9;
      double min_opacity = 0.05;

      double opacity =
          (max_opacity - min_opacity) * exp(-5 * eigenvalues.at<float>(0) * eigenvalues.at<float>(1)) + min_opacity;

      cv::addWeighted(overlay, opacity, *out, 1 - opacity, 0, *out);
    }
    else
    {
      cv::circle(*out, img_pos, circle_radius, color, thickness, cv::LINE_AA);

      if (pose.has_dir())
      {
        // Tagging dir
        double dir_rad = pose.dir().mean();
        double dist = 0.1;
        cv::Point3f field_arrow_end(pos.x() + cos(dir_rad) * dist, pos.y() + sin(dir_rad) * dist, 0.0);
        cv::Point2f img_end;
        bool valid_end = converter(field_arrow_end, &img_end);
        if (valid_end && pose.has_dir())
        {
          cv::Point2f img_dir = img_end - img_pos;
          float img_length = std::sqrt(img_dir.x * img_dir.x + img_dir.y * img_dir.y);
          cv::Point2f img_arrow_end = img_pos + circle_radius * img_dir / img_length;
          cv::line(*out, img_pos, img_arrow_end, color * 1.5, thickness, cv::LINE_AA);
        }
      }
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
