#include <hl_monitoring/manual_pose_solver.h>

#include <hl_monitoring/utils.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>

namespace hl_monitoring
{
ManualPoseSolver::ManualPoseSolver(const cv::Mat& img, const IntrinsicParameters& camera_parameters, const Field& field)
  : field(field), calibration_img(img.clone()), point_index(0)
{
  intrinsicToCV(camera_parameters, &camera_matrix, &distortion_coefficients, &img_size);
  for (const auto& entry : field.getPointsOfInterest())
  {
    points_names.push_back(entry.first);
    points_in_world.push_back(entry.second);
  }

  cv::namedWindow("manual_pose_solver", cv::WINDOW_NORMAL);
  cv::setMouseCallback("manual_pose_solver",
                       [](int event, int x, int y, int, void* param) -> void {
                         ((ManualPoseSolver*)param)->onClick(event, x, y, param);
                       },
                       this);
}

ManualPoseSolver::~ManualPoseSolver()
{
  cv::destroyWindow("manual_pose_solver");
}

void ManualPoseSolver::updatePose()
{
  if (points_in_img.size() < 4)
  {
    return;
  }
  std::vector<cv::Point3f> object_points;
  std::vector<cv::Point2f> img_points;
  for (const auto& entry : points_in_img)
  {
    cv::Point3f obj_point = points_in_world[entry.first];
    object_points.push_back(obj_point);
    img_points.push_back(entry.second);
  }
  cv::solvePnP(object_points, img_points, camera_matrix, distortion_coefficients, rvec, tvec);
}

void ManualPoseSolver::onClick(int event, int x, int y, void* param)
{
  if (event != cv::EVENT_LBUTTONDOWN)
  {
    return;
  }
  if (point_index >= (int)points_in_world.size())
  {
    return;
  }
  points_in_img[point_index] = cv::Point(x, y);
  updatePose();
  point_index++;
}

// Main loop
bool ManualPoseSolver::solve(cv::Mat* rvec_out, cv::Mat* tvec_out)
{
  bool exit = false;
  while (!exit)
  {
    cv::Mat display_img = calibration_img.clone();
    for (const auto& entry : points_in_img)
    {
      cv::circle(display_img, entry.second, 5, cv::Scalar(0, 0, 0), -1);
    }
    if (points_in_img.size() >= 4)
    {
      field.tagLines(camera_matrix, distortion_coefficients, rvec, tvec, &display_img, cv::Scalar(0, 0, 0), 2.0, 30);
    }

    cv::Scalar text_color = cv::Scalar(255, 0, 255);
    if (point_index >= (int)points_in_world.size())
    {
      cv::putText(display_img, "All points have been tagged", cv::Point(0, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                  text_color, 2);
    }
    else
    {
      std::string line1 = "Click on: " + points_names[point_index];
      std::ostringstream line2;
      line2 << "(world pos: " << points_in_world[point_index] << ")";
      cv::putText(display_img, line1, cv::Point(0, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, text_color, 2);
      cv::putText(display_img, line2.str(), cv::Point(0, 70), cv::FONT_HERSHEY_SIMPLEX, 1.0, text_color, 2);
    }

    cv::imshow("manual_pose_solver", display_img);
    char key = cv::waitKey(20);
    switch (key)
    {
      case 'q':  // Quit
        exit = true;
        break;
      case 'i':  // Ignore point
        point_index++;
        break;
      case 'c':  // Cancel last point
        if (point_index > 0)
        {
          point_index--;
          points_in_img.erase(point_index);
          updatePose();
        }
        break;
        // TODO: add help on 'h'
    }
  }
  if (points_in_img.size() >= 4)
  {
    *rvec_out = rvec;
    *tvec_out = tvec;
    return true;
  }
  return false;
}

bool ManualPoseSolver::solve(Pose3D* pose)
{
  cv::Mat loc_rvec, loc_tvec;
  if (solve(&loc_rvec, &loc_tvec))
  {
    cvToPose3D(loc_rvec, loc_tvec, pose);
    return true;
  }
  return false;
}

}  // namespace hl_monitoring