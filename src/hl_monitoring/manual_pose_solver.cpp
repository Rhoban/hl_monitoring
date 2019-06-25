#include <hl_monitoring/manual_pose_solver.h>

#include <hl_communication/utils.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>

using namespace hl_communication;

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

void ManualPoseSolver::exportMatches(std::vector<hl_communication::Match2D3DMsg>* matches)
{
  matches->clear();
  for (const auto& entry : points_in_img)
  {
    int point_idx = entry.first;
    const cv::Point2f& img_point = entry.second;
    Match2D3DMsg msg;
    cvToProtobuf(img_point, msg.mutable_img_pos());
    cvToProtobuf(points_in_world[point_idx], msg.mutable_obj_pos());
    matches->push_back(msg);
  }
}

bool ManualPoseSolver::solvePose(const std::vector<cv::Point2f>& img_pos, const std::vector<cv::Point3f>& obj_pos,
                                 const cv::Mat& camera_matrix, const cv::Mat& distortion_coefficients, cv::Mat* rvec,
                                 cv::Mat* tvec)
{
  if (img_pos.size() < 4)
    return false;
  cv::solvePnP(obj_pos, img_pos, camera_matrix, distortion_coefficients, *rvec, *tvec);
  return true;
}

bool ManualPoseSolver::solvePose(const std::vector<Match2D3DMsg>& matches, const IntrinsicParameters& camera_parameters,
                                 Pose3D* pose)
{
  std::vector<cv::Point2f> img_pos;
  std::vector<cv::Point3f> obj_pos;
  protobufToCV(matches, &img_pos, &obj_pos);
  cv::Mat camera_matrix, distortion_coefficients, rvec, tvec;
  cv::Size img_size;
  intrinsicToCV(camera_parameters, &camera_matrix, &distortion_coefficients, &img_size);
  if (solvePose(img_pos, obj_pos, camera_matrix, distortion_coefficients, &rvec, &tvec))
  {
    cvToPose3D(rvec, tvec, pose);
    return true;
  }
  return false;
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
    cv::Mat drawing_img = display_img.clone();
    cv::Scalar drawing_color(255, 0, 255);
    for (const auto& entry : points_in_img)
    {
      cv::drawMarker(drawing_img, entry.second, drawing_color, cv::MARKER_TILTED_CROSS, 10, 2, cv::LINE_AA);
    }
    if (points_in_img.size() >= 4)
    {
      field.tagLines(camera_matrix, distortion_coefficients, rvec, tvec, &drawing_img, drawing_color, 2.0, 30);
    }
    double drawing_alpha = 0.3;
    cv::addWeighted(drawing_img, drawing_alpha, display_img, 1 - drawing_alpha, 0, display_img);

    int font = cv::FONT_HERSHEY_PLAIN;
    float text_scale = 1.5;
    int text_thickness = 1;
    cv::Scalar text_color = cv::Scalar(255, 0, 255);
    int line_type = cv::LINE_AA;
    cv::Point text_pos(0, display_img.rows - 20);
    if (point_index >= (int)points_in_world.size())
    {
      cv::putText(display_img, "All points have been tagged", text_pos, font, text_scale, text_color, text_thickness,
                  line_type);
    }
    else
    {
      std::ostringstream line;
      line << "Click on: " << points_names[point_index] << " at " << points_in_world[point_index] << ")";
      cv::putText(display_img, line.str(), text_pos, font, text_scale, text_color, text_thickness, line_type);
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

bool ManualPoseSolver::solve(Pose3D* pose, std::vector<hl_communication::Match2D3DMsg>* matches)
{
  cv::Mat loc_rvec, loc_tvec;
  bool success = solve(&loc_rvec, &loc_tvec);
  if (matches != nullptr)
  {
    exportMatches(matches);
  }
  if (success)
  {
    cvToPose3D(loc_rvec, loc_tvec, pose);
    return true;
  }
  return false;
}

}  // namespace hl_monitoring
