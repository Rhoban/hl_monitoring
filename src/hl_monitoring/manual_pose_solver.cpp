#include <hl_monitoring/manual_pose_solver.h>

#include <hl_communication/utils.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>

using namespace hl_communication;

namespace hl_monitoring
{
cv::SolvePnPMethod ManualPoseSolver::pnp_method;
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
  cv::setMouseCallback(
      "manual_pose_solver",
      [](int event, int x, int y, int, void* param) -> void {
        ((ManualPoseSolver*)param)->onClick(event, x, y, param);
      },
      this);
}

ManualPoseSolver::~ManualPoseSolver()
{
  cv::destroyWindow("manual_pose_solver");
}
void ManualPoseSolver::setPnpMethod(int pnp_method_id)
{ /*
 solve_methods :
   SOLVEPNP_ITERATIVE Iterative method is based on a Levenberg-Marquardt optimization. In this case the function
 finds such a pose that minimizes reprojection error, that is the sum of squared distances between the observed
 projections imagePoints and the projected (using projectPoints ) objectPoints .
 0/default -> cv::SOLVEPNP_ITERATIVE
 1 -> cv::SOLVEPNP_P3P
 2 -> cv::SOLVEPNP_AP3P
 3 -> cv::SOLVEPNP_EPNP
 4 -> cv::SOLVEPNP_DLS
 5-> cv::SOLVEPNP_UPNP
 6 -> cv::SOLVEPNP_IPPE
 7 -> cv::SOLVEPNP_IPPE_SQUARE
*/

  switch (pnp_method_id)
  {
    case 1:
      ManualPoseSolver::pnp_method = cv::SOLVEPNP_P3P;
      break;
    case 2:
      ManualPoseSolver::pnp_method = cv::SOLVEPNP_AP3P;
      break;
    case 3:
      ManualPoseSolver::pnp_method = cv::SOLVEPNP_EPNP;
      break;
    case 4:
      ManualPoseSolver::pnp_method = cv::SOLVEPNP_DLS;
      break;
    case 5:
      ManualPoseSolver::pnp_method = cv::SOLVEPNP_UPNP;
      break;
    case 6:
      ManualPoseSolver::pnp_method = cv::SOLVEPNP_IPPE;
      break;
    case 7:
      ManualPoseSolver::pnp_method = cv::SOLVEPNP_IPPE_SQUARE;
      break;
    default:
      ManualPoseSolver::pnp_method = cv::SOLVEPNP_ITERATIVE;
  }
}
void ManualPoseSolver::updatePose()
{
  std::vector<cv::Point3f> object_points;
  std::vector<cv::Point2f> img_points;
  for (const auto& entry : points_in_img)
  {
    cv::Point3f obj_point = points_in_world[entry.first];
    object_points.push_back(obj_point);
    img_points.push_back(entry.second);
  }
  cv::Mat rvec_copy = rvec.clone();
  cv::Mat tvec_copy = tvec.clone();
  // The update of tvec and rvec is only performed is the 'solvePose' operation is successful
  if (solvePose(img_points, object_points, camera_matrix, distortion_coefficients, &rvec_copy, &tvec_copy))
  {
    rvec = rvec_copy;
    tvec = tvec_copy;
  }
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
                                 cv::Mat* tvec, bool useExtrinsicGuess)
{
  if (img_pos.size() < 4)
    return false;
  if ((ManualPoseSolver::pnp_method == cv::SOLVEPNP_P3P) & (img_pos.size() != 4))
    return false;
  std::vector<cv::Mat> rvecs, tvecs;
  if ((*rvec).empty() || (*tvec).empty())
    useExtrinsicGuess = false;
  cv::solvePnPGeneric(obj_pos, img_pos, camera_matrix, distortion_coefficients, rvecs, tvecs, useExtrinsicGuess,
                      ManualPoseSolver::pnp_method, *rvec, *tvec);

  int best_id = 0;
  float min_tvec_error = 999.0f;
  cv::Mat current_tvec;

  if (!(tvec->empty()))
  {
    for (int i = 0; i < tvecs.size(); i++)
    {
      current_tvec = tvecs[i];

      float tvec_error = 0;

      for (int j = 0; j < current_tvec.size().height; j++)
      {
        tvec_error +=
            (tvec->at<double>(j) - current_tvec.at<double>(j)) * (tvec->at<double>(j) - current_tvec.at<double>(j));
      }
      if (tvec_error < min_tvec_error)
      {
        best_id = i;
        min_tvec_error = tvec_error;
      }
    }
  }

  *rvec = rvecs[best_id];
  *tvec = tvecs[best_id];
  std::cout << *tvec << *rvec << std::endl;
  // cv::solvePnP(obj_pos, img_pos, camera_matrix, distortion_coefficients, *rvec, *tvec);
  return true;
}

bool ManualPoseSolver::solvePose(const std::vector<Match2D3DMsg>& matches, const IntrinsicParameters& camera_parameters,
                                 Pose3D* pose, bool useExtrinsicGuess)
{
  std::vector<cv::Point2f> img_pos;
  std::vector<cv::Point3f> obj_pos;
  protobufToCV(matches, &img_pos, &obj_pos);
  cv::Mat camera_matrix, distortion_coefficients, rvec, tvec;
  cv::Size img_size;
  intrinsicToCV(camera_parameters, &camera_matrix, &distortion_coefficients, &img_size);
  if (solvePose(img_pos, obj_pos, camera_matrix, distortion_coefficients, &rvec, &tvec, useExtrinsicGuess))
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
bool ManualPoseSolver::solve(cv::Mat* rvec_out, cv::Mat* tvec_out, bool has_guess)
{
  bool exit = false;
  const int space_for_overview = 200;

  while (!exit)
  {
    cv::Mat display_img = calibration_img.clone();
    cv::Mat drawing_img = display_img.clone();
    cv::Scalar drawing_color(255, 0, 255);
    cv::Scalar guess_color(0, 0, 0);

    if (has_guess)
    {
      field.tagLines(camera_matrix, distortion_coefficients, *rvec_out, *tvec_out, &drawing_img, guess_color, 2.0, 30);
      tryDrawHelper(*rvec_out, *tvec_out, guess_color, &drawing_img);
    }
    for (const auto& entry : points_in_img)
    {
      cv::drawMarker(drawing_img, entry.second, drawing_color, cv::MARKER_TILTED_CROSS, 10, 2, cv::LINE_AA);
    }
    if (points_in_img.size() >= 4)
    {
      field.tagLines(camera_matrix, distortion_coefficients, rvec, tvec, &drawing_img, drawing_color, 2.0, 30);
      tryDrawHelper(rvec, tvec, drawing_color, &drawing_img);
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

    cv::Mat result = cv::Mat(display_img.rows, display_img.cols + space_for_overview, CV_8UC3, 0.0);

    display_img.copyTo(result(cv::Rect(0, 0, display_img.cols, display_img.rows)));
    field.overview(&result, drawing_color, 1.0, points_in_world[point_index], display_img.rows, display_img.cols);

    cv::imshow("manual_pose_solver", result);

    char key = cv::waitKey(20);
    if (key == (char)255)
      continue;
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
      case 'h':
        printHelp();
        break;
      default:
        std::cout << "unknown key pressed" << std::endl;
        printHelp();
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
  bool has_guess = false;
  if (pose->rotation_size() > 0 && pose->translation_size() > 0)
  {
    std::cout << "using guess" << std::endl;
    pose3DToCV(*pose, &loc_rvec, &loc_tvec);
    has_guess = true;
  }
  bool success = solve(&loc_rvec, &loc_tvec, has_guess);
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

void ManualPoseSolver::tryDrawHelper(const cv::Mat& rvec, const cv::Mat& tvec, const cv::Scalar& color, cv::Mat* img)
{
  if (point_index >= (int)points_in_world.size())
    return;
  cv::Point3f world_pos = points_in_world[point_index];
  if (fieldToCamera(world_pos, rvec, tvec).z >= 0)
  {
    std::vector<cv::Point3f> object_points = { world_pos };
    std::vector<cv::Point2f> img_points;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix, distortion_coefficients, img_points);
    cv::circle(*img, img_points[0], 10, color, 3, cv::LINE_AA);
  }
}

void ManualPoseSolver::printHelp() const
{
  std::cout << "-------------------------" << std::endl
            << "| ManualPoseSolver Help |" << std::endl
            << "-------------------------" << std::endl;
  std::cout << "Some elements are added to the initial image based on the following code:" << std::endl
            << "- Color: black shows initial guess and magenta shows current estimate according to specified points"
            << std::endl
            << "         (only applicable if at least 4 points are provided)" << std::endl
            << "- Lines represents the markings of the fields" << std::endl
            << "- Circles indicates the point which is currently requested (only drawn if inside the image)"
            << std::endl;
  std::cout << "Key actions are as follows:" << std::endl;
  std::cout << "'q': Quit the active process and ends with the points specified until now." << std::endl;
  std::cout << "'i': Ignore the point currently indicated by text." << std::endl;
  std::cout << "'c': Go back to previous point proposed and remove the correspondance if it was added by the user."
            << std::endl;
  std::cout << "'h': print this help." << std::endl;
}

}  // namespace hl_monitoring
