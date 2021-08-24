#include "hl_monitoring/field.h"

#include <hl_communication/utils.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <fstream>
#include <iostream>

using namespace hl_communication;

namespace hl_monitoring
{
std::vector<Field::POIType> Field::poi_type_values = { Field::POIType::ArenaCorner, Field::POIType::LineCorner,
                                                       Field::POIType::T,           Field::POIType::X,
                                                       Field::POIType::Center,      Field::POIType::PenaltyMark,
                                                       Field::POIType::PostBase,    Field::POIType::Unknown };

hl_monitoring::Field::POIType Field::string2POIType(const std::string& str)
{
  static std::map<std::string, POIType> types_by_str;
  if (types_by_str.size() == 0)
  {
    for (Field::POIType type : poi_type_values)
    {
      types_by_str[poiType2String(type)] = type;
    }
  }
  try
  {
    return types_by_str.at(str);
  }
  catch (const std::out_of_range& exc)
  {
    throw std::out_of_range(HL_DEBUG + " cannont convert str'" + str + "' to poi type");
  }
}

std::string Field::poiType2String(hl_monitoring::Field::POIType type)
{
  switch (type)
  {
    case POIType::ArenaCorner:
      return "ArenaCorner";
    case POIType::LineCorner:
      return "LineCorner";
    case POIType::T:
      return "T";
    case POIType::X:
      return "X";
    case POIType::Center:
      return "Center";
    case POIType::PenaltyMark:
      return "PenaltyMark";
    case POIType::PostBase:
      return "PostBase";
    default:
      return "UNKOWN";
  }
}

const std::vector<Field::POIType>& Field::getPOITypeValues()
{
  return poi_type_values;
}

Field::Field()
{
  ball_radius = 0.075;
  /// Expected field sizes
  line_width = 0.05;
  center_radius = 0.75;
  border_strip_width_x = 0.70;  // Minimum value
  border_strip_width_y = 0.70;  // Minimum value
  penalty_mark_dist = 1.50;
  penalty_mark_length = 0.10;
  goal_width = 2.60;
  goal_depth = 0.60;
  goal_area_length = 1.00;
  goal_area_width = 3.00;
  field_length = 9.00;
  field_width = 6.00;
  penalty_area_length = 2.00;
  penalty_area_width = 5.00;
  updateAll();
}

Json::Value Field::toJson() const
{
  Json::Value v;
  v["ball_radius"] = ball_radius;
  v["line_width"] = line_width;
  v["center_radius"] = center_radius;
  v["border_strip_width_x"] = border_strip_width_x;
  v["border_strip_width_y"] = border_strip_width_y;
  v["penalty_mark_dist"] = penalty_mark_dist;
  v["penalty_mark_length"] = penalty_mark_length;
  v["goal_width"] = goal_width;
  v["goal_depth"] = goal_depth;
  v["goal_area_length"] = goal_area_length;
  v["goal_area_width"] = goal_area_width;
  v["field_length"] = field_length;
  v["field_width"] = field_width;

  if (penalty_area_length >= 0.0)
  {
    v["penalty_area_length"] = penalty_area_length;
    v["penalty_area_width"] = penalty_area_width;
  }
  return v;
}

void Field::fromJson(const Json::Value& v)
{
  readVal(v, "ball_radius", &ball_radius);
  readVal(v, "line_width", &line_width);
  readVal(v, "center_radius", &center_radius);
  readVal(v, "border_strip_width_x", &border_strip_width_x);
  readVal(v, "border_strip_width_y", &border_strip_width_y);
  readVal(v, "penalty_mark_dist", &penalty_mark_dist);
  readVal(v, "penalty_mark_length", &penalty_mark_length);
  readVal(v, "goal_width", &goal_width);
  readVal(v, "goal_depth", &goal_depth);
  readVal(v, "goal_area_length", &goal_area_length);
  readVal(v, "goal_area_width", &goal_area_width);
  readVal(v, "field_length", &field_length);
  readVal(v, "field_width", &field_width);

  penalty_area_length = -1;
  penalty_area_width = -1;
  tryReadVal(v, "penalty_area_length", &penalty_area_width);
  tryReadVal(v, "penalty_area_width", &penalty_area_length);

  updateAll();
}

void Field::loadFile(const std::string& path)
{
  std::ifstream in(path);
  if (!in.good())
  {
    throw std::runtime_error(HL_DEBUG + " failed to open file '" + path + "'");
  }
  Json::Value root;
  in >> root;
  fromJson(root);
}

bool Field::isInArena(const cv::Point2f& pos_in_field) const
{
  double half_length = getArenaLength() / 2;
  double half_width = getArenaWidth() / 2;
  return std::fabs(pos_in_field.x) < half_length && std::fabs(pos_in_field.y) < half_width;
}

const cv::Point3f& Field::getPoint(const std::string& name) const
{
  return points_of_interest.at(name);
}

const std::map<std::string, cv::Point3f>& Field::getPointsOfInterest() const
{
  return points_of_interest;
}

const std::vector<Field::Segment>& Field::getWhiteLines() const
{
  return white_lines;
}

const std::vector<Field::Segment>& Field::getArenaBorders() const
{
  return arena_borders;
}

const std::vector<Field::Segment>& Field::getGoals() const
{
  return goals;
}

const std::vector<cv::Point3f>& Field::getGoalPosts() const
{
  return goal_posts;
}

const std::vector<cv::Point3f>& Field::getPenaltyMarks() const
{
  return penalty_marks;
}

const Field::POICollection& Field::getPointsOfInterestByType() const
{
  return poi_by_type;
}

void Field::updateAll()
{
  updatePointsOfInterest();
  updateWhiteLines();
  updateArenaBorders();
  updateGoals();
  updatePenaltyMarks();
  updatePointsOfInterestByType();
}

void Field::updatePointsOfInterest()
{
  points_of_interest.clear();
  points_of_interest["center"] = cv::Point3f(0, 0, 0);
  points_of_interest["center_x+"] = cv::Point3f(0, center_radius, 0);
  points_of_interest["center_x-"] = cv::Point3f(0, -center_radius, 0);
  double ac_x = field_length / 2 + border_strip_width_x;
  double ac_y = field_width / 2 + border_strip_width_y;
  points_of_interest["arena_corner++"] = cv::Point3f(ac_x, ac_y, 0);
  points_of_interest["arena_corner+-"] = cv::Point3f(ac_x, -ac_y, 0);
  points_of_interest["arena_corner-+"] = cv::Point3f(-ac_x, ac_y, 0);
  points_of_interest["arena_corner--"] = cv::Point3f(-ac_x, -ac_y, 0);
  double fc_x = field_length / 2;
  double fc_y = field_width / 2;
  points_of_interest["field_corner++"] = cv::Point3f(fc_x, fc_y, 0);
  points_of_interest["field_corner+-"] = cv::Point3f(fc_x, -fc_y, 0);
  points_of_interest["field_corner-+"] = cv::Point3f(-fc_x, fc_y, 0);
  points_of_interest["field_corner--"] = cv::Point3f(-fc_x, -fc_y, 0);
  double gac_x = field_length / 2 - goal_area_length;
  double gac_y = goal_area_width / 2;
  points_of_interest["goal_area_corner++"] = cv::Point3f(gac_x, gac_y, 0);
  points_of_interest["goal_area_corner+-"] = cv::Point3f(gac_x, -gac_y, 0);
  points_of_interest["goal_area_corner-+"] = cv::Point3f(-gac_x, gac_y, 0);
  points_of_interest["goal_area_corner--"] = cv::Point3f(-gac_x, -gac_y, 0);
  double gat_x = field_length / 2;
  double gat_y = goal_area_width / 2;
  points_of_interest["goal_area_t++"] = cv::Point3f(gat_x, gat_y, 0);
  points_of_interest["goal_area_t+-"] = cv::Point3f(gat_x, -gat_y, 0);
  points_of_interest["goal_area_t-+"] = cv::Point3f(-gat_x, gat_y, 0);
  points_of_interest["goal_area_t--"] = cv::Point3f(-gat_x, -gat_y, 0);
  double pm_x = field_length / 2 - penalty_mark_dist;
  points_of_interest["penalty_mark+"] = cv::Point3f(pm_x, 0, 0);
  points_of_interest["penalty_mark-"] = cv::Point3f(-pm_x, 0, 0);
  double mlt_y = field_width / 2;
  points_of_interest["middle_line_t+"] = cv::Point3f(0, mlt_y, 0);
  points_of_interest["middle_line_t-"] = cv::Point3f(0, -mlt_y, 0);
  double goal_x = field_length / 2;
  double goal_y = goal_width / 2;
  points_of_interest["post_base++"] = cv::Point3f(goal_x, goal_y, 0);
  points_of_interest["post_base+-"] = cv::Point3f(goal_x, -goal_y, 0);
  points_of_interest["post_base-+"] = cv::Point3f(-goal_x, goal_y, 0);
  points_of_interest["post_base--"] = cv::Point3f(-goal_x, -goal_y, 0);

  if (penalty_area_length >= 0.0)
  {
    double pa_x = field_length / 2 - penalty_area_length;
    double pa_y = penalty_area_width / 2;
    points_of_interest["penalty_area_corner++"] = cv::Point3f(pa_x, pa_y, 0);
    points_of_interest["penalty_area_corner+-"] = cv::Point3f(pa_x, -pa_y, 0);
    points_of_interest["penalty_area_corner-+"] = cv::Point3f(-pa_x, pa_y, 0);
    points_of_interest["penalty_area_corner--"] = cv::Point3f(-pa_x, -pa_y, 0);

    double pat_x = field_length / 2;
    double pat_y = penalty_area_width / 2;
    points_of_interest["penalty_area_t++"] = cv::Point3f(pat_x, pat_y, 0);
    points_of_interest["penalty_area_t+-"] = cv::Point3f(pat_x, -pat_y, 0);
    points_of_interest["penalty_area_t-+"] = cv::Point3f(-pat_x, pat_y, 0);
    points_of_interest["penalty_area_t--"] = cv::Point3f(-pat_x, -pat_y, 0);
  }
}

void Field::updateWhiteLines()
{
  white_lines.clear();
  white_lines.push_back({ getPoint("field_corner++"), getPoint("field_corner+-") });
  white_lines.push_back({ getPoint("field_corner+-"), getPoint("field_corner--") });
  white_lines.push_back({ getPoint("field_corner--"), getPoint("field_corner-+") });
  white_lines.push_back({ getPoint("field_corner-+"), getPoint("field_corner++") });
  white_lines.push_back({ getPoint("middle_line_t+"), getPoint("middle_line_t-") });
  white_lines.push_back({ getPoint("goal_area_t++"), getPoint("goal_area_corner++") });
  white_lines.push_back({ getPoint("goal_area_corner++"), getPoint("goal_area_corner+-") });
  white_lines.push_back({ getPoint("goal_area_corner+-"), getPoint("goal_area_t+-") });
  white_lines.push_back({ getPoint("goal_area_t-+"), getPoint("goal_area_corner-+") });
  white_lines.push_back({ getPoint("goal_area_corner-+"), getPoint("goal_area_corner--") });
  white_lines.push_back({ getPoint("goal_area_corner--"), getPoint("goal_area_t--") });

  if (penalty_area_length >= 0.0)
  {
    white_lines.push_back({ getPoint("penalty_area_t++"), getPoint("penalty_area_corner++") });
    white_lines.push_back({ getPoint("penalty_area_corner++"), getPoint("penalty_area_corner+-") });
    white_lines.push_back({ getPoint("penalty_area_corner+-"), getPoint("penalty_area_t+-") });
    white_lines.push_back({ getPoint("penalty_area_t-+"), getPoint("penalty_area_corner-+") });
    white_lines.push_back({ getPoint("penalty_area_corner-+"), getPoint("penalty_area_corner--") });
    white_lines.push_back({ getPoint("penalty_area_corner--"), getPoint("penalty_area_t--") });
  }
}

void Field::updateArenaBorders()
{
  arena_borders = { { getPoint("arena_corner++"), getPoint("field_corner+-") },
                    { getPoint("arena_corner+-"), getPoint("field_corner--") },
                    { getPoint("field_corner--"), getPoint("field_corner-+") },
                    { getPoint("field_corner-+"), getPoint("field_corner++") } };
}

void Field::updateGoals()
{
  goals = { { getPoint("post_base-+"), getPoint("post_base--") },
            { getPoint("post_base++"), getPoint("post_base+-") } };
  goal_posts = { getPoint("post_base++"), getPoint("post_base+-"), getPoint("post_base-+"), getPoint("post_base--") };
}

void Field::updatePenaltyMarks()
{
  penalty_marks = { getPoint("penalty_mark+"), getPoint("penalty_mark-") };
}

void Field::updatePointsOfInterestByType()
{
  poi_by_type.clear();
  for (const auto& entry : points_of_interest)
  {
    POIType type = POIType::Unknown;
    if (entry.first.find("arena_corner") != std::string::npos)
    {
      type = POIType::ArenaCorner;
    }
    else if (entry.first.find("corner") != std::string::npos)
    {
      type = POIType::LineCorner;
    }
    else if (entry.first.find("_t") != std::string::npos)
    {
      type = POIType::T;
    }
    else if (entry.first.find("_x") != std::string::npos)
    {
      type = POIType::X;
    }
    else if (entry.first.find("penalty_mark") != std::string::npos)
    {
      type = POIType::PenaltyMark;
    }
    else if (entry.first.find("post_base") != std::string::npos)
    {
      type = POIType::PostBase;
    }
    else if (entry.first.find("center") != std::string::npos)
    {
      type = POIType::Center;
    }
    poi_by_type[type].push_back(entry.second);
  }
}

void Field::tagPointsOfInterest(const cv::Mat& camera_matrix, const cv::Mat& distortion_coeffs, const cv::Mat& rvec,
                                const cv::Mat& tvec, cv::Mat* tag_img)
{
  cv::Rect2f img_rect(cv::Point2f(), cv::Point2f(tag_img->cols, tag_img->rows));
  for (const auto& entry : poi_by_type)
  {
    const std::vector<cv::Point3f>& field_positions = entry.second;
    std::vector<cv::Point2f> img_points;
    cv::projectPoints(field_positions, rvec, tvec, camera_matrix, distortion_coeffs, img_points);
    for (size_t idx = 0; idx < field_positions.size(); idx++)
    {
      if (isPointValidForCorrection(field_positions[idx], rvec, tvec, camera_matrix, distortion_coeffs))
      {
        cv::Point3f camera_pos = hl_communication::fieldToCamera(field_positions[idx], rvec, tvec);
        if (camera_pos.z > 0 && img_rect.contains(img_points[idx]))
        {
          cv::circle(*tag_img, img_points[idx], 5, cv::Scalar(255, 0, 255), cv::FILLED);
        }
      }
    }
  }
}

void Field::tagLines(const CameraMetaInformation& camera_information, cv::Mat* tag_img, const cv::Scalar& line_color,
                     double line_thickness, int nb_segments) const
{
  if (!camera_information.has_camera_parameters() || !camera_information.has_pose())
  {
    throw std::runtime_error(HL_DEBUG + " camera_information is not fully specified");
  }
  cv::Mat camera_matrix, distortion_coefficients, rvec, tvec;
  cv::Size size;
  intrinsicToCV(camera_information.camera_parameters(), &camera_matrix, &distortion_coefficients, &size);
  pose3DToCV(camera_information.pose(), &rvec, &tvec);
  if (size.width != tag_img->cols || size.height != tag_img->rows)
  {
    std::ostringstream oss;
    oss << HL_DEBUG << " size mismatch " << size << " != " << tag_img->size;
    throw std::runtime_error(oss.str());
  }
  tagLines(camera_matrix, distortion_coefficients, rvec, tvec, tag_img, line_color, line_thickness, nb_segments);
}

void Field::tagLines(const cv::Mat& camera_matrix, const cv::Mat& distortion_coeffs, const cv::Mat& rvec,
                     const cv::Mat& tvec, cv::Mat* tag_img, const cv::Scalar& line_color, double line_thickness,
                     int nb_segments) const
{
  for (const auto& segment : getWhiteLines())
  {
    cv::Point3f object_diff = segment.second - segment.first;
    for (int i = 0; i < nb_segments; i++)
    {
      std::vector<cv::Point3f> object_points = { segment.first + i * object_diff / nb_segments,
                                                 segment.first + (i + 1) * object_diff / nb_segments };
      bool has_point_behind = false;
      for (const cv::Point3f& obj_point : object_points)
      {
        cv::Point3f point_in_camera = hl_communication::fieldToCamera(obj_point, rvec, tvec);
        if (point_in_camera.z <= 0)
        {
          has_point_behind = true;
        }
      }
      if (has_point_behind)
      {
        continue;
      }
      std::vector<cv::Point2f> img_points;
      if (not isPointValidForCorrection(object_points[0], rvec, tvec, camera_matrix, distortion_coeffs))
        continue;
      if (not isPointValidForCorrection(object_points[1], rvec, tvec, camera_matrix, distortion_coeffs))
        continue;
      cv::projectPoints(object_points, rvec, tvec, camera_matrix, distortion_coeffs, img_points);
      // When point is outside of image, screw up the drawing
      cv::Rect2f img_rect(cv::Point(), tag_img->size());
      if (img_rect.contains(img_points[0]) && img_rect.contains(img_points[1]))
      {
        cv::line(*tag_img, img_points[0], img_points[1], line_color, line_thickness, cv::LINE_AA);
      }
    }
  }
}

double Field::getArenaLength() const
{
  return field_length + 2 * border_strip_width_x;
}

double Field::getArenaWidth() const
{
  return field_width + 2 * border_strip_width_y;
}

void Field::overview(const cv::Mat* tag_img, const cv::Scalar& line_color, double line_thickness,
                     const cv::Point3f circle, int rows = 0, int cols = 0)
{
  // size of the field in meters
  float max_size_x = field_length + 2 * border_strip_width_x;
  float max_size_y = field_width + 2 * border_strip_width_x;
  float ratio = max_size_x / max_size_y;

  // width and height of the displayed field in pixels
  float width = tag_img->cols - cols;
  float height = (float)width * ratio;

  // resize factor to give space to the markers
  float resize = 0.6f;
  // factors to convert from meters to pixels
  float multiplier_y = ((float)width) / max_size_y * resize;
  float multiplier_x = ((float)height) / max_size_x * resize;
  // offsets
  float offx = cols + width / 2.0f;
  float offy = height / 2.0f;

  // generation of the lines
  for (const auto& segment : getWhiteLines())
  {
    cv::Point2f p1, p2;
    p1 = cv::Point2f(segment.first.y * multiplier_x + offx, -segment.first.x * multiplier_y + offy);
    p2 = cv::Point2f(segment.second.y * multiplier_x + offx, -segment.second.x * multiplier_y + offy);
    cv::line(*tag_img, p1, p2, line_color, line_thickness, cv::LINE_AA);
  }

  // generation of the point of interest markers
  for (const auto& entry : poi_by_type)
  {
    const std::vector<cv::Point3f>& field_positions = entry.second;
    cv::MarkerTypes marker;
    float size = (multiplier_x + multiplier_y) / 4.0;
    cv::Scalar col(255, 100, 100);
    switch (entry.first)
    {
      case ArenaCorner:
        marker = cv::MARKER_CROSS;
        break;
      case LineCorner:
        marker = cv::MARKER_SQUARE;
        break;
      case T:
        marker = cv::MARKER_CROSS;
        size *= 0.75;
        break;
      case X:
        marker = cv::MARKER_TILTED_CROSS;
        break;
      case Center:
        marker = cv::MARKER_STAR;
        break;
      case PenaltyMark:
        marker = cv::MARKER_CROSS;
        break;
      case PostBase:
        marker = cv::MARKER_DIAMOND;
        size *= 0.75;
        break;
      default:
        marker = cv::MARKER_TRIANGLE_UP;
        break;
    }

    for (const auto& position : field_positions)
    {
      cv::Point2f coord = cv::Point2f(-position.y * multiplier_x + offx, -position.x * multiplier_y + offy);
      cv::drawMarker(*tag_img, coord, col, marker, size, line_thickness);
    }
  }

  // generation of the red circle surrounding the current point of interest
  cv::Point2f coord(-circle.y * multiplier_x + offx, -circle.x * multiplier_y + offy);
  cv::circle(*tag_img, coord, (multiplier_x + multiplier_y) / 4.0, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
}
}  // namespace hl_monitoring
