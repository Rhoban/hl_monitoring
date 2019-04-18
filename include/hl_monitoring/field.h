#pragma once

#include <hl_monitoring/camera.pb.h>

#include <opencv2/core.hpp>
#include <json/json.h>

namespace hl_monitoring
{
/**
 * Contains all the information about the size of the field, also allows to
 * retrieve points of interest
 *
 * The field is composed of:
 * - Penalty marks
 * - Field area (inside the external white lines)
 * - Arena (the whole turf surface)
 *
 * Referential is as follows:
 * - Zero: Center of field at ground level
 * - X-axis goes from center of the field to the center of the post at the right side of the team area
 * - Y-axis goes from center toward side line opposite to team area
 * - Z-axis points toward the roof
 *
 *                         y
 *  |----------------------------------------------|
 *  |                      |                       |
 *  |                      |                       |
 *  |                      |                       |
 *  |                      |                       |
 *  |                      0--->x                  |
 *  |                      |                       |
 *  |                      |                       |
 *  |                      |                       |
 *  |                      |                       |
 *  |----------------------------------------------|
 *
 *         |--------------------------------|
 *         |                                |
 *         |           Team area            |
 *         |                                |
 *         |--------------------------------|
 *
 * - All field informations are in meters
 */
class Field
{
public:
  typedef std::pair<cv::Point3f, cv::Point3f> Segment;

  enum POIType
  {
    ArenaCorner,
    LineCorner,
    T,
    X,
    Center,
    PenaltyMark,
    PostBase,
    Unknown
  };

  static std::string poiType2String(hl_monitoring::Field::POIType type);

  Field();

  Json::Value toJson() const;
  void fromJson(const Json::Value& v);
  void loadFile(const std::string& path);

  /**
   * Return true if the given position is inside the arena
   */
  bool isInArena(const cv::Point2f& pos_in_field) const;

  /**
   * Throws an out_of_range exception on invalid name
   */
  const cv::Point3f& getPoint(const std::string& name) const;

  const std::map<std::string, cv::Point3f>& getPointsOfInterest() const;
  const std::vector<Segment>& getArenaBorders() const;
  const std::vector<Segment>& getWhiteLines() const;
  const std::vector<Segment>& getGoals() const;
  const std::vector<cv::Point3f>& getGoalPosts() const;
  const std::vector<cv::Point3f>& getPenaltyMarks() const;

  const std::map<POIType, std::vector<cv::Point3f>>& getPointsOfInterestByType() const;

  void tagLines(const CameraMetaInformation& camera_information, cv::Mat* tag_img, const cv::Scalar& line_color,
                double line_thickness, int nb_segments = 1);
  void tagLines(const cv::Mat& camera_matrix, const cv::Mat& distortion_coeffs, const cv::Mat& rvec,
                const cv::Mat& tvec, cv::Mat* tag_img, const cv::Scalar& line_color, double line_thickness,
                int nb_segments = 1);

  double getArenaLength() const;
  double getArenaWidth() const;

  /**
   * Radius of the ball [m]
   */
  double ball_radius;

  /**
   *  The width of white lines [m]
   */
  double line_width;

  /**
   *  Radius of the central circle (lines included) [m]
   */
  double center_radius;

  /**
   *  Distance from fieldBorder to arenaBorder along x_axis (line excluded) [m]
   */
  double border_strip_width_x;

  /**
   *  Distance from fieldBorder to arenaBorder along y_axis (line excluded) [m]
   */
  double border_strip_width_y;

  /**
   *  Distance from center of penaltyMark to closest goal (goal line included) [m]
   */
  double penalty_mark_dist;

  /**
   *  Length of the penalty mark [m]
   */
  double penalty_mark_length;

  /**
   *  Distance between two posts (posts excluded) [m]
   */
  double goal_width;

  /**
   *  From goal line to the back of the goal (goal line included) [m]
   */
  double goal_depth;

  /**
   *  From goal line to goal area line (lines included) [m]
   */
  double goal_area_length;

  /**
   *  From one side of goal area to the other (lines included) [m]
   */
  double goal_area_width;

  /**
   *  From one goal line to the other (lines included) [m]
   */
  double field_length;

  /**
   *  From one side of the field to the other (lines included) [m]
   */
  double field_width;

private:
  /**
   * Synchronize the points of interests with current size of the field
   */
  void updatePointsOfInterest();

  /**
   * Synchronize the white lines based on the points of interest
   */
  void updateWhiteLines();

  /**
   * Synchronize the arena borders based on the points of interest
   */
  void updateArenaBorders();

  /**
   * Synchronize the goals and goal posts based on the points of interest
   */
  void updateGoals();

  /**
   * Synchronize the penalty marks based on the points of interest
   */
  void updatePenaltyMarks();

  /**
   * Synchronize poi_by_type based on points_of_interest
   */
  void updatePointsOfInterestByType();

  /**
   * Stores points of interest visually identifiable of the field along with a given name.
   * e.g.: corners penalty marks etc...
   */
  std::map<std::string, cv::Point3f> points_of_interest;

  /**
   * Stores points of interest positions by type of feature
   */
  std::map<POIType, std::vector<cv::Point3f>> poi_by_type;

  /**
   * List all the white segments in the field
   */
  std::vector<Segment> white_lines;
  std::vector<Segment> arena_borders;

  std::vector<Segment> goals;

  std::vector<cv::Point3f> goal_posts;
  std::vector<cv::Point3f> penalty_marks;
};

}  // namespace hl_monitoring
