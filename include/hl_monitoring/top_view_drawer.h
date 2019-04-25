#pragma once

#include <hl_monitoring/field.h>

namespace hl_monitoring
{
class TopViewDrawer
{
public:
  TopViewDrawer();
  TopViewDrawer(const cv::Size& img_size);

  void setImgSize(const cv::Size& new_img_size);

  /**
   * Return an Image with the field drawn on it
   */
  cv::Mat getImg(const Field& f) const;

  /**
   * Returns the scale of the field [px/m]
   */
  double getScale(const Field& f) const;

  /**
   * Return the image point corresponding to the given position (field_basis)
   */
  cv::Point getImgFromField(const Field& f, const cv::Point3f& pos_in_field) const;
  cv::Point getImgFromField(const Field& f, const cv::Point2f& pos_in_field) const;

private:
  /**
   * The size of the image to be generated
   */
  cv::Size img_size;

  /**
   * Color used for parts outside of the field
   */
  cv::Scalar background_color;

  /**
   * Color used for the field
   */
  cv::Scalar field_color;

  /**
   * Color used for parts outside of the field
   */
  cv::Scalar lines_color;

  /**
   * Color used for parts outside of the field
   */
  cv::Scalar goals_color;

  /**
   * Return the width of the lines on image
   */
  int getLineWidth(const Field& f) const;

  /**
   * Return the width of a goal on image
   */
  int getGoalWidth(const Field& f) const;

  /**
   * Return the length of a penalty mark on image
   */
  int getMarkLength(const Field& f) const;

  void drawTurf(const Field& f, cv::Mat* dst) const;
  void drawLines(const Field& f, cv::Mat* dst) const;
  void drawPenaltyMarks(const Field& f, cv::Mat* dst) const;
  void drawGoals(const Field& f, cv::Mat* dst) const;
  void drawCenter(const Field& f, cv::Mat* dst) const;

  void drawMark(const Field& f, const cv::Point3f& mark_pos_in_field, cv::Mat* dst) const;
};

}  // namespace hl_monitoring
