#pragma once

#include <hl_monitoring/drawers/drawer.h>

namespace hl_monitoring
{
/**
 * Project a disk on the ground and draw it (no border)
 * ground_pos: field referential [m]
 * radius: [m]
 * alpha: transparency level (1.0 -> opaque)
 */
void drawGroundDisk(cv::Mat* out, FieldToImgConverter converter, const cv::Point2f& ground_center, double radius,
                    const cv::Scalar& color, double alpha = 1.0, double arc_step = M_PI / 180);

/**
 * Project a circle on the ground and draw it (no border)
 * ground_pos: field referential [m]
 * radius: [m]
 * alpha: transparency level (1.0 -> opaque)
 */
void drawGroundCircle(cv::Mat* out, FieldToImgConverter converter, const cv::Point2f& ground_center, double radius,
                      const cv::Scalar& color, double thickness = 1.0, double arc_step = M_PI / 180);

}  // namespace hl_monitoring
