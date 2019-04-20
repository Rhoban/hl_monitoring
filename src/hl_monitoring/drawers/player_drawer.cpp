#include <hl_monitoring/drawers/player_drawer.h>

#include <hl_communication/utils.h>
#include <hl_monitoring/utils.h>

#include <opencv2/imgproc.hpp>

using namespace hl_communication;

namespace hl_monitoring
{
PlayerDrawer::PlayerDrawer() : color(0,0,0)
{
}

PlayerDrawer::~PlayerDrawer()
{
}

void PlayerDrawer::draw(const CameraMetaInformation& camera_information, const RobotMsg& robot, cv::Mat* out)
{
  if (robot.has_perception())
  {
    const Perception& perception = robot.perception();
    if (perception.self_in_field_size() > 0)
    {
      // Currently only draw first pose
      const WeightedPose& weighted_pose = perception.self_in_field(0);
      pose_drawer.draw(camera_information, weighted_pose.pose(), out);
    }
  }
}

void PlayerDrawer::draw(const Field& f, const TopViewDrawer& top_view_drawer, const RobotMsg& robot, cv::Mat* out)
{
  if (robot.has_perception())
  {
    const Perception& perception = robot.perception();
    if (perception.self_in_field_size() > 0)
    {
      // Currently only draw first position
      const WeightedPose& weighted_pose = perception.self_in_field(0);
      pose_drawer.draw(f, top_view_drawer, weighted_pose.pose(), out);
    }
  }
}

void PlayerDrawer::setColor(const cv::Scalar& new_color)
{
  color = new_color;
  pose_drawer.setColor(new_color);
}

Json::Value PlayerDrawer::toJson() const
{
  Json::Value v = Drawer::toJson();
  v["color"] = hl_monitoring::toJson(color);
  return v;
}

void PlayerDrawer::fromJson(const Json::Value& v)
{
  tryReadVal(v, "color", &color);
}

}  // namespace hl_monitoring
