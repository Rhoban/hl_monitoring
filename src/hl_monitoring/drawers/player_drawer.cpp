#include <hl_monitoring/drawers/player_drawer.h>

#include <hl_communication/utils.h>
#include <hl_monitoring/utils.h>

#include <opencv2/imgproc.hpp>

using namespace hl_communication;

namespace hl_monitoring
{
PlayerDrawer::PlayerDrawer() : color(0, 0, 0)
{
}

PlayerDrawer::~PlayerDrawer()
{
}

void PlayerDrawer::draw(FieldToImgConverter converter, const RobotMsg& robot, cv::Mat* out)
{
  if (robot.has_perception())
  {
    const Perception& perception = robot.perception();
    if (perception.self_in_field_size() > 0)
    {
      // Currently only draw first pose
      const WeightedPose& weighted_pose = perception.self_in_field(0);
      const PoseDistribution& pose = weighted_pose.pose();
      pose_drawer.draw(converter, pose, out);
      const PositionDistribution& position = pose.position();
      cv::Point3f robot_pos(position.x(), position.y(), 0);

      int robot_id = robot.robot_id().robot_id();

      name_drawer.setImgOffset(cv::Point2f(0,30));
      name_drawer.draw(converter, {robot_pos, std::to_string(robot_id)}, out);

      // Drawing intention
      // TODO: check dst
      // TODO: take into account waypoints
      // TODO: take into account kick_target_in_field
      // TODO: maybe move this to IntentionDrawer (Requires initial position)?
      if (robot.has_intention() && robot.intention().has_target_pose_in_field())
      {
        const Intention& intention = robot.intention();
        const PositionDistribution& target_pos = intention.target_pose_in_field().position();
        cv::Point3f robot_dst(target_pos.x(), target_pos.y(), 0);
        target_drawer.draw(converter, { robot_pos, robot_dst }, out);
      }
    }
  }
}

void PlayerDrawer::setColor(const cv::Scalar& new_color)
{
  color = new_color;
  pose_drawer.setColor(new_color);
  name_drawer.setColor(new_color);
  target_drawer.setColor(new_color);
}

Json::Value PlayerDrawer::toJson() const
{
  Json::Value v = Drawer::toJson();
  v["color"] = hl_monitoring::toJson(color);
  v["pose_drawer"] = pose_drawer.toJson();
  v["name_drawer"] = name_drawer.toJson();
  v["target_drawer"] = target_drawer.toJson();
  return v;
}

void PlayerDrawer::fromJson(const Json::Value& v)
{
  Drawer::fromJson(v);
  cv::Scalar old_color = color;
  tryReadVal(v, "color", &old_color);
  if (old_color != color)
  {
    setColor(old_color);
  }
  if (v.isMember("pose_drawer"))
  {
    pose_drawer.fromJson(v["pose_drawer"]);
  }
  if (v.isMember("name_drawer"))
  {
    pose_drawer.fromJson(v["name_drawer"]);
  }
  if (v.isMember("target_drawer"))
  {
    target_drawer.fromJson(v["target_drawer"]);
  }
}

}  // namespace hl_monitoring
