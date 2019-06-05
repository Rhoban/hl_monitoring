#include <hl_monitoring/drawers/player_drawer.h>

#include <hl_communication/utils.h>
#include <hl_monitoring/utils.h>
#include <iostream>

#include <opencv2/imgproc.hpp>

using namespace hl_communication;

namespace hl_monitoring
{
PlayerDrawer::PlayerDrawer() : color(0, 0, 0), target_drawer(ArrowDrawer::ArrowCross, 1.0)
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
      for (int msg_idx = 0; msg_idx < perception.self_in_field_size(); msg_idx++)
      {
        const WeightedPose& weighted_pose = perception.self_in_field(msg_idx);
        const PoseDistribution& pose = weighted_pose.pose();
        pose_drawer.draw(converter, pose, out);

        // pose_drawer.draw(converter, pose, out);

        const PositionDistribution& position = pose.position();
        cv::Point3f robot_pos(position.x(), position.y(), 0);

        int robot_id = robot.robot_id().robot_id();

        name_drawer.setImgOffset(cv::Point2f(0, 30));
        name_drawer.draw(converter, { robot_pos, std::to_string(robot_id) }, out);
      }

      const WeightedPose& weighted_pose = perception.self_in_field(0);
      const PoseDistribution& pose = weighted_pose.pose();
      const PositionDistribution& position = pose.position();
      cv::Point3f robot_pos(position.x(), position.y(), 0);

      int robot_id = robot.robot_id().robot_id();

      // Drawing intention
      // TODO: take into account waypoints
      // TODO: maybe move this to IntentionDrawer (Requires initial position of robot?)
      if (robot.has_intention())
      {
        const Intention& intention = robot.intention();
        // Draw positioning target
        if (robot.intention().has_target_pose_in_field())
        {
          const PositionDistribution& target_pos = intention.target_pose_in_field().position();
          cv::Point3f robot_dst(target_pos.x(), target_pos.y(), 0);
          target_drawer.draw(converter, { robot_pos, robot_dst }, out);
        }
        // Draw kick target if allowed
        if (intention.has_kick())
        {
          const KickIntention& kick = intention.kick();
          cv::Point3f kick_src_in_field = cvtToPoint3f(kick.start());
          cv::Point3f kick_target_in_field = cvtToPoint3f(kick.target());
          kick_drawer.draw(converter, { kick_src_in_field, kick_target_in_field }, out);
        }
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
  kick_drawer.setColor(new_color);
}

Json::Value PlayerDrawer::toJson() const
{
  Json::Value v = Drawer::toJson();
  v["color"] = val2Json(color);
  v["pose_drawer"] = pose_drawer.toJson();
  v["name_drawer"] = name_drawer.toJson();
  v["target_drawer"] = target_drawer.toJson();
  v["kick_drawer"] = kick_drawer.toJson();
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
  if (v.isMember("kick_drawer"))
  {
    kick_drawer.fromJson(v["kick_drawer"]);
  }
}

}  // namespace hl_monitoring
