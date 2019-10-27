#include <hl_monitoring/drawers/player_drawer.h>

#include <hl_monitoring/drawers/geometry.h>

#include <hl_communication/perception.pb.h>
#include <hl_communication/utils.h>

#include <opencv2/imgproc.hpp>

#include <iostream>

using namespace hl_communication;

namespace hl_monitoring
{
PlayerDrawer::PlayerDrawer()
  : target_drawer(ArrowDrawer::ArrowCross, 1.0), color(0, 0, 0), ball_enabled(false), opponents_enabled(false)
{
  name_drawer.setFontScale(0.7);
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
      for (int msg_idx = 0; msg_idx < perception.self_in_field_size(); msg_idx++)
      {
        const WeightedPose& weighted_pose = perception.self_in_field(msg_idx);
        const PoseDistribution& pose = weighted_pose.pose();
        pose_drawer.draw(converter, pose, out);

        const PositionDistribution& position = pose.position();
        cv::Point3f robot_pos(position.x(), position.y(), 0);

        int robot_id = robot.robot_id().robot_id();

        name_drawer.setImgOffset(cv::Point2f(0, 15));
        name_drawer.draw(converter, { robot_pos, std::to_string(robot_id) }, out);
      }

      const WeightedPose& weighted_pose = perception.self_in_field(0);
      const PoseDistribution& pose = weighted_pose.pose();
      const PositionDistribution& position = pose.position();
      cv::Point3f robot_pos(position.x(), position.y(), 0);

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
      // Drawing self balls
      if (ball_enabled && perception.has_ball_in_self())
      {
        cv::Point3f ball_in_field = fieldFromSelf(perception.ball_in_self(), pose);
        double ball_radius = 0.07;  // [m]
        drawGroundDisk(out, converter, cv::Point2f(ball_in_field.x, ball_in_field.y), ball_radius,
                       cv::Scalar(0, 0, 255));
      }
      // Drawing other robots
      if (opponents_enabled)
      {
        for (const WeightedRobotPose& robot_weighted_pose : perception.robots())
        {
          const PoseDistribution& robot_pose_in_self = robot_weighted_pose.robot().robot_in_self();
          const PositionDistribution& robot_pos_in_self = robot_pose_in_self.position();
          cv::Point3f robot_in_field = fieldFromSelf(robot_pos_in_self, pose);
          double opp_radius = 0.25;  // [m]
          double min_alpha = 0;
          double max_alpha = 0.7;
          double alpha = robot_weighted_pose.probability() * (max_alpha - min_alpha) + min_alpha;
          drawGroundDisk(out, converter, cv::Point2f(robot_in_field.x, robot_in_field.y), opp_radius,
                         cv::Scalar(0, 0, 255), alpha);
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

void PlayerDrawer::setBallDrawing(bool enabled)
{
  ball_enabled = enabled;
}

void PlayerDrawer::setOpponentsDrawing(bool enabled)
{
  opponents_enabled = enabled;
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
