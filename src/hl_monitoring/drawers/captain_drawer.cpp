#include <hl_monitoring/drawers/captain_drawer.h>

#include <opencv2/imgproc.hpp>

using namespace hl_communication;

namespace hl_monitoring
{
CaptainDrawer::CaptainDrawer()
{
  ball_drawer.setRadius(0.07);
  ball_drawer.setColor(cv::Scalar(0, 0, 255));
  opponent_drawer.setRadius(0.25);
}

CaptainDrawer::~CaptainDrawer()
{
}

void CaptainDrawer::draw(FieldToImgConverter converter, const hl_communication::Captain& captain, cv::Mat* out)
{
  if (captain.has_ball())
  {
    ball_drawer.draw(converter, captain.ball().position(), out);
  }
  for (const CommonOpponent& opponent : captain.opponents())
  {
    opponent_drawer.draw(converter, opponent.pose(), out);
  }
  // TODO: draw orders
}

Json::Value CaptainDrawer::toJson() const
{
  Json::Value v = Drawer::toJson();
  v["ball_drawer"] = ball_drawer.toJson();
  v["opponent_drawer"] = opponent_drawer.toJson();
  return v;
}

void CaptainDrawer::fromJson(const Json::Value& v)
{
  Drawer::fromJson(v);
  if (v.isMember("ball_drawer"))
  {
    ball_drawer.fromJson(v["ball_drawer"]);
  }
  if (v.isMember("opponent_drawer"))
  {
    opponent_drawer.fromJson(v["opponent_drawer"]);
  }
}

void CaptainDrawer::setColor(const cv::Scalar& new_color)
{
  // ball_drawer.setColor(new_color);
  opponent_drawer.setColor(new_color);
}

}  // namespace hl_monitoring
