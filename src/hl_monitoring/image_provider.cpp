#include "hl_monitoring/image_provider.h"

namespace hl_monitoring
{
ImageProvider::ImageProvider() : index(-1), nb_frames(0)
{
}

CalibratedImage ImageProvider::getCalibratedImage(uint64_t time_stamp, bool system_clock)
{
  if (system_clock)
  {
    time_stamp -= getOffset();
  }
  return getCalibratedImage(time_stamp);
}

uint64_t ImageProvider::getStart() const
{
  if (indices_by_time_stamp.size() == 0)
    return 0;
  return indices_by_time_stamp.begin()->first;
}

uint64_t ImageProvider::getEnd() const
{
  if (indices_by_time_stamp.size() == 0)
    return 0;
  return indices_by_time_stamp.rbegin()->first;
}

size_t ImageProvider::getNbFrames() const
{
  return nb_frames;
}

void ImageProvider::setIntrinsic(const IntrinsicParameters& params)
{
  meta_information.mutable_camera_parameters()->CopyFrom(params);
}

void ImageProvider::setDefaultPose(const Pose3D& pose)
{
  meta_information.mutable_default_pose()->CopyFrom(pose);
}

void ImageProvider::setOffset(int64 offset)
{
  meta_information.set_time_offset(offset);
}

int64 ImageProvider::getOffset() const
{
  if (!meta_information.has_time_offset())
  {
    return 0;
  }
  return meta_information.time_offset();
}

}  // namespace hl_monitoring
