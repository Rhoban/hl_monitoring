#include "hl_monitoring/image_provider.h"

#include <hl_communication/utils.h>

using namespace hl_communication;

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

void ImageProvider::setPose(int frame_idx, const Pose3D& pose)
{
  meta_information.mutable_frames(frame_idx)->mutable_pose()->CopyFrom(pose);
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

const VideoMetaInformation& ImageProvider::getMetaInformation() const
{
  return meta_information;
}

CameraMetaInformation ImageProvider::getCameraMetaInformation() const
{
  return getCameraMetaInformation(index);
}

CameraMetaInformation ImageProvider::getCameraMetaInformation(int index) const
{
  CameraMetaInformation camera_meta;
  if (meta_information.has_camera_parameters())
  {
    camera_meta.mutable_camera_parameters()->CopyFrom(meta_information.camera_parameters());
  }
  if (meta_information.frames_size() <= index)
  {
    throw std::out_of_range(HL_DEBUG + "invalid index: " + std::to_string(index));
  }

  const FrameEntry& frame = meta_information.frames(index);
  if (frame.has_pose())
  {
    camera_meta.mutable_pose()->CopyFrom(frame.pose());
  }
  else if (meta_information.has_default_pose())
  {
    camera_meta.mutable_pose()->CopyFrom(meta_information.default_pose());
  }
  return camera_meta;
}

uint64_t ImageProvider::getTimeStamp() const
{
  return getTimeStamp(index);
}

uint64_t ImageProvider::getTimeStamp(int idx) const
{
  return time_stamp_by_index.at(idx);
}

const FrameEntry& ImageProvider::getFrameEntry(uint64_t ts)
{
  int idx = getIndex(ts);
  return meta_information.frames(idx);
}

void ImageProvider::pushTimeStamp(int idx, uint64_t time_stamp)
{
  indices_by_time_stamp[time_stamp] = idx;
  time_stamp_by_index[idx] = time_stamp;
}

int ImageProvider::getIndex(uint64_t time_stamp) const
{
  if (indices_by_time_stamp.size() == 0 || indices_by_time_stamp.begin()->first > time_stamp)
  {
    return -1;
  }
  auto it = indices_by_time_stamp.upper_bound(time_stamp);
  if (it == indices_by_time_stamp.end() || it->first > time_stamp)
  {
    it--;
  }
  return std::min(nb_frames - 1, it->second);
}

}  // namespace hl_monitoring
