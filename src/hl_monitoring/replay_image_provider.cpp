#include "hl_monitoring/replay_image_provider.h"

#include <hl_communication/utils.h>

#include <fstream>

using namespace hl_communication;

namespace hl_monitoring
{
ReplayImageProvider::ReplayImageProvider()
{
}

ReplayImageProvider::ReplayImageProvider(const std::string& video_path)
{
  loadVideo(video_path);
  setDefaultMetaInformation();
}

ReplayImageProvider::ReplayImageProvider(const std::string& video_path, const std::string& meta_information_path)
{
  loadVideo(video_path);
  loadMetaInformation(meta_information_path);
}

void ReplayImageProvider::loadVideo(const std::string& video_path)
{
  if (!video.open(video_path))
  {
    throw std::runtime_error("Failed to open video '" + video_path + "'");
  }
  index = 0;
  nb_frames = video.get(cv::CAP_PROP_FRAME_COUNT);
}

void ReplayImageProvider::loadMetaInformation(const std::string& meta_information_path)
{
  std::ifstream in(meta_information_path, std::ios::binary);
  if (!in.good())
  {
    throw std::runtime_error("Failed to open file '" + meta_information_path + "'");
  }
  if (!meta_information.ParseFromIstream(&in))
  {
    throw std::runtime_error("Failed to read file '" + meta_information_path + "'");
  }
  index = 0;
  nb_frames = meta_information.frames_size();
  for (int idx = 0; idx < nb_frames; idx++)
  {
    uint64_t time_stamp = meta_information.frames(idx).time_stamp();
    if (indices_by_time_stamp.count(time_stamp) > 0)
    {
      throw std::runtime_error(HL_DEBUG + "Duplicated time_stamp " + std::to_string(time_stamp));
    }
    pushTimeStamp(idx, time_stamp);
  }
}

void ReplayImageProvider::setDefaultMetaInformation()
{
  double fps = video.get(cv::CAP_PROP_FPS);
  uint64_t dt = std::pow(10, 6) / fps;
  uint64_t ts = 0;
  for (int idx = 0; idx < nb_frames; idx++)
  {
    FrameEntry* frame = meta_information.add_frames();
    frame->set_time_stamp(ts);
    ts += dt;
    pushTimeStamp(idx, ts);
  }
}

void ReplayImageProvider::restartStream()
{
  setIndex(0);
}

CalibratedImage ReplayImageProvider::getCalibratedImage(uint64_t time_stamp)
{
  int new_index = getIndex(time_stamp);
  cv::Mat img;
  if (new_index == -1)
  {
    return CalibratedImage();
  }
  else if (new_index == index - 1)  // Asking for previous image again
  {
    img = last_img;
  }
  else
  {
    if (new_index != index)
    {
      setIndex(new_index);
    }
    img = getNextImg();
  }
  return CalibratedImage(img, getCameraMetaInformation(new_index));
}

cv::Mat ReplayImageProvider::getNextImg()
{
  if (isStreamFinished())
  {
    throw std::logic_error("Asking for a new frame while stream is finished");
  }

  video >> last_img;
  index++;
  if (last_img.empty())
  {
    throw std::runtime_error(HL_DEBUG + "Blank frame at frame: " + std::to_string(index) + "/" +
                             std::to_string(nb_frames));
  }
  return last_img;
}

void ReplayImageProvider::update()
{
  // Nothing required
}

bool ReplayImageProvider::isStreamFinished()
{
  return index >= nb_frames;
}

void ReplayImageProvider::setIndex(int new_index)
{
  index = new_index;
  if (!video.set(cv::CAP_PROP_POS_FRAMES, index))
  {
    throw std::runtime_error(HL_DEBUG + "Failed to set index to " + std::to_string(index) + " in video");
  }
}
}  // namespace hl_monitoring
