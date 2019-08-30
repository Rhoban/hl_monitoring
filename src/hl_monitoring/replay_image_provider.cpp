#include "hl_monitoring/replay_image_provider.h"

#include <hl_communication/utils.h>

#include <experimental/filesystem>  // Due to gcc version
#include <fstream>
#include <iostream>

namespace fs = std::experimental::filesystem;

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
    uint64_t time_stamp = getTS(meta_information.frames(idx), true);
    if (indices_by_time_stamp.count(time_stamp) > 0)
    {
      throw std::runtime_error(HL_DEBUG + "Duplicated time_stamp " + std::to_string(time_stamp));
    }
    pushTimeStamp(idx, time_stamp);
  }
  if (nb_frames > 0 && meta_information.has_source_id() && !meta_information.source_id().has_utc_start())
  {
    std::cout << "Setting meta-information default start" << std::endl;
    uint64_t first_frame_utc = getTS(meta_information.frames(0), true);
    meta_information.mutable_source_id()->set_utc_start(first_frame_utc);
  }
}

void ReplayImageProvider::setDefaultMetaInformation()
{
  double fps = video.get(cv::CAP_PROP_FPS);
  uint64_t dt = std::pow(10, 6) / fps;
  uint64_t monotonic_ts = 0;
  uint64_t utc_ts = getUTCTimeStamp();  // When setting default meta-information, use current date
  for (int idx = 0; idx < nb_frames; idx++)
  {
    FrameEntry* frame = meta_information.add_frames();
    frame->set_monotonic_ts(monotonic_ts);
    monotonic_ts += dt;
    frame->set_utc_ts(utc_ts);
    utc_ts += dt;
    pushTimeStamp(idx, monotonic_ts);
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

std::vector<std::unique_ptr<ImageProvider>> ReplayImageProvider::loadReplays(const std::string& folder)
{
  std::vector<std::unique_ptr<ImageProvider>> providers;
  std::vector<fs::path> movie_paths;
  for (auto& p : fs::recursive_directory_iterator(folder))
  {
    const fs::path& path = p.path();
    if (path.extension() == ".avi")
      movie_paths.push_back(path);
  }
  for (const fs::path& movie_path : movie_paths)
  {
    fs::path pb_path = movie_path;
    pb_path.replace_extension(".pb");
    if (fs::exists(pb_path))
    {
      providers.push_back(std::unique_ptr<ImageProvider>(new ReplayImageProvider(movie_path, pb_path)));
    }
    else
    {
      std::cerr << "File " << pb_path << " was not found" << std::endl;
    }
  }
  return providers;
}

}  // namespace hl_monitoring
