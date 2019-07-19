#pragma once

#include "hl_monitoring/image_provider.h"

#include <opencv2/videoio.hpp>

namespace hl_monitoring
{
class ReplayImageProvider : public ImageProvider
{
public:
  ReplayImageProvider();
  ReplayImageProvider(const std::string& video_path);
  ReplayImageProvider(const std::string& video_path, const std::string& meta_information_path);

  void loadVideo(const std::string& video_path);
  /**
   * Creates default meta information based on video fps
   */
  void setDefaultMetaInformation();
  void loadMetaInformation(const std::string& meta_information_path);

  void restartStream() override;

  CalibratedImage getCalibratedImage(uint64_t time_stamp) override;

  cv::Mat getNextImg() override;

  void update() override;

  bool isStreamFinished() override;

  void setIndex(int index);

  /**
   * Try to load all the replays in the given folder based on the following elements:
   * - Movie ends with '.avi' files
   * - Metadata files have the same name as movie file but extension ".pb"
   */
  static std::vector<std::unique_ptr<ImageProvider>> loadReplays(const std::string& folder);

private:
  /**
   * The video read from the file
   */
  cv::VideoCapture video;

  /**
   * The last image retrieved
   */
  cv::Mat last_img;
};

}  // namespace hl_monitoring
