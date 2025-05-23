#pragma once

#include "hl_monitoring/image_provider.h"

#include <opencv2/videoio.hpp>

namespace hl_monitoring
{
/**
 * Use OpenCV standard API to open a video stream
 * - Images read can be directly encoded in a video
 * - Timestamps are based on the steady clock acquisition time, not time_since_epoch
 */
class OpenCVImageProvider : public ImageProvider
{
public:
  /**
   * If output_prefix is not empty, write video during execution and saves
   * MetaInformation when object is closed
   */
  OpenCVImageProvider(const std::string& video_path, const std::string& output_prefix = "");
  virtual ~OpenCVImageProvider();

  double getFPS() const;

  void openInputStream(const std::string& video_path);
  void openOutputStream(const std::string& output_path);

  void restartStream() override;

  CalibratedImage getCalibratedImage(uint64_t time_stamp) override;

  void update() override;

  cv::Mat getNextImg() override;

  bool isStreamFinished() override;

  void saveVideoMetaInformation();

private:
  /**
   * The video read from the file
   */
  cv::VideoCapture input;

  /**
   * The video read from the file
   */
  cv::VideoWriter output;

  /**
   * Last img read
   */
  cv::Mat img;

  /**
   * Size of the images provided by the camera
   */
  cv::Size img_size;

  /**
   * The prefix used for writing video file and meta_information file. If empty,
   * then no files are written
   */
  std::string output_prefix;
};

}  // namespace hl_monitoring
