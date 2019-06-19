#pragma once

#include "hl_monitoring/image_provider.h"

#include <json/json.h>
#include <flycapture/FlyCapture2.h>
#include <opencv2/videoio.hpp>

namespace hl_monitoring
{
class PtGreyException : public std::runtime_error
{
public:
  PtGreyException(const std::string& msg);
};

class PtGreyConnectionException : public std::runtime_error
{
public:
  PtGreyConnectionException(const std::string& msg);
};

/**
 * Use 'flycapture' API from FLIR (previously PtGrey) to capture images
 */
class FlyCapImageProvider : public ImageProvider
{
public:
  /**
   * If output_prefix is not empty, write video during execution and saves
   * MetaInformation when object is closed
   */
  FlyCapImageProvider(const Json::Value& v, const std::string& output_prefix = "");
  virtual ~FlyCapImageProvider();

  void connect();
  /**
   * Cut current connection if active, then start a new connection
   */
  void reconnectCamera();

  void openInputStream();
  void openOutputStream(const std::string& output_path);

  void restartStream() override;

  CalibratedImage getCalibratedImage(uint64_t time_stamp) override;

  void update() override;

  cv::Mat getNextImg() override;

  bool isStreamFinished() override;

  void saveVideoMetaInformation();

  void updateProperty(const FlyCapture2::Property& wished_property);
  void applyWishedProperties();

  /**
   * Retrieve current image format
   */
  FlyCapture2::GigEImageSettings getImageSettings();

  /*
   * Update the packet size and the packet delay to reduce image
   * inconsistencies
   * NOTE: this has to be done while the camera is not capturing
   */
  void updatePacketProperties();

  /**
   * Update the imaging mode, binning and PixelFormat of the camera
   */
  void updateImageSettings();

  void setImagingMode(FlyCapture2::Mode mode);

  /*
   * Update binning properties if required
   * WARNING: updating binning can change the current imaging mode
   */
  void updateBinning(unsigned int h_binning, unsigned int v_binning);

  /**
   * Set current pixel format without changing other properties
   * Note: dump(getImageSettingsInfo(), std::cout) to see available formats
   */
  void setPixelFormat(FlyCapture2::PixelFormat pixel_format);

private:
  /**
   * PtGrey camera
   */
  FlyCapture2::GigECamera camera;

  /**
   * Is the image stream started
   */
  bool is_capturing;

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
   * Required frame_rate
   */
  double frame_rate;

  /**
   * Shutter duration [ms]
   */
  double shutter;

  /**
   * Camera gain
   */
  double gain;

  /**
   * The prefix used for writing video file and meta_information file. If empty,
   * then no files are written
   */
  std::string output_prefix;

  /**
   * General unique identifier of the camera
   */
  FlyCapture2::PGRGuid guid;

  /**
   * Is guid provided?
   */
  bool has_guid;
};

}  // namespace hl_monitoring
