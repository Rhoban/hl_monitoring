#pragma once

#include "hl_monitoring/calibrated_image.h"

namespace hl_monitoring
{
class ImageProvider
{
public:
  ImageProvider();

  virtual ~ImageProvider()
  {
  }

  /**
   * Return to the first frame of the stream, throw error on live stream
   */
  virtual void restartStream() = 0;

  /**
   * Return an image along with associated intrinsic and extrinsic parameters
   */
  virtual CalibratedImage getCalibratedImage(uint64_t time_stamp) = 0;

  /**
   * system_clock:
   * - true: time_stamp is given in a global referential (time_since_epoch)
   * - false: If time_stamp is given with a local referential (steady_clock)
   */
  CalibratedImage getCalibratedImage(uint64_t time_stamp, bool system_clock);

  /**
   * For livestream, receive images from the stream
   */
  virtual void update() = 0;

  /**
   * Get the next available image
   */
  virtual cv::Mat getNextImg() = 0;

  /**
   * Return true if the last image of the stream has been reached.
   * For live streams, means that the camera has been disconnected
   */
  virtual bool isStreamFinished() = 0;

  /**
   * Return the first time_stamp of the images received
   * If no element is found, returns 0
   */
  virtual uint64_t getStart() const;

  /**
   * Return the last time_stamp of the images received
   * If no element is found, returns 0
   */
  virtual uint64_t getEnd() const;

  /**
   * Returns the number of frames currently available in the image provider
   */
  virtual size_t getNbFrames() const;

  virtual void setIntrinsic(const IntrinsicParameters& params);
  virtual void setDefaultPose(const Pose3D& pose);

  /**
   * Set the offset in us between steady_clock and system_clock (time_since_epoch)
   */
  void setOffset(int64_t offset);

  /**
   * Get the offset in us between steady_clock and system_clock (time_since_epoch)
   */
  int64_t getOffset() const;

protected:
  /**
   * Information relevant to the video stream
   */
  VideoMetaInformation meta_information;

  /**
   * Provide access to indices based on steady_clock time_stamps
   */
  std::map<uint64_t, int> indices_by_time_stamp;

  /**
   * Index of the next image read in the video
   */
  int index;

  /**
   * The number of frames in the video
   */
  int nb_frames;
};

}  // namespace hl_monitoring
