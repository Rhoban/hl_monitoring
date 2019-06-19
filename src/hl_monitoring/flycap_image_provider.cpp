#include "hl_monitoring/flycap_image_provider.h"

#include <hl_communication/utils.h>

#include <opencv2/imgproc.hpp>

#include <chrono>
#include <fstream>
#include <iostream>

using namespace std::chrono;
using namespace hl_communication;

namespace hl_monitoring
{
PtGreyException::PtGreyException(const std::string& msg) : std::runtime_error(msg)
{
}

PtGreyConnectionException::PtGreyConnectionException(const std::string& msg) : std::runtime_error(msg)
{
}

FlyCapImageProvider::FlyCapImageProvider(const Json::Value& v, const std::string& output_prefix_)
  : output_prefix(output_prefix_)
{
  readVal(v, "frame_rate", &frame_rate);
  readVal(v, "shutter", &shutter);
  readVal(v, "gain", &gain);
  if (v.isMember("guid"))
  {
    const Json::Value& guid_value = v["guid"];
    if (guid_value.size() != 4)
    {
      throw std::runtime_error(HL_DEBUG + "Invalid size for 'guid': " + std::to_string(guid_value.size()));
    }
    for (Json::ArrayIndex idx = 0; idx < 4; idx++)
    {
      guid.value[idx] = guid_value[idx].asUInt();
    }
    has_guid = true;
  }
  openInputStream();
  getNextImg();
}
FlyCapImageProvider::~FlyCapImageProvider()
{
  saveVideoMetaInformation();
}

void FlyCapImageProvider::connect()
{
  FlyCapture2::Error error;
  if (has_guid)
  {
    error = camera.Connect(&guid);
  }
  else
  {
    error = camera.Connect(0);
  }
  if (error != FlyCapture2::PGRERROR_OK)
  {
    throw PtGreyConnectionException(HL_DEBUG + "Failed to connect to camera");
  }
}

void FlyCapImageProvider::reconnectCamera()
{
  FlyCapture2::Error error;
  // If connected, disconnect
  if (camera.IsConnected())
  {
    camera.Disconnect();
  }
  connect();
}

void FlyCapImageProvider::openInputStream()
{
  FlyCapture2::Error error;
  // If connected, disconnect
  if (camera.IsConnected())
  {
    camera.Disconnect();
  }
  connect();
  try
  {
    // Properly set up size of packet and delay between packets
    updatePacketProperties();
    // Set appropriate mode and size
    updateImageSettings();
    // Start capture
    while (true)
    {
      error = camera.StartCapture();
      if (error == FlyCapture2::PGRERROR_ISOCH_BANDWIDTH_EXCEEDED)
      {
        throw PtGreyException(HL_DEBUG + "bandwidth exceeded");
      }
      else if (error == FlyCapture2::PGRERROR_ISOCH_ALREADY_STARTED)
      {
        std::cerr << HL_DEBUG + "Isoch already started: stopping" << std::endl;
        camera.StopCapture();
      }
      else if (error != FlyCapture2::PGRERROR_OK)
      {
        std::cerr << HL_DEBUG + "Failed to start image capture: '" << error.GetDescription() << "'";
        reconnectCamera();
      }
      else
      {
        break;
      }
    }
    is_capturing = true;
    // Apply wished properties
    applyWishedProperties();

    // Currently, timestamp from the camera is not used, accurate timing is
    // therefore not provided
    FlyCapture2::EmbeddedImageInfo embeddedInfo;
    embeddedInfo.timestamp.onOff = false;
    error = camera.SetEmbeddedImageInfo(&embeddedInfo);
    if (error != FlyCapture2::PGRERROR_OK)
    {
      throw PtGreyException("failed to set 'embedded ImageInfo'");
    }
  }
  catch (const PtGreyException& exc)
  {
    std::cerr << "Got a PtGreyException while preparing camera: " << exc.what() << std::endl;
    camera.Disconnect();
    throw;
  }
}

void FlyCapImageProvider::openOutputStream(const std::string& output_path)
{
  if (!camera.IsConnected())
  {
    throw std::logic_error(HL_DEBUG + " input stream is not open yet");
  }
  bool use_color = true;
  std::cout << "Opening video_stream of size: " << img_size << std::endl;
  output.open(output_path, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), frame_rate, img_size, use_color);
  if (!output.isOpened())
  {
    throw std::runtime_error(HL_DEBUG + "Failed to open video at '" + output_path + "'");
  }
}

void FlyCapImageProvider::restartStream()
{
  throw std::logic_error("It makes no sense to restart the stream in a 'FlyCapImageProvider'");
}

CalibratedImage FlyCapImageProvider::getCalibratedImage(uint64_t time_stamp)
{
  if (nb_frames == 0)
  {
    throw std::runtime_error(HL_DEBUG + " no frames found in the stream");
  }
  if (time_stamp < indices_by_time_stamp.rbegin()->first)
  {
    throw std::runtime_error(HL_DEBUG + " asking for frames in the past is not supported");
  }

  int index = indices_by_time_stamp.size() - 1;
  return CalibratedImage(img, getCameraMetaInformation(index));
}

void FlyCapImageProvider::update()
{
  // TODO: note: ideally, images should be polled in another thread and only
  // synchronization should happen here
  getNextImg();
}

cv::Mat FlyCapImageProvider::getNextImg()
{
  FlyCapture2::Image fc_image;
  bool retry = true;
  while (retry)
  {
    FlyCapture2::Error error = camera.RetrieveBuffer(&fc_image);
    if (error == FlyCapture2::PGRERROR_TIMEOUT)
    {
      std::cerr << HL_DEBUG << "Retrieve buffer timed out" << std::endl;
    }
    else if (error != FlyCapture2::PGRERROR_OK)
    {
      std::cerr << HL_DEBUG << "Failed buffer retrieval" << error.GetDescription() << std::endl;
    }
    else
    {
      retry = false;
    }
  }

  uint64_t monotonic_ts = hl_communication::getTimeStamp();
  uint64_t utc_ts = hl_communication::getUTCTimeStamp();

  unsigned int bytes_per_row = fc_image.GetReceivedDataSize() / fc_image.GetRows();
  cv::Mat tmp_img = cv::Mat(fc_image.GetRows(), fc_image.GetCols(), CV_8UC3, fc_image.GetData(), bytes_per_row).clone();
  if (tmp_img.empty())
  {
    throw std::runtime_error(HL_DEBUG + "Blank frame at frame: " + std::to_string(index) + "/" +
                             std::to_string(nb_frames));
  }
  cv::cvtColor(tmp_img, img, cv::COLOR_RGB2BGR);
  // register image
  pushTimeStamp(index, monotonic_ts);
  FrameEntry* entry = meta_information.add_frames();
  entry->set_utc_ts(utc_ts);
  entry->set_monotonic_ts(monotonic_ts);
  index++;
  nb_frames++;
  // Open output stream after capturing first image
  if (output_prefix != "" && !output.isOpened())
  {
    img_size = img.size();
    openOutputStream(output_prefix + ".avi");
  }
  // Write image to output video if opened
  if (output.isOpened())
  {
    if (img.size() != img_size)
    {
      std::ostringstream oss;
      oss << "Size mismatch: (video size: " << img_size << ", img size: " << img.size() << ")";
      throw std::runtime_error(HL_DEBUG + oss.str());
    }
    output.write(img);
  }
  return img;
}

bool FlyCapImageProvider::isStreamFinished()
{
  return false;
}

void FlyCapImageProvider::saveVideoMetaInformation()
{
  // Do not save if no output_prefix has been provided
  if (output_prefix == "")
    return;

  std::string path = output_prefix + ".bin";
  std::ofstream out(path, std::ios::binary);
  if (!out.good())
  {
    throw std::runtime_error(HL_DEBUG + "Failed to open file '" + path + "'");
  }
  if (!meta_information.SerializeToOstream(&out))
  {
    throw std::runtime_error(HL_DEBUG + "Failed to write to file '" + path + "'");
  }
}

void FlyCapImageProvider::updatePacketProperties()
{
  // Prepare packets
  // Using larger packets reduces the number of CPU interruptions
  FlyCapture2::GigEProperty packet_size_prop;
  packet_size_prop.propType = FlyCapture2::PACKET_SIZE;
  packet_size_prop.value = 9000;
  FlyCapture2::GigEProperty packet_delay_prop;
  packet_delay_prop.propType = FlyCapture2::PACKET_DELAY;
  // In 640*480, at 25 fps, with 3 bytes per pixel, the required bandwidth is:
  // 640 * 480 * 25 * 3 ~= 24 * 10^6 <- 23 MB/s
  // Some examples of combinations packet_delay, packet_size and bandwidth are
  // provided in the Technical reference guide of blackfly:
  // section:  "Determining Bandwidth requirements"
  // 55 MB: (9000,1800) or (1400,255)
  // 25 MB: (9000,5900) or (1400,900)
  //
  // Since no obvious relationships between Baudrate and value is exhibited,
  // setting this value is subjet to extreme caution
  // - If the value is too low: Risk of 'dropping frames'
  // - If the value is too high: Unknown consequences
  packet_delay_prop.value = 6000;

  // Send packets
  FlyCapture2::Error error;
  error = camera.SetGigEProperty(&packet_size_prop);
  if (error != FlyCapture2::PGRERROR_OK)
  {
    std::ostringstream oss;
    oss << "FlyCapImageProvider::updatePacketProperties: failed to set packet size: " << error.GetType();
    {
      oss << ": Camera was capturing, stop capturing before changing packet "
             "size";
    }
    throw PtGreyException(oss.str());
  }
  error = camera.SetGigEProperty(&packet_delay_prop);
  if (error != FlyCapture2::PGRERROR_OK)
  {
    std::ostringstream oss;
    oss << "FlyCapImageProvider::updatePacketProperties: failed to set packet delay";
    if (is_capturing)
    {
      oss << ": Camera was capturing, stop capturing before changing packet "
             "size";
    }
    throw PtGreyException(oss.str());
  }
}

void FlyCapImageProvider::updateImageSettings()
{
  // Error variable
  setImagingMode(FlyCapture2::Mode::MODE_1);
  updateBinning(2, 2);
  setPixelFormat(FlyCapture2::PixelFormat::PIXEL_FORMAT_RGB8);
}

void FlyCapImageProvider::setImagingMode(FlyCapture2::Mode mode)
{
  FlyCapture2::Error error = camera.SetGigEImagingMode(mode);
  if (error != FlyCapture2::ErrorType::PGRERROR_OK)
  {
    throw PtGreyException(HL_DEBUG + error.GetDescription());
  }
}

void FlyCapImageProvider::updateBinning(unsigned int h_binning, unsigned int v_binning)
{
  // Retrieving current binning properties
  unsigned int current_h_binning = 0;
  unsigned int current_v_binning = 0;
  FlyCapture2::Error error;
  error = camera.GetGigEImageBinningSettings(&current_h_binning, &current_v_binning);
  if (error != FlyCapture2::ErrorType::PGRERROR_OK)
  {
    throw PtGreyException(HL_DEBUG + "Error getting current binning settings" + error.GetDescription());
  }

  // Setting binning
  if (h_binning != current_h_binning || v_binning != current_v_binning)
  {
    error = camera.SetGigEImageBinningSettings(h_binning, v_binning);
    if (error != FlyCapture2::ErrorType::PGRERROR_OK)
    {
      throw PtGreyException(HL_DEBUG + "Error setting binning settings" + error.GetDescription());
    }
  }
}

void FlyCapImageProvider::setPixelFormat(FlyCapture2::PixelFormat pixel_format)
{
  struct FlyCapture2::GigEImageSettings image_settings = getImageSettings();
  image_settings.pixelFormat = pixel_format;
  FlyCapture2::Error error = camera.SetGigEImageSettings(&image_settings);
  if (error != FlyCapture2::ErrorType::PGRERROR_OK)
  {
    throw PtGreyException(HL_DEBUG + "Error setting image settings: " + error.GetDescription());
  }
}

FlyCapture2::Property getDefaultProperty(FlyCapture2::PropertyType type)
{
  FlyCapture2::Property p;
  p.type = type;
  p.present = true;
  p.absControl = true;
  p.onePush = false;
  p.onOff = true;
  p.autoManualMode = false;  // false -> manual
  p.valueA = 0;
  p.valueB = 0;
  return p;
}

void FlyCapImageProvider::updateProperty(const FlyCapture2::Property& wished_property)
{
  FlyCapture2::Error error;
  // Uploading property to camera
  error = camera.SetProperty(&wished_property, false);
  if (error != FlyCapture2::PGRERROR_OK)
  {
    throw PtGreyException(HL_DEBUG + "Failed to apply a property");
  }
}

void FlyCapImageProvider::applyWishedProperties()
{
  FlyCapture2::Property shutter_prop = getDefaultProperty(FlyCapture2::PropertyType::SHUTTER);
  FlyCapture2::Property gain_prop = getDefaultProperty(FlyCapture2::PropertyType::GAIN);
  FlyCapture2::Property frame_rate_prop = getDefaultProperty(FlyCapture2::PropertyType::FRAME_RATE);
  shutter_prop.absValue = shutter;
  gain_prop.absValue = gain;
  frame_rate_prop.absValue = frame_rate;
  std::cout << "Shutter" << std::endl;
  updateProperty(shutter_prop);
  std::cout << "Gain" << std::endl;
  updateProperty(gain_prop);
  std::cout << "FPS" << std::endl;
  updateProperty(frame_rate_prop);
}

FlyCapture2::GigEImageSettings FlyCapImageProvider::getImageSettings()
{
  FlyCapture2::GigEImageSettings settings;
  FlyCapture2::Error error = camera.GetGigEImageSettings(&settings);
  if (error != FlyCapture2::ErrorType::PGRERROR_OK)
  {
    throw PtGreyException(HL_DEBUG + error.GetDescription());
  }
  return settings;
}

}  // namespace hl_monitoring
