#include <hl_communication/utils.h>
#include <hl_communication/camera.pb.h>
#include <hl_monitoring/replay_image_provider.h>

#include <tclap/CmdLine.h>

using namespace hl_communication;
using namespace hl_monitoring;

int main(int argc, char** argv)
{
  TCLAP::CmdLine cmd("Combine multiple type of files to create meta information for a video", ' ', "0.9");

  TCLAP::ValueArg<std::string> video_arg("v", "video",
                                         "Path to a video, if specified, use the video as a "
                                         "source to create the time_stamps for each frame",
                                         false, "", "path", cmd);
  TCLAP::ValueArg<std::string> intrinsic_arg("i", "intrinsic",
                                             "Path to the file describing intrinsic parameters "
                                             "of the camera.",
                                             false, "", "path", cmd);
  TCLAP::ValueArg<std::string> meta_arg("m", "meta-information", "Path to a file containing initial meta-information",
                                        false, "", "path", cmd);
  TCLAP::ValueArg<std::string> pose_arg("p", "pose", "Path to the file describing the pose of the camera", false, "",
                                        "path", cmd);
  TCLAP::ValueArg<std::string> output_arg("o", "output", "The output path for the meta_information", false,
                                          "meta_information.bin", "path", cmd);
  TCLAP::SwitchArg force_switch("f", "force", "Allows to overwrite existing data in meta-information", cmd, false);
  TCLAP::SwitchArg show_id_arg("", "show-id", "Show id from meta-information", cmd);
  TCLAP::SwitchArg guess_start_arg("", "guess-start",
                                   "Use first frame of meta-information to guess the start of the video", cmd);
  try
  {
    cmd.parse(argc, argv);
  }
  catch (const TCLAP::ArgException& e)
  {
    std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
    exit(EXIT_FAILURE);
  }

  VideoMetaInformation information;

  // Allow to overwrite initial object only if explicit or if no file will be written
  bool modification_allowed = force_switch.getValue() || !output_arg.isSet();

  if (meta_arg.getValue() != "")
  {
    readFromFile(meta_arg.getValue(), &information);
  }
  if (pose_arg.getValue() != "")
  {
    if (!modification_allowed && information.has_default_pose())
    {
      throw std::runtime_error(HL_DEBUG + "video meta information already contains default pose."
                                          " use -f to overwrite");
    }
    readFromFile(pose_arg.getValue(), information.mutable_default_pose());
  }
  if (intrinsic_arg.getValue() != "")
  {
    if (!modification_allowed && information.has_camera_parameters())
    {
      throw std::runtime_error(HL_DEBUG + "video meta information already contains camera_parameters."
                                          " use -f to overwrite");
    }
    readFromFile(intrinsic_arg.getValue(), information.mutable_camera_parameters());
  }
  if (video_arg.getValue() != "")
  {
    if (!modification_allowed && information.frames_size() > 0)
    {
      throw std::runtime_error(HL_DEBUG + "video meta information already contains frame entries."
                                          " use -f to overwrite");
    }
    ReplayImageProvider video(video_arg.getValue());
    video.setDefaultMetaInformation();
    information = video.getMetaInformation();
  }
  if (guess_start_arg.getValue())
  {
    VideoSourceID* source = information.mutable_source_id();
    if (source->has_utc_start())
      throw std::logic_error(HL_DEBUG + " source has already a start specified");
    else if (information.frames_size() == 0)
      throw std::logic_error(HL_DEBUG + " no frames available");
    source->set_utc_start(information.frames(0).utc_ts());
  }
  if (show_id_arg.getValue())
  {
    if (information.has_source_id())
    {
      std::cout << "Source id: " << information.source_id() << std::endl;
    }
    else
    {
      std::cout << "Meta info has no id" << std::endl;
    }
  }
  if (output_arg.isSet())
  {
    writeToFile(output_arg.getValue(), information);
  }
}
