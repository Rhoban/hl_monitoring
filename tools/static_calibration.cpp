/**
 * This program open a video stream and a file describing the intrinsic
 * parameters of the camera. It uses manual input to estimate the pose of the
 * camera and then draw the field inside the image for the rest of the video.
 */

#include <hl_communication/utils.h>
#include <hl_monitoring/field.h>
#include <hl_monitoring/manual_pose_solver.h>
#include <hl_monitoring/replay_image_provider.h>
#include <hl_monitoring/replay_viewer.h>
#include <hl_monitoring/utils.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <tclap/CmdLine.h>

#include <fstream>

using namespace hl_monitoring;

class CalibrationViewer : public ReplayViewer
{
public:
  CalibrationViewer(std::unique_ptr<ReplayImageProvider> provider, const Field& field,
                    const IntrinsicParameters& camera_parameters)
    : ReplayViewer(std::move(provider), "CalibrationTool", false, field), intrinsic(camera_parameters), has_calib(false)
  {
    addBinding('c', "Run pose calibration", [this]() { this->runCalibration(); });
  }

  void runCalibration()
  {
    ManualPoseSolver pose_solver(display_img, intrinsic, field);
    has_calib = pose_solver.solve(&pose);
  }
  void paintImg() override
  {
    if (has_calib)
    {
      CameraMetaInformation information;
      information.mutable_camera_parameters()->CopyFrom(intrinsic);
      information.mutable_pose()->CopyFrom(pose);
      field.tagLines(information, &display_img, cv::Scalar(0, 0, 0), 1, 10);
    }
  }

  Field field;
  IntrinsicParameters intrinsic;
  bool has_calib;
  Pose3D pose;
};

int main(int argc, char** argv)
{
  TCLAP::CmdLine cmd("Extract the pose of the camera inside a video using pre-computed intrinsic"
                     "parameters",
                     ' ', "0.9");

  TCLAP::ValueArg<std::string> video_arg("v", "video", "The path to the video", true, "video.avi", "string");
  TCLAP::ValueArg<std::string> output_arg("o", "output", "The output path for the pose of the camera", true, "pose.bin",
                                          "string");
  TCLAP::ValueArg<std::string> intrinsic_arg("i", "intrinsic", "Path to the file containing the intrinsic parameters",
                                             true, "intrinsic.bin", "string");
  TCLAP::ValueArg<std::string> field_arg("f", "field", "The path to the field file containing the dimensions", false,
                                         "field.json", "string");
  cmd.add(video_arg);
  cmd.add(output_arg);
  cmd.add(intrinsic_arg);
  cmd.add(field_arg);

  try
  {
    cmd.parse(argc, argv);
  }
  catch (const TCLAP::ArgException& e)
  {
    std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
  }

  std::unique_ptr<ReplayImageProvider> provider(new ReplayImageProvider(video_arg.getValue()));

  IntrinsicParameters intrinsic;
  std::ifstream in(intrinsic_arg.getValue());
  if (!in.good())
  {
    throw std::runtime_error(HL_DEBUG + " failed to open file '" + intrinsic_arg.getValue() + "'");
  }
  intrinsic.ParseFromIstream(&in);

  Field field;
  if (field_arg.isSet())
  {
    field.loadFile(field_arg.getValue());
  }

  CalibrationViewer calibration(std::move(provider), field, intrinsic);
  calibration.run();
  if (calibration.has_calib)
  {
    hl_communication::writeToFile(output_arg.getValue(), calibration.pose);
  }
}
