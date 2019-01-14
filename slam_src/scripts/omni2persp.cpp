#include <gflags/gflags.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "FishCam.h"
#include "undistort.h"

DEFINE_string(calib, "wide/calib_results.txt", "Path to calibration file");
DEFINE_string(input, "wide/hostel.MP4", "Name of the input distorted video");
DEFINE_string(output, "wide/hostel_out1.avi",
              "Name of the output undistorted video");

int main(int argc, char **argv) {
  google::SetUsageMessage("undistort --help");
  google::SetVersionString("1.0.0");
  google::ParseCommandLineFlags(&argc, &argv, true);

  std::string s = FLAGS_calib;
  std::string file_or_video_name = FLAGS_input;
  std::string out_video = FLAGS_output;
  FishOcam f;
  f.init(s);
  cv::VideoCapture inputVideo(file_or_video_name);
  cv::VideoWriter outputVideo;
  int ex = static_cast<int>(inputVideo.get(CV_CAP_PROP_FOURCC));
  cv::Size S = cv::Size(f.wout, f.hout);

  int waitTime = 0;

  if (!(inputVideo.isOpened())) {
    std::cerr << "Invalid video file\n";
    return -1;
  }

  outputVideo.open(out_video, ex, inputVideo.get(CV_CAP_PROP_FPS), S, true);

  if (!outputVideo.isOpened()) {
    std::cout << "Could not open the output video for write: \n";
    return -1;
  }

  int grabber_counter(0), framid(0);
  while (1) {
    grabber_counter += 1;
    cv::Mat image;
    inputVideo.read(image);
    if (image.empty()) {
      if (grabber_counter >= 200) break;
      continue;
    }
    std::cout << "\rFrame : " << framid << std::flush;
    grabber_counter = 0;
    framid++;

    cv::Mat image2;
    f.WarpImage(image, image2);
    outputVideo << image2;
  }
  return 0;
}
