#include <iostream>
#include <memory>
#include <chrono>
#include <System.h>
#include "cxxopts.h"

using namespace std;

int main(int argc, const char *argv[]) {
  try {
    cxxopts::Options options("os2-service", "ORB-SLAM2 service executable");
    options.add_options()
        ("s,settings", "camera settings file (YAML)", cxxopts::value<std::string>()->default_value("webcam.yaml"))
        ("v,vocab", "vocabulary file (TXT)", cxxopts::value<std::string>()->default_value("ORBvoc.txt"))
        ("c,cap", "capture input", cxxopts::value<std::string>()->default_value("0"))
        ("help", "print help");
    auto result = options.parse(argc, argv);
    if (result.count("help")) {
      std::cout << options.help({"", "Group"}) << std::endl;
      return 0;
    }

    string settingsPath = result["s"].as<std::string>();
    string vocabPath = result["v"].as<std::string>();
    string capPath = result["c"].as<std::string>();
    std::shared_ptr<cv::VideoCapture> cap;

    if (capPath.empty()) cap = std::make_shared<cv::VideoCapture>(0);
    else {
      try { cap = std::make_shared<cv::VideoCapture>(std::stoi(capPath)); }
      catch (...) { cerr << "unable to parse the cap as integer" << endl; }
      if (!cap) cap = std::make_shared<cv::VideoCapture>(capPath);
    }

    if (!cap->isOpened()) {
      cerr << "unable to open any capture input for SLAM" << endl;
      return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(vocabPath, settingsPath, ORB_SLAM2::System::MONOCULAR, true);

    // Main loop
    cv::Mat Tcw;
    while (true) {
      cv::Mat im;
      *cap >> im;
      if (im.empty())
        break;

      auto now =
          std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::system_clock::now().time_since_epoch()).count();

      // Pass the image to the SLAM system
      Tcw = SLAM.TrackMonocular(im, now/1000.0);

      /* This can write each image with its position to a file if you want
      if (!Tcw.empty())
      {
          cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
          cv::Mat twc = -Rwc*Tcw.rowRange(0, 3).col(3);
          std::ostringstream stream;
          //stream << "imgs/" << Rwc.at<float>(0, 0) << " " << Rwc.at<float>(0, 1) << " " << Rwc.at<float>(0, 2) << " " << twc.at<float>(0) << " " <<
          //	Rwc.at<float>(1, 0) << " " << Rwc.at<float>(1, 1) << " " << Rwc.at<float>(1, 2) << " " << twc.at<float>(1) << " " <<
              //Rwc.at<float>(2, 0) << " " << Rwc.at<float>(2, 1) << " " << Rwc.at<float>(2, 2) << " " << twc.at<float>(2) << ".jpg";
          stream << "imgs/" << curNow << ".jpg";
          string fileName = stream.str();
          cv::imwrite(fileName, im);
      }
      */

      cv::imshow("input", im);
      if (cv::waitKey(1) >= 0)
        break;
    }

    SLAM.Shutdown();
    cap->release();
  } catch (const cxxopts::OptionException &e) {
    std::cout << "error parsing options: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}