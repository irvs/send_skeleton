#include "FaceDetector.h"

//------------------------------------------------------------------------------
FaceDetector::FaceDetector(const char* data)
{
  if (!face_cascade_.load(data))
  {
    std::cerr << "Failed to load file: " << data << std::endl;
    exit(-1);
  }
  return;
}

//------------------------------------------------------------------------------
FaceDetector::~FaceDetector()
{
  return;
}

//------------------------------------------------------------------------------
void FaceDetector::check(const cv::Mat& src, std::vector<cv::Rect>& faces)
{
  cv::Mat src_gray;
  cv::cvtColor(src, src_gray, CV_RGB2GRAY);
  cv::equalizeHist(src_gray, src_gray);

  face_cascade_.detectMultiScale(src_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30,30));

  return;
}
