#ifndef FACE_DETECTOR_H
#define FACE_DETECTOR_H

#include<opencv2/opencv.hpp>

class FaceDetector
{
public:
  FaceDetector(const char* data);
  ~FaceDetector();

  void check(const cv::Mat& src, std::vector<cv::Rect>& faces);

private:
  cv::CascadeClassifier face_cascade_;
};

#endif  // FACE_DETECTOR_H
