#ifndef _CALIBRATION_H_
#define _CALIBRATION_H_

#define CAMERA_RESOLUTION_X 512
#define CAMERA_RESOLUTION_Y 424

#define PATTERN_COLS 5
#define PATTERN_ROWS 6
#define PATTERN_WIDTH 0.178 // [m]

#define INVALID_PIXEL_VALUE 0

#define USING_KINECT_V2

#include <cstdint>
#include <Eigen/Eigen>

//typedef uint16_t* FRAME;
typedef Eigen::Vector2i FRAME_INDEX;
typedef Eigen::Vector3f POINT3;
typedef Eigen::Matrix3f MATRIX3;
typedef Eigen::Quaternionf QUATERNION;

typedef int FRAME_LENGTH;
typedef unsigned int FRAME_PIXEL_SIZE;
typedef enum {
  INFRARED,
  COLOR,
  DEPTH
} FRAME_TYPE;
typedef struct {
  FRAME_TYPE type;
  FRAME_LENGTH width;
  FRAME_LENGTH height;
} FRAME;

#ifdef USING_KINECT_V2
typedef IKinectSensor* SENSOR;
#endif

class Calibration
{
public:
  Calibration(SENSOR camera);
  ~Calibration();

  void run();

private:
  FRAME depth_frame;
  FRAME frame;

  SENSOR sensor;

  POINT3 translation;
  QUATERNION rotation;

  void core_calcuration(cv::Mat& img, cv::Mat& depth_img);
  void exportXML();

#ifdef USING_KINECT_V2
	IDepthFrameSource* pDepthSource;
	IDepthFrameReader* pDepthReader;
	IFrameDescription* pDepthDescription;
  IInfraredFrameSource* pInfraredSource;
  IInfraredFrameReader* pInfraredReader;
  IColorFrameSource* pColorSource;
  IColorFrameReader* pColorReader;
  IFrameDescription* pFrameDescription;
#endif
};

template<class Interface>
inline void SafeRelease( Interface **ppInterfaceToRelease )
{
    if (*ppInterfaceToRelease != NULL)
    {
        (*ppInterfaceToRelease)->Release();

        (*ppInterfaceToRelease) = NULL;
    }
}

#endif
