/**
 * author Masatomo Onishi
 * I got this source from https://github.com/UnaNancyOwen/Kinect2Sample
 * I modified a bit.
 */

#include <Kinect.h>
#include <opencv2/opencv.hpp>

#include <Windows.h>

#include "SkeletonSender.h"

const char* IP_LIST[] = {
  "192.168.4.155",
  "192.168.4.156",
  "192.168.4.157",
  "192.168.4.158",
  "192.168.4.159",
  "192.168.4.160"
};

template<class Interface> inline void SafeRelease( Interface *& pInterfaceToRelease )
{
	if( pInterfaceToRelease != NULL ){
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

int main(int argc, char **argv)
{
  if (argc != 2)
  {
    std::cout << "Please set IP address for first argument." << std::endl;
    std::cout << "IP list ===" << std::endl;
    for (int i = 0; i < 6; i++)
    {
      std::cout << i+1 << ": " << IP_LIST[i] << std::endl;
    }
    return -1;
  }
  int ip_index = atoi(argv[1]) - 1;
  if (ip_index < 0 || ip_index >= 6)
  {
    std::cout << "Given invalid argument. Input the ID of sending PC." << std::endl;
    std::cout << "IP list ===" << std::endl;
    for (int i = 0; i < 6; i++)
    {
      std::cout << i+1 << ": " << IP_LIST[i] << std::endl;
    }
    return -2;
  }
  SkeletonSender instance(IP_LIST[ip_index]);

  cv::setUseOptimized(true);

  // Sensor
  IKinectSensor* pSensor;
  HRESULT hResult = S_OK;
  hResult = GetDefaultKinectSensor(&pSensor);
  if (FAILED(hResult)) {
    std::cerr << "Error : GetDefaultKinectSensor" << std::endl;
    return -1;
  }

  hResult = pSensor->Open();
  if (FAILED(hResult)) {
    std::cerr << "Error : IKinectSensor::Open()" << std::endl;
    return -1;
  }

  // Source
  IColorFrameSource* pColorSource;
  hResult = pSensor->get_ColorFrameSource(&pColorSource);
  if (FAILED(hResult)) {
    std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
    return -1;
  }

  IBodyFrameSource* pBodySource;
  hResult = pSensor->get_BodyFrameSource(&pBodySource);
  if (FAILED(hResult)) {
    std::cerr << "Error : IKinectSensor::get_BodyFrameSource()" << std::endl;
    return -1;
  }

  // Reader
  IColorFrameReader* pColorReader;
  hResult = pColorSource->OpenReader(&pColorReader);
  if (FAILED(hResult)) {
    std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
    return -1;
  }

  IBodyFrameReader* pBodyReader;
  hResult = pBodySource->OpenReader(&pBodyReader);
  if (FAILED(hResult)) {
    std::cerr << "Error : IBodyFrameSource::OpenReader()" << std::endl;
    return -1;
  }

  // Description
  IFrameDescription* pDescription;
  hResult = pColorSource->get_FrameDescription(&pDescription);
  if (FAILED(hResult)) {
    std::cerr << "Error : IColorFrameSource::get_FrameDescription()" << std::endl;
    return -1;
  }

  int width = 0;
  int height = 0;
  pDescription->get_Width(&width); // 1920
  pDescription->get_Height(&height); // 1080
  unsigned int bufferSize = width * height * 4 * sizeof(unsigned char);

  cv::Mat bufferMat(height, width, CV_8UC4);
  cv::Mat bodyMat(height / 2, width / 2, CV_8UC4);
  cv::namedWindow("Body");

  // Color Table
  cv::Vec3b color[BODY_COUNT];
  color[0] = cv::Vec3b(255, 0, 0);
  color[1] = cv::Vec3b(0, 255, 0);
  color[2] = cv::Vec3b(0, 0, 255);
  color[3] = cv::Vec3b(255, 255, 0);
  color[4] = cv::Vec3b(255, 0, 255);
  color[5] = cv::Vec3b(0, 255, 255);

  // Coordinate Mapper
  ICoordinateMapper* pCoordinateMapper;
  hResult = pSensor->get_CoordinateMapper(&pCoordinateMapper);
  if (FAILED(hResult)) {
    std::cerr << "Error : IKinectSensor::get_CoordinateMapper()" << std::endl;
    return -1;
  }

  while (1) {
    // Frame
    IColorFrame* pColorFrame = nullptr;
    hResult = pColorReader->AcquireLatestFrame(&pColorFrame);
    if (SUCCEEDED(hResult)) {
      hResult = pColorFrame->CopyConvertedFrameDataToArray(bufferSize, reinterpret_cast<BYTE*>(bufferMat.data), ColorImageFormat::ColorImageFormat_Bgra);
      if (SUCCEEDED(hResult)) {
        cv::resize(bufferMat, bodyMat, cv::Size(), 0.5, 0.5);
      }
    }

    IBodyFrame* pBodyFrame = nullptr;
    hResult = pBodyReader->AcquireLatestFrame(&pBodyFrame);
    if (SUCCEEDED(hResult)) {
      IBody* pBody[BODY_COUNT] = { 0 };
      hResult = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, pBody);
      if (SUCCEEDED(hResult)) {
        for (int count = 0; count < BODY_COUNT; count++) {
          BOOLEAN bTracked = false;
          hResult = pBody[count]->get_IsTracked(&bTracked);
          if (SUCCEEDED(hResult) && bTracked) {
            Joint joint[JointType::JointType_Count];
            hResult = pBody[count]->GetJoints(JointType::JointType_Count, joint);
            if (SUCCEEDED(hResult)) {
              // Joint
              for (int type = 0; type < JointType::JointType_Count; type++) {
                ColorSpacePoint colorSpacePoint = { 0 };
                pCoordinateMapper->MapCameraPointToColorSpace(joint[type].Position, &colorSpacePoint);
                int x = static_cast<int>(colorSpacePoint.X);
                int y = static_cast<int>(colorSpacePoint.Y);
                if ((x >= 0) && (x < width) && (y >= 0) && (y < height)) {
                  cv::circle(bufferMat, cv::Point(x, y), 5, static_cast<cv::Scalar>(color[count]));
                }
              }
              // Send Position by UDP protocol
              SendingSkeleton data;
              makeSendingSkeleton(count, *pBody[count], data);
              instance.set(data);
            }
          }
        }
        instance.sendOnce();
        cv::resize(bufferMat, bodyMat, cv::Size(), 0.5, 0.5);
      }
      for (int count = 0; count < BODY_COUNT; count++) {
        SafeRelease(pBody[count]);
      }
    }

    cv::imshow("Body", bodyMat);

    SafeRelease(pColorFrame);
    SafeRelease(pBodyFrame);

    if (cv::waitKey(100) == VK_ESCAPE) {
      break;
    }
  }

  SafeRelease(pColorSource);
  SafeRelease(pBodySource);
  SafeRelease(pColorReader);
  SafeRelease(pBodyReader);
  SafeRelease(pDescription);
  SafeRelease(pCoordinateMapper);
  if (pSensor) {
    pSensor->Close();
  }
  SafeRelease(pSensor);
  cv::destroyAllWindows();

  return 0;
}