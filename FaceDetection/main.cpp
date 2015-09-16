/**
 * ref: about face detection
 *   Face detection -> http://opencv.jp/sample/object_detection.html
 *   cv2 tutorial -> http://docs.opencv.org/doc/tutorials/objdetect/cascade_classifier/cascade_classifier.html
 *   What is ROI? -> http://www.eml.ele.cst.nihon-u.ac.jp/~momma/wiki/wiki.cgi/OpenCV/ROI%E3%81%AE%E4%BD%BF%E3%81%84%E6%96%B9.html
 */

#include <iostream>

#include<iostream>
#include<opencv2/opencv.hpp>
#include<Kinect.h>
#include<Windows.h>

const char *kFaceDetectorData[] = {
  "C:\\OpenCV2.4.11\\build\\share\\OpenCV\\haarcascades\\haarcascade_frontalface_default.xml",
  "C:\\OpenCV2.4.11\\build\\share\\OpenCV\\haarcascades\\haarcascade_frontalface_alt.xml",
  "C:\\OpenCV2.4.11\\build\\share\\OpenCV\\haarcascades\\haarcascade_frontalface_alt2.xml"
};

//------------------------------------------------------------------------------
template<class Interface>
inline void SafeRelease( Interface **ppInterfaceToRelease )
{
  if (*ppInterfaceToRelease != NULL)
  {
    (*ppInterfaceToRelease)->Release();

    (*ppInterfaceToRelease) = NULL;
  }
}

//------------------------------------------------------------------------------
class FaceDetecter
{
public:
  FaceDetecter(const char* data);
  ~FaceDetecter();

  void check(const cv::Mat& src, std::vector<cv::Rect>& faces);

private:
  cv::CascadeClassifier face_cascade_;
};

//------------------------------------------------------------------------------
FaceDetecter::FaceDetecter(const char* data)
{
  if (!face_cascade_.load(data))
  {
    std::cerr << "Failed to load file: " << data << std::endl;
    exit(-1);
  }
  return;
}

//------------------------------------------------------------------------------
FaceDetecter::~FaceDetecter()
{
  return;
}

//------------------------------------------------------------------------------
void FaceDetecter::check(const cv::Mat& src, std::vector<cv::Rect>& faces)
{
  cv::Mat src_gray;
  cv::cvtColor(src, src_gray, CV_RGB2GRAY);
  cv::equalizeHist(src_gray, src_gray);

  face_cascade_.detectMultiScale(src_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30,30));

  return;
}

//------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  FaceDetecter detector(kFaceDetectorData[0]);
  std::vector<cv::Rect> faces;

  cv::setUseOptimized( true );

  // Sensor
  IKinectSensor* pSensor;
  HRESULT hResult = S_OK;
  hResult = GetDefaultKinectSensor( &pSensor );
  if( FAILED( hResult ) ){
    std::cerr << "Error : GetDefaultKinectSensor" << std::endl;
    return -1;
  }

  hResult = pSensor->Open();
  if( FAILED( hResult ) ){
    std::cerr << "Error : IKinectSensor::Open()" << std::endl;
    return -1;
  }

  // Source
  IColorFrameSource* pColorSource;
  hResult = pSensor->get_ColorFrameSource( &pColorSource );
  if( FAILED( hResult ) ){
    std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
    return -1;
  }

  // Reader
  IColorFrameReader* pColorReader;
  hResult = pColorSource->OpenReader( &pColorReader );
  if( FAILED( hResult ) ){
    std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
    return -1;
  }

  // Description
  IFrameDescription* pDescription;
  hResult = pColorSource->get_FrameDescription( &pDescription );
  if( FAILED( hResult ) ){
    std::cerr << "Error : IColorFrameSource::get_FrameDescription()" << std::endl;
    return -1;
  }

  int width = 0;
  int height = 0;
  pDescription->get_Width( &width ); // 1920
  pDescription->get_Height( &height ); // 1080
  unsigned int bufferSize = width * height * 4 * sizeof( unsigned char );

  cv::Mat bufferMat( height, width, CV_8UC4 );
  cv::Mat colorMat( height / 2, width / 2, CV_8UC4 );
  cv::namedWindow( "Color" );

  while( 1 ){
    // Frame
    IColorFrame* pColorFrame = nullptr;
    hResult = pColorReader->AcquireLatestFrame( &pColorFrame );
    if( SUCCEEDED( hResult ) ){
      hResult = pColorFrame->CopyConvertedFrameDataToArray( bufferSize, reinterpret_cast<BYTE*>( bufferMat.data ), ColorImageFormat::ColorImageFormat_Bgra );
      if( SUCCEEDED( hResult ) ){
        detector.check(bufferMat, faces);

        // View detection result
        for (size_t i = 0; i < faces.size(); i++)
        {
          cv::Point center(faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5);
          cv::ellipse(bufferMat, center, cv::Size(faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, cv::Scalar(255, 0, 255), 4, 8, 0);
        }

        cv::resize( bufferMat, colorMat, cv::Size(), 0.5, 0.5 );
      }
    }
    SafeRelease( &pColorFrame );

    cv::imshow( "Color", colorMat );

    if( cv::waitKey( 30 ) == VK_ESCAPE ){
      break;
    }
  }

  SafeRelease( &pColorSource );
  SafeRelease( &pColorReader );
  SafeRelease( &pDescription );
  if( pSensor ){
    pSensor->Close();
  }
  SafeRelease( &pSensor );
  cv::destroyAllWindows();

  return 0;
}