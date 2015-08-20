#include<iostream>
#include<opencv2/opencv.hpp>
#include<Kinect.h>

#include"Calibration.h"

void usage()
{
  std::cout << "Usage:" << std::endl;
  std::cout << "\tCalibration.exe [pattern_width]" << std::endl;
  return;
}

int main(int argc, char **argv)
{
  if (argc != 2)
  {
    usage();
    return -1;
  }
  float checker_width = (float)atof(argv[1]);

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
  
  Calibration calibration(pSensor, checker_width);
  calibration.run();

	if( pSensor ){
		pSensor->Close();
	}
	SafeRelease( &pSensor );

  return 0;
}