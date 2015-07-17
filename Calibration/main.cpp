#include<iostream>
#include<opencv2/opencv.hpp>
#include<Kinect.h>

#include"Calibration.h"

int main(int argc, char **argv)
{
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
  
  Calibration calibration(pSensor);
  calibration.run();

	if( pSensor ){
		pSensor->Close();
	}
	SafeRelease( &pSensor );

  return 0;
}