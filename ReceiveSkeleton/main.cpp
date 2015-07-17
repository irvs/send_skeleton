/**
 * \author Masatomo Onishi
 */

#include <Kinect.h>
#include <opencv2/opencv.hpp>

#include <Windows.h>

#include "SkeletonReceiver.h"

int main(int argc, char **argv)
{
  SkeletonReceiver instance;
  SendingSkeleton data;
  while (1)
  {
    instance.receive(data);
  }
  
  return 0;
}