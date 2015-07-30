/**
 * \author Masatomo Onishi
 */

#include <Kinect.h>

#include <Windows.h>

#include "SkeletonReceiver.h"

int main(int argc, char **argv)
{
  SkeletonReceiver instance;
  while (1)
  {
    instance.receive();
  }
  
  return 0;
}