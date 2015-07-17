#include <iostream>
#include <opencv2/opencv.hpp>

#include "SkeletonReceiver.h"

SkeletonReceiver::SkeletonReceiver(unsigned short port_num)
{
  port = port_num;
  memset(&sockAddrIn, 0, sizeof(sockAddrIn));
  sockAddrIn.sin_port = htons(port);
  sockAddrIn.sin_family = AF_INET;
  sockAddrIn.sin_addr.s_addr = htonl(INADDR_ANY);

  if (WSAStartup(MAKEWORD(2, 0), &wsadata) != 0)
  {
    std::cout << "WSAStartup Failed." << std::endl;
    exit(-1);
  }

  sockRecv = socket(AF_INET, SOCK_DGRAM, 0);
  if (INVALID_SOCKET == sockRecv)
  {
    exit(-2);
  }

  if (SOCKET_ERROR == bind(sockRecv, (const struct sockaddr*) &sockAddrIn,
    sizeof(sockAddrIn)))
  {
    closesocket(sockRecv);
    exit(-3);
  }

  return;
}

SkeletonReceiver::~SkeletonReceiver()
{
  closesocket(sockRecv);

  WSACleanup();

  return;
}

void SkeletonReceiver::receive(SendingSkeleton &out)
{
  memset((void*)&out, 0, sizeof(SendingSkeleton));
  int ilen = recvfrom(sockRecv, (char*)&out, sizeof(SendingSkeleton)/sizeof(char), 0, NULL, NULL);
  if (ilen > 0)
  {
    std::cout << "Data Received:: ID -> " << (int)out.id << std::endl <<
      "\t1st joint position (" <<
      out.joints[0].Position.X << ", " <<
      out.joints[0].Position.Y << ", " <<
      out.joints[0].Position.Z << ")" <<
      std::endl;
  }
  return;
}
