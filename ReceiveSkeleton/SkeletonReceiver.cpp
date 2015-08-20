#include <iostream>
#include <string>

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
  std::cout << "Host IP: " << htonl(INADDR_ANY) << std::endl;
  std::cout << "Port : " << port << std::endl;

  return;
}

SkeletonReceiver::~SkeletonReceiver()
{
  closesocket(sockRecv);

  WSACleanup();

  return;
}

void SkeletonReceiver::receive()
{
  char *recv_data;
  recv_data = (char*)malloc(sizeof(char) * 16383);
  int ilen = recvfrom(sockRecv, (char*)recv_data, 16383/sizeof(char), 0, NULL, NULL);
  if (ilen > 0)
  {
	  std::cout << "Received data size: " << ilen << std::endl;
  }
  free(recv_data);
  return;
}
