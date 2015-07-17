#ifndef SKELETON_RECIVER_H_
#define SKELETON_RECIVER_H_

#define PORT_NUM 10000

#include "../SendSkeleton/SkeletonFormat.h"

class SkeletonReceiver
{
public:
  SkeletonReceiver(unsigned short port_num=PORT_NUM);
  ~SkeletonReceiver();
  void receive(SendingSkeleton &out);
private:
  unsigned short port;
  WSADATA wsadata;
  SOCKET sockRecv;
  struct sockaddr_in sockAddrIn;
};

#endif // SKELETON_RECIVER_H_
