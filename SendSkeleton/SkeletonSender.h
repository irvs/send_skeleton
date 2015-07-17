#ifndef SKELETON_SENDER_H_
#define SKELETON_SENDER_H_

#define PORT_NUM 10000
#define QUEUE_SIZE 6

#include <queue>
#include "SkeletonFormat.h"

class SkeletonSender
{
public:
  SkeletonSender(const char* ip_addr, unsigned short port_num=PORT_NUM);
  ~SkeletonSender();
  void sendOnce();
  void set(const SendingSkeleton& data);

private:
  char *ip;
  unsigned short port;
  WSADATA wsadata;
  SOCKET sockSend;
  struct sockaddr_in sockAddrIn;

  bool isOpened = false;
  float tx, ty, tz;
  float qx, qy, qz, qw;

  std::queue<SendingSkeleton> queue;
};


#endif //SKELETON_SENDER_H_
