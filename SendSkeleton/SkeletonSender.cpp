#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <WinSock2.h>

#include <iostream>

#include "tinyxml2.h"

#include "SkeletonSender.h"

#define IP_ADDR_LENGTH 16

SkeletonSender::SkeletonSender(const char* ip_addr, unsigned short port_num)
{
  ip = (char*)malloc(sizeof(char) * IP_ADDR_LENGTH);
  strcpy_s(ip, IP_ADDR_LENGTH, ip_addr);
  port = port_num;

  if (WSAStartup(MAKEWORD(2, 0), &wsadata) != 0)
  {
    exit(-1);
  }

  memset(&sockAddrIn, 0, sizeof(sockAddrIn));
  sockAddrIn.sin_addr.s_addr = inet_addr(ip);
  sockAddrIn.sin_port = htons(port);
  sockAddrIn.sin_family = AF_INET;

  sockSend = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockSend == INVALID_SOCKET)
  {
    std::cout << "SkeletonSender: Invalid socket. exitting ..." << std::endl;
    exit(-2);
  }


  tinyxml2::XMLDocument xml;
  if (xml.LoadFile("../parameter.xml") == tinyxml2::XML_NO_ERROR)
  {
    std::cout << "Succeed to calibration file." << std::endl;

    tinyxml2::XMLElement *extrinsic_parameter = xml.FirstChildElement("extrinsic_parameter");
    tinyxml2::XMLElement *translation = extrinsic_parameter->FirstChildElement("translation");
    tinyxml2::XMLElement *rotation = extrinsic_parameter->FirstChildElement("rotation");

    tinyxml2::XMLElement *t;
    t = translation->FirstChildElement("param");
    while (t != NULL)
    {
      if (t->Attribute("name", "tx"))
      {
        tx = (float)atof(t->GetText());
      }
      else if (t->Attribute("name", "ty"))
      {
        ty = (float)atof(t->GetText());
      }
      else if (t->Attribute("name", "tz"))
      {
        tz = (float)atof(t->GetText());
      }
      t = t->NextSiblingElement("param");
    }

    tinyxml2::XMLElement *q;
    q = rotation->FirstChildElement("param");
    while (q != NULL)
    {
      if (q->Attribute("name", "qx"))
      {
        qx = (float)atof(q->GetText());
      }
      else if (q->Attribute("name", "qy"))
      {
        qy = (float)atof(q->GetText());
      }
      else if (q->Attribute("name", "qz"))
      {
        qz = (float)atof(q->GetText());
      }
      else if (q->Attribute("name", "qw"))
      {
        qw = (float)atof(q->GetText());
      }
      q = q->NextSiblingElement("param");
    }
    isOpened = true;
  }

  return;
}

SkeletonSender::~SkeletonSender()
{
  free((void*)ip);

  closesocket(sockSend);

  WSACleanup();

  return;
}

void SkeletonSender::sendOnce()
{
  SendingSkeleton data;
  char json[16383];
  while (!queue.empty())
  {
    data = queue.front();
    queue.pop();
    std::cout << "sending skeleton:: ID -> " << data.id << std::endl;
    int len = ConvertSendingSkeletonToJSON(data, json, isOpened,
      tx, ty, tz, qx, qy, qz, qw);
    sendto(sockSend, json, len,
      0, (const struct sockaddr *)&sockAddrIn, sizeof(sockAddrIn));
  }
  return;
}

void SkeletonSender::set(const SendingSkeleton& data)
{
  queue.push(data);
  return;
}