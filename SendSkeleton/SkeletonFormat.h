#ifndef SKELETON_FORMAT_H_
#define SKELETON_FORMAT_H_

#include <stdint.h>
#include <sstream>
#include <Kinect.h>

typedef struct
{
  int id;
  Joint joints[JointType::JointType_Count];
  int face_state;
}
SendingSkeleton;

inline void makeSendingSkeleton(
  int id, IBody& src, int face_state, SendingSkeleton& dst)
{
  dst.id = id;
  Joint joint[JointType::JointType_Count];
  HRESULT ret = src.GetJoints(JointType::JointType_Count, joint);
  if (SUCCEEDED(ret))
  {
    for (int i = 0; i < JointType::JointType_Count; i++)
    {
      dst.joints[i] = joint[i];
    }
  }
  dst.face_state = face_state;
  return;
}

inline size_t ConvertSendingSkeletonToJSON(const SendingSkeleton& in, char* out, bool is_opened_calib_file=false,
  float tx = 0.0, float ty = 0.0, float tz = 0.0, float qx = 0.0, float qy = 0.0, float qz = 0.0, float qw = 0.0)
{
  std::stringstream jointStream;
  for (int i = 0; i < JointType::JointType_Count; i++)
  {
    const Joint& joint = in.joints[i];
    jointStream << "{"
      "\"JointType\":" << joint.JointType << "," <<
      "\"CameraSpacePoint\":{" <<
      "\"X\":" << joint.Position.X << "," <<
      "\"Y\":" << joint.Position.Y << "," <<
      "\"Z\":" << joint.Position.Z << "},"
      "\"TrackingState\":" << joint.TrackingState <<
      "}";
    if (i < JointType::JointType_Count - 1)
    {
      jointStream << ",";
    }
  }
  std::string joints_str(jointStream.str());
  std::stringstream ss;
  ss << "{" <<
    "\"id\":" << (int)in.id << "," <<
    "\"joints\":" << "[" << joints_str << "]" << "," <<
    "\"FaceState\":" << in.face_state;
  if (is_opened_calib_file)
  {
    ss << "," <<
      "\"CameraParam\":{" <<
      "\"T\":{" <<
      "\"X\":" << tx << "," <<
      "\"Y\":" << ty << "," <<
      "\"Z\":" << tz << "},"
      "\"R\":{"
      "\"X\":" << qx << "," <<
      "\"Y\":" << qy << "," <<
      "\"Z\":" << qz << "," <<
      "\"W\":" << qw << "}" <<
      "}";
  }
  ss << "}";
  std::string json(ss.str());
  strcpy_s(out, json.length()+1, json.c_str());
  return json.length();
}

#endif // SKELETON_FORMAT_H_