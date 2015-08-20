#include <limits>
#include <vector>
#include <algorithm>
#include <sstream>
#include <opencv2/opencv.hpp>

#include <Kinect.h>

#include "tinyxml2.h"
#include "Calibration.h"

inline int find_corner(const cv::Mat& image, const cv::Size pattern_size,
    std::vector<cv::Point2f>& corners, bool& pattern_found)
{
  cv::Mat gray(image.rows, image.cols, CV_8UC1);
  switch(image.type())
  {
  case CV_8UC1:
    gray = image;
    break;
  case CV_8UC3:
    cv::cvtColor(image, gray, CV_BGR2GRAY);
    break;
  case CV_8UC4:
    cv::cvtColor(image, gray, CV_BGRA2GRAY);
    break;
  case CV_16UC1:
    image.convertTo(gray, CV_8UC1);
    break;
  default:
    std::cerr << "Unexpected image type." << std::endl;
    return -1;
  }
  pattern_found = cv::findChessboardCorners(gray, pattern_size, corners,
      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
  if (pattern_found)
  {
    cv::cornerSubPix(gray, corners, cv::Size(15,15), cv::Size(-1,-1),
        cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 15, 0.01));
  }
  return 0;
}

inline void normalize(cv::Mat &img)
{
  double min, max;
  double block_max;
  switch (img.type())
  {
  case CV_8UC4:
    block_max = 255;
    break;
  case CV_16UC1:
    block_max = 65535;
    break;
  }
  cv::minMaxLoc(img, &min, &max);
  cv::convertScaleAbs(img, img, block_max / (max - min), -min);
  return;
}

inline int ref(FRAME_INDEX index, int width)
{
  return index[0] + index[1] * width;
}

template<class TYPE>
inline void calcurate_error(
  TYPE phi_x, TYPE phi_y, TYPE phi_z, TYPE t_x, TYPE t_y, TYPE t_z,
  const Eigen::Matrix<TYPE, 3, 1>* X_w_Array,
  const Eigen::Matrix<TYPE, 3, 1>* X_c_Array,
  TYPE& error)
{
  Eigen::Matrix<TYPE, 3, 3> rot;
  rot =
      Eigen::AngleAxis<TYPE>(phi_z, Eigen::Matrix<TYPE, 3, 1>::UnitZ())
    * Eigen::AngleAxis<TYPE>(phi_y, Eigen::Matrix<TYPE, 3, 1>::UnitY())
    * Eigen::AngleAxis<TYPE>(phi_x, Eigen::Matrix<TYPE, 3, 1>::UnitX());
  error = 0.0;
  for (int i = 0; i < PATTERN_ROWS*PATTERN_COLS; i++)
  {
    Eigen::Matrix<TYPE, 3, 1> tmp;
    tmp = X_w_Array[i] - (rot.inverse()*(X_c_Array[i] - Eigen::Matrix<TYPE, 3, 1>(t_x, t_y, t_z)));
    error += (TYPE)pow(tmp.norm(), 2.0);
  }
  return;
}

template<class TYPE>
inline void Calibration::draw_axis(
  const Eigen::Matrix<TYPE, 3, 1>& axis,
  const Eigen::Matrix<TYPE, 3, 1>& origin,
  cv::Mat& img,
  const cv::Scalar& color,
  ICoordinateMapper *mapper)
{
  CameraSpacePoint axis_camera;
  CameraSpacePoint origin_camera;
  DepthSpacePoint axis_depthframe;
  DepthSpacePoint origin_depthframe;
  axis_camera.X = (axis*(float)pattern_width + origin)[0];
  axis_camera.Y = (axis*(float)pattern_width + origin)[1];
  axis_camera.Z = (axis*(float)pattern_width + origin)[2];
  origin_camera.X = origin[0];
  origin_camera.Y = origin[1];
  origin_camera.Z = origin[2];
  HRESULT hResult1 = mapper->MapCameraPointToDepthSpace(axis_camera,&axis_depthframe);
  HRESULT hResult2 = mapper->MapCameraPointToDepthSpace(origin_camera,&origin_depthframe);
  if (SUCCEEDED(hResult1)&&SUCCEEDED(hResult2))
  {
    cv::line(img, cv::Point((int)axis_depthframe.X, (int)axis_depthframe.Y),
      cv::Point((int)origin_depthframe.X, (int)origin_depthframe.Y),
      color, 1.5, CV_AA);
  }
  return;
}

void Calibration::core_calcuration(cv::Mat& img, cv::Mat& depth_img)
{
  // Detect corner of chessboard
  std::vector<cv::Point2f> corners;
  bool pattern_found;
  normalize(img);
  find_corner(img, cv::Size(PATTERN_COLS, PATTERN_ROWS),
    corners, pattern_found);

  // Make lookup table for converting from color frame to depth frame
  ICoordinateMapper* pCoordinateMapper;
  HRESULT hResult = sensor->get_CoordinateMapper(&pCoordinateMapper);
  if (FAILED(hResult))
  {
    std::cerr << "Error : IKinectSensor::get_CoordinateMapper()" << std::endl;
    exit(-1);
  }
  DepthSpacePoint* color_to_depth;
  color_to_depth = new DepthSpacePoint [frame.width*frame.height];
  hResult = pCoordinateMapper->MapColorFrameToDepthSpace(
    depth_frame.width*depth_frame.height, reinterpret_cast<UINT16*>(depth_img.data),
    frame.width*frame.height, color_to_depth);
  if (FAILED(hResult))
  {
    std::cerr << "Error : ICoordinateMapper::MapColorFrameToDepthSpace()" << std::endl;
    exit(-1);
  }

  cv::Mat viewer(cv::Size(depth_frame.width, depth_frame.height), CV_8UC4);
  viewer = cv::Mat::zeros(cv::Size(depth_frame.width, depth_frame.height), CV_8UC4);
  for (int color_index = 0; color_index < frame.width*frame.height; color_index++)
  {
    FRAME_INDEX index(
      (int)color_to_depth[color_index].X + 0.5f,
      (int)color_to_depth[color_index].Y + 0.5f);
    if (color_to_depth[color_index].X != INVALID_PIXEL_VALUE &&
      color_to_depth[color_index].Y != INVALID_PIXEL_VALUE &&
      !(index[0] < 0) && index[0] < depth_frame.width &&
      !(index[1] < 0) && index[1] < depth_frame.height)
    {
      viewer.at<cv::Vec4b>(index[1], index[0]) =
        img.at<cv::Vec4b>(color_index / frame.width,
          color_index%frame.width);
    }
  }
  cv::drawChessboardCorners(img, cv::Size(PATTERN_COLS, PATTERN_ROWS),
    (cv::Mat)corners, pattern_found);
  if (!pattern_found)
  {
    return;
  }
	cv::Mat frameMat( frame.height / 2, frame.width / 2, CV_8UC4 );
  cv::resize(img, frameMat, cv::Size(), 0.5, 0.5);
  cv::imshow("Frame", frameMat);
  std::cout << "If you want to reverse corners, push \'s\' on the CV window." << std::endl;
  if ((cv::waitKey(0) & 0x00ff) == 's')
  {
    std::cout << "Array of detected corners are reversed." << std::endl;
    std::reverse(std::begin(corners), std::end(corners));
  }

  // Extract points on chessboard
  FRAME_INDEX chess_corner[4];
  int corner_index;
  const int chess_corner_num[] =
  {
    0,
    (PATTERN_ROWS - 1)*PATTERN_COLS,
    PATTERN_ROWS*PATTERN_COLS - 1,
    PATTERN_COLS - 1
  };
  for (int i = 0; i < 4; i++)
  {
    corner_index = (int)corners[chess_corner_num[i]].x
      + (int)corners[chess_corner_num[i]].y * frame.width;
    chess_corner[i] = FRAME_INDEX(
      (int)color_to_depth[corner_index].X + 0.5f,
      (int)color_to_depth[corner_index].Y + 0.5f);
  }

  int check;
  bool check_flg;
  std::vector<FRAME_INDEX> loop_stack;
  int *check_map;

  check_map = new int[depth_frame.width*depth_frame.height];
  for (int i = 0; i < depth_frame.width*depth_frame.height; i++)
  {
    check_map[i] = 0; // 0: not checked, 1: pending, 2: checked
  }
  loop_stack.clear();
  FRAME_INDEX start_pixel(
    (chess_corner[0]+chess_corner[1]+chess_corner[2]+chess_corner[3])/4);
  loop_stack.push_back(start_pixel);
  check_map[ref(start_pixel, depth_frame.width)] = 1;

  while (!loop_stack.empty())
  {
    FRAME_INDEX check_pixel;
    check_pixel = loop_stack.back();
    loop_stack.pop_back();
    for (int i = 0; i<4; i++)
    {
      check = 0;
      check_flg = true;
      Eigen::Vector2i check_pixel_direction = check_pixel - chess_corner[i];
      Eigen::Vector2i chess_edge = chess_corner[(i+1)%4] - chess_corner[i];
      check = (check_pixel_direction[1] * chess_edge[0]
        - check_pixel_direction[0]*chess_edge[1]);
      if (check > 0)
      {
        check_flg = false;
        break;
      }
    }
    if (check_flg)
    {
      check_map[ref(check_pixel, depth_frame.width)] = 2;
      if (!(check_map[check_pixel[0]+1+check_pixel[1]*depth_frame.width] > 0) )
      {
        loop_stack.push_back(FRAME_INDEX(check_pixel[0]+1, check_pixel[1]));
      }
      if (!(check_map[check_pixel[0]-1+check_pixel[1]*depth_frame.width] > 0) )
      {
        loop_stack.push_back(FRAME_INDEX(check_pixel[0]-1, check_pixel[1]));
      }
      if (!(check_map[check_pixel[0]+(check_pixel[1]+1)*depth_frame.width] > 0) )
      {
        loop_stack.push_back(FRAME_INDEX(check_pixel[0], check_pixel[1]+1));
      }
      if (!(check_map[check_pixel[0]+(check_pixel[1]-1)*depth_frame.width] > 0) )
      {
        loop_stack.push_back(FRAME_INDEX(check_pixel[0], check_pixel[1]-1));
      }
    }
  }
  int camera_points_num = depth_frame.width * depth_frame.height;
  CameraSpacePoint* depth_to_camera;
  depth_to_camera = new CameraSpacePoint[camera_points_num];
  hResult = pCoordinateMapper->MapDepthFrameToCameraSpace(camera_points_num,
    reinterpret_cast<UINT16*>(depth_img.data), camera_points_num, depth_to_camera);
  if (FAILED(hResult))
  {
    std::cerr << "Error : ICoordinateMapper::MapDepthFrameToCameraSpace()" << std::endl;
    exit(-1);
  }

  std::vector<POINT3> points_on_board;
  points_on_board.clear();
  for (int index = 0; index < depth_frame.width*depth_frame.height; index++)
  {
    int x = index%depth_frame.width;
    int y = index / depth_frame.width;
    if (check_map[index] == 2 && depth_img.at<UINT16>(y,x) != INVALID_PIXEL_VALUE)
    {
      viewer.at<cv::Vec4b>(y, x) = cv::Vec4b(255, 255, 255, 255) - viewer.at<cv::Vec4b>(y, x);
      points_on_board.push_back(POINT3(
        depth_to_camera[index].X,
        depth_to_camera[index].Y,
        depth_to_camera[index].Z));
    }
  }

  // Estimate extrinsic parameters
  const int radius = 3;
  int now;
  POINT3 real_pattern[PATTERN_COLS*PATTERN_ROWS];
  for (int k = 0; k < PATTERN_COLS*PATTERN_ROWS; k++)
  { // Calcuration average around of detected corners for avoiding choosing invalid pixel.
    int points_num = 0;
    real_pattern[k] = POINT3::Zero();
    int corner_index = (int)corners[k].x + (int)corners[k].y * frame.width;
    FRAME_INDEX corner_on_depthframe(
      (int)color_to_depth[corner_index].X + 0.5f,
      (int)color_to_depth[corner_index].Y + 0.5f);
    for (int i = corner_on_depthframe[1] - radius; i <= corner_on_depthframe[1] + radius; i++)
    {
      for (int j = (int)(-sqrt(pow(radius, 2.0) - pow((i - corner_on_depthframe[1]), 2.0)) + corner_on_depthframe[0]);
      j <= sqrt(pow(radius, 2.0) - pow((i - corner_on_depthframe[1]), 2.0)) + corner_on_depthframe[0]; j++)
      {
        if (i >= 0 && i < depth_frame.height && j >= 0 && j < depth_frame.width &&
          depth_img.at<UINT16>(i,j) != INVALID_PIXEL_VALUE)
        {
          now = i*depth_frame.width+j;
          real_pattern[k] += POINT3(
            depth_to_camera[now].X,
            depth_to_camera[now].Y,
            depth_to_camera[now].Z);
          points_num++;
          viewer.at<cv::Vec4b>(i,j) = cv::Vec4b(0,128,255);
        }
      }
    }
    real_pattern[k] = real_pattern[k]/(float)points_num;
  }
  // calcuration of average
  POINT3 avg_vec(POINT3::Zero());
  for (int k = 0; k < points_on_board.size(); k++)
  {
    avg_vec += points_on_board[k];
  }
  avg_vec /= (float)points_on_board.size();
  // calcuration of covariance
  MATRIX3 cov_mat(MATRIX3::Zero());
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      for (int k = 0; k < points_on_board.size(); k++)
      {
        POINT3 tmp = points_on_board[k];
        cov_mat(i, j) += ((tmp[i] - avg_vec[i])
          * (tmp[j] - avg_vec[j])) / points_on_board.size();
      }
    }
  }
  // Calcurate Z-axis
  POINT3 camera_axis_x(POINT3::UnitX());
  POINT3 camera_axis_y(POINT3::UnitY());
  POINT3 camera_axis_z(POINT3::UnitZ());
  POINT3 world_axis_x(POINT3::UnitX());
  POINT3 world_axis_y(POINT3::UnitY());
  POINT3 world_axis_z(POINT3::UnitZ());
  Eigen::EigenSolver<MATRIX3> solver(cov_mat);
  float eigen_min = std::numeric_limits<float>::infinity();
  int z_index = 0;
  for (int i = 0; i < 3; i++)
  {
    if (solver.eigenvalues()(i).real() < eigen_min)
    {
      z_index = i;
      eigen_min = solver.eigenvalues()(i).real();
    }
  }
  Eigen::Matrix3cf  eigen_matrix;
  eigen_matrix = solver.eigenvectors();

  world_axis_z[0] = eigen_matrix(0, z_index).real();
  world_axis_z[1] = eigen_matrix(1, z_index).real();
  world_axis_z[2] = eigen_matrix(2, z_index).real();
  if (world_axis_z.dot(camera_axis_z) > 0)
  {
    world_axis_z = -world_axis_z;
  }
  world_axis_z = world_axis_z.normalized() * (float)pattern_width;

  // Estimate other 4 parameters
  POINT3 model_pattern[PATTERN_ROWS*PATTERN_COLS];
  { // Make model points
    for (int y = 0; y < PATTERN_ROWS; y++)
    {
      for (int x = 0; x < PATTERN_COLS; x++)
      {
        model_pattern[x + y*PATTERN_COLS] =
          POINT3(x*(float)pattern_width, y*(float)pattern_width, 0.0);
      }
    }
  }

  float roll = atan2f(world_axis_z[1], world_axis_z[2]);
  float pitch = atan2f(-world_axis_z[0],
    sqrtf(powf(world_axis_z[1], 2.0) + powf(world_axis_z[2], 2.0)));
  float yaw = 0.0;
  float d_x = 0.0;
  float d_y = 0.0;
  float d_z = 0.0;
  float Err = 0.0;

  MATRIX3 rotate_rp;
  rotate_rp = Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
    * Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());
  for (int i = 0; i < PATTERN_ROWS*PATTERN_COLS; i++)
  {
    d_z += model_pattern[i][2] - (rotate_rp * real_pattern[i])[2];
  }
  d_z /= (float)(PATTERN_ROWS * PATTERN_COLS);

  const float step = (float)0.00002;
  const float grad_th = (float)0.01;
  Eigen::Vector3f grad_E(Eigen::Vector3f::Ones());
  int cnt = 0;
  while (grad_E.norm() > grad_th)
  {
    MATRIX3 rot;
    rot = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ())
      * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
      * Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX());

    grad_E = Eigen::Vector3f::Zero();
    for (int i = 0; i < PATTERN_ROWS*PATTERN_COLS; i++)
    {
      Eigen::Vector3f rotated_point = rot * real_pattern[i];
      Eigen::Vector3f world_point = model_pattern[i];
      grad_E[0] += (float)2.0 * ((world_point[0] - d_x)*rotated_point[1] - (world_point[1] - d_y)*rotated_point[0]);
      grad_E[1] -= (float)2.0 * (world_point[0] - (rotated_point[0] + d_x));
      grad_E[2] -= (float)2.0 * (world_point[1] - (rotated_point[1] + d_y));
    }
    yaw -= step * grad_E[0];
    d_x -= step * grad_E[1];
    d_y -= step * grad_E[2];
    cnt++;

    if (cnt % 10000 == 0)
    {
      cv::Mat progress = viewer.clone();
      world_axis_x = rot.inverse() * Eigen::Vector3f::UnitX();
      world_axis_y = rot.inverse() * Eigen::Vector3f::UnitY();
      world_axis_z = rot.inverse() * Eigen::Vector3f::UnitZ();
      POINT3 origin(rot.inverse() * (POINT3::Zero() - POINT3(d_x, d_y, d_z)));
      draw_axis<float>(world_axis_x, origin, progress, cv::Scalar(0, 0, 255), pCoordinateMapper);
      draw_axis<float>(world_axis_y, origin, progress, cv::Scalar(0, 255, 0), pCoordinateMapper);
      draw_axis<float>(world_axis_z, origin, progress, cv::Scalar(255, 0, 0), pCoordinateMapper);
      for (int i = 0; i < PATTERN_ROWS*PATTERN_COLS; i++)
      {
        POINT3 pattern = rot.inverse() * (model_pattern[i] - POINT3(d_x, d_y, d_z));
        CameraSpacePoint pattern_camera;
        DepthSpacePoint pattern_depth;
        pattern_camera.X = pattern[0];
        pattern_camera.Y = pattern[1];
        pattern_camera.Z = pattern[2];
        pCoordinateMapper->MapCameraPointToDepthSpace(pattern_camera, &pattern_depth);
        cv::circle(progress, cv::Point(pattern_depth.X, pattern_depth.Y), 1, cv::Scalar(255, 128, 0));
      }
      cv::imshow("viewer", progress);
      cv::waitKey(10);
    }
  }

  // Output final result
  MATRIX3 rot;
  rot = Eigen::AngleAxisf(yaw, camera_axis_z)
    * Eigen::AngleAxisf(pitch, camera_axis_y)
    * Eigen::AngleAxisf(roll, camera_axis_x);
  world_axis_x = rot.inverse() * camera_axis_x;
  world_axis_y = rot.inverse() * camera_axis_y;
  world_axis_z = rot.inverse() * camera_axis_z;
  POINT3 origin(rot.inverse() * (POINT3::Zero() - POINT3(d_x, d_y, d_z)));
  draw_axis<float>(world_axis_x, origin, viewer, cv::Scalar(0, 0, 255), pCoordinateMapper);
  draw_axis<float>(world_axis_y, origin, viewer, cv::Scalar(0, 255, 0), pCoordinateMapper);
  draw_axis<float>(world_axis_z, origin, viewer, cv::Scalar(255, 0, 0), pCoordinateMapper);

  calcurate_error<float>(roll, pitch, yaw, d_x, d_y, d_z, real_pattern, model_pattern, Err);
  std::cout << "k: " << cnt << "---" << std::endl
    << "phi_x: " << roll << "\t"
    << "phi_y: " << pitch << "\t"
    << "phi_z: " << yaw << std::endl
    << "t_x: " << d_x << "\t"
    << "t_y: " << d_y << "\t"
    << "t_z: " << d_z << std::endl
    << "E: " << Err << "\t"
    << "grad_E: " << grad_E[0] << "\t" << grad_E[1] << "\t" << grad_E[2] << std::endl
    << "|grad_E|: " << grad_E.norm() << std::endl
    << std::endl;

  cv::imshow("viewer", viewer);

  // Choose attribution after calibration
  int key = cv::waitKey(0);
  if ('w' == key & 0x00ff)
  {
    translation = POINT3(d_x, d_y, d_z);
    rotation = QUATERNION(
      Eigen::AngleAxisf(yaw, camera_axis_z)*
      Eigen::AngleAxisf(pitch, camera_axis_y)*
      Eigen::AngleAxisf(roll, camera_axis_x));
    exportXML();
    exit(0);
  }
  if ('q' == key & 0x00ff)
  {
    exit(0);
  }

  delete[] check_map;
  delete[] color_to_depth;
  delete[] depth_to_camera;
  return;
}

void Calibration::exportXML()
{
  std::cout << "==== Export XML File ====" << std::endl;

  tinyxml2::XMLDocument xml;
  tinyxml2::XMLDeclaration *xml_decl = xml.NewDeclaration();
  xml.InsertEndChild(xml_decl);

  tinyxml2::XMLElement* extrinsic_parameter = xml.NewElement("extrinsic_parameter");
  tinyxml2::XMLElement* translation = extrinsic_parameter->GetDocument()->NewElement("translation");
  tinyxml2::XMLElement* rotation = extrinsic_parameter->GetDocument()->NewElement("rotation");
  tinyxml2::XMLElement* tx = translation->GetDocument()->NewElement("param");
  tinyxml2::XMLElement* ty = translation->GetDocument()->NewElement("param");
  tinyxml2::XMLElement* tz = translation->GetDocument()->NewElement("param");
  tinyxml2::XMLElement* qx = rotation->GetDocument()->NewElement("param");
  tinyxml2::XMLElement* qy = rotation->GetDocument()->NewElement("param");
  tinyxml2::XMLElement* qz = rotation->GetDocument()->NewElement("param");
  tinyxml2::XMLElement* qw = rotation->GetDocument()->NewElement("param");

  xml.InsertEndChild(extrinsic_parameter);

  std::stringstream ss;
  extrinsic_parameter->InsertEndChild(translation);
  translation->InsertEndChild(tx);
  translation->InsertEndChild(ty);
  translation->InsertEndChild(tz);

  ss << this->translation[0];
  tx->InsertEndChild(tx->GetDocument()->NewText(ss.str().c_str()));
  ss.str("");
  ss.clear(std::stringstream::goodbit);

  ss << this->translation[1];
  ty->InsertEndChild(ty->GetDocument()->NewText(ss.str().c_str()));
  ss.str("");
  ss.clear(std::stringstream::goodbit);

  ss << this->translation[2];
  tz->InsertEndChild(tz->GetDocument()->NewText(ss.str().c_str()));
  ss.str("");
  ss.clear(std::stringstream::goodbit);

  tx->SetAttribute("name", "tx");
  ty->SetAttribute("name", "ty");
  tz->SetAttribute("name", "tz");

  extrinsic_parameter->InsertEndChild(rotation);
  rotation->InsertEndChild(qx);
  rotation->InsertEndChild(qy);
  rotation->InsertEndChild(qz);
  rotation->InsertEndChild(qw);

  ss << this->rotation.x();
  qx->InsertEndChild(qx->GetDocument()->NewText(ss.str().c_str()));
  ss.str("");
  ss.clear(std::stringstream::goodbit);

  ss << this->rotation.y();
  qy->InsertEndChild(qy->GetDocument()->NewText(ss.str().c_str()));
  ss.str("");
  ss.clear(std::stringstream::goodbit);

  ss << this->rotation.z();
  qz->InsertEndChild(qz->GetDocument()->NewText(ss.str().c_str()));
  ss.str("");
  ss.clear(std::stringstream::goodbit);

  ss << this->rotation.w();
  qw->InsertEndChild(qw->GetDocument()->NewText(ss.str().c_str()));
  ss.str("");
  ss.clear(std::stringstream::goodbit);

  qx->SetAttribute("name", "qx");
  qy->SetAttribute("name", "qy");
  qz->SetAttribute("name", "qz");
  qw->SetAttribute("name", "qw");

  xml.Print();
  tinyxml2::XMLError xml_state = xml.SaveFile("../parameter.xml");
  if (xml_state != tinyxml2::XML_NO_ERROR)
  {
    std::cout << "Failed to write XML file." << std::endl;
  }
  std::cout << std::endl;
  cv::waitKey(0);

  return;
}

Calibration::Calibration(SENSOR camera, float pattern_width) :
  sensor(camera),
  pattern_width(pattern_width)
{
  frame.type = COLOR;
#ifdef USING_KINECT_V2
  // Setting for reading depth frame
	HRESULT hResult = sensor->get_DepthFrameSource( &pDepthSource );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IKinectSensor::get_DepthFrameSource()" << std::endl;
    exit(-1);
	}
	hResult = pDepthSource->OpenReader( &pDepthReader );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IDepthFrameSource::OpenReader()" << std::endl;
    exit(-1);
	}
	hResult = pDepthSource->get_FrameDescription( &pDepthDescription );
	if( FAILED( hResult ) ){
		std::cerr << "Error : IDepthFrameSource::get_FrameDescription()" << std::endl;
    exit(-1);
	}
  depth_frame.type = DEPTH;
	pDepthDescription->get_Width( &depth_frame.width );
	pDepthDescription->get_Height( &depth_frame.height );

  switch (frame.type)
  {
  case INFRARED:
    // Setting for reading color frame
    hResult = sensor->get_InfraredFrameSource(&pInfraredSource);
    if (FAILED(hResult)) {
      std::cerr << "Error : IKinectSensor::get_InfraredFrameSource()" << std::endl;
      exit(-1);
    }
    hResult = pInfraredSource->OpenReader(&pInfraredReader);
    if (FAILED(hResult)) {
      std::cerr << "Error : IInfraredFrameSource::OpenReader()" << std::endl;
      exit(-1);
    }
    hResult = pInfraredSource->get_FrameDescription(&pFrameDescription);
    if (FAILED(hResult)) {
      std::cerr << "Error : IInfraredFrameSource::get_FrameDescription()" << std::endl;
      exit(-1);
    }
    break;
  case COLOR:
    // Setting for reading color frame
    hResult = sensor->get_ColorFrameSource(&pColorSource);
    if (FAILED(hResult)) {
      std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
      exit(-1);
    }
    hResult = pColorSource->OpenReader(&pColorReader);
    if (FAILED(hResult)) {
      std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
      exit(-1);
    }
    hResult = pColorSource->get_FrameDescription(&pFrameDescription);
    if (FAILED(hResult)) {
      std::cerr << "Error : IColorFrameSource::get_FrameDescription()" << std::endl;
      exit(-1);
    }
    break;
  }
  pFrameDescription->get_Width(&frame.width);
  pFrameDescription->get_Height(&frame.height);
#endif
  return;
}

Calibration::~Calibration()
{
  SafeRelease(&pDepthSource);
  SafeRelease(&pDepthReader);
  switch (frame.type)
  {
  case INFRARED:
    SafeRelease(&pInfraredSource);
    SafeRelease(&pInfraredReader);
    break;
  case COLOR:
    SafeRelease(&pColorSource);
    SafeRelease(&pColorReader);
    break;
  }
	SafeRelease( &pFrameDescription );

	cv::destroyAllWindows();
  return;
}

void Calibration::run()
{
  cv::setUseOptimized(true);

  int cv_type;
  int cv_channel;
  double display_scale;
  switch (frame.type)
  {
  case INFRARED:
    cv_type = CV_16UC1;
    cv_channel = 1;
    display_scale = 1.0;
    break;
  case COLOR:
    cv_type = CV_8UC4;
    cv_channel = 4;
    display_scale = 0.5;
    break;
  }

	unsigned int depthSize = depth_frame.width * depth_frame.height* sizeof( unsigned short );
	unsigned int bufferSize = frame.width * frame.height
    * cv_channel * sizeof( unsigned char );

	cv::Mat depthMat( depth_frame.height, depth_frame.width, CV_16UC1 );
	cv::Mat depthFrameMat(depth_frame.height, depth_frame.width, CV_8UC1);
	cv::Mat bufferMat( frame.height, frame.width, cv_type );
	cv::Mat frameMat( frame.height / 2, frame.width / 2, cv_type );
	//cv::namedWindow( "Depth" );
	cv::namedWindow( "Frame" );

	while( 1 )
  {
    // Depth frame
    IDepthFrame* pDepthFrame = nullptr;
    HRESULT hResult = pDepthReader->AcquireLatestFrame(&pDepthFrame);
		if( SUCCEEDED( hResult ) ){
			hResult = pDepthFrame->AccessUnderlyingBuffer( &depthSize,
        reinterpret_cast<UINT16**>( &depthMat.data ) );
			//if( SUCCEEDED( hResult ) ){
			//	depthMat.convertTo( depthFrameMat, CV_8U, -255.0f / 8000.0f, 255.0f );
			//}
		}
    
    // Frame
    if (frame.type == INFRARED)
    {
      IInfraredFrame* pInfraredFrame = nullptr;
      hResult = pInfraredReader->AcquireLatestFrame(&pInfraredFrame);
      if (SUCCEEDED(hResult))
      {
        hResult = pInfraredFrame->CopyFrameDataToArray(bufferSize,
          reinterpret_cast<UINT16*>(bufferMat.data));
        if (SUCCEEDED(hResult))
        {
          cv::resize(bufferMat, frameMat, cv::Size(), display_scale, display_scale);
        }
      }
      SafeRelease(&pInfraredFrame);
    }
    else if (frame.type == COLOR)
    {
      IColorFrame* pColorFrame = nullptr;
      hResult = pColorReader->AcquireLatestFrame(&pColorFrame);
      if (SUCCEEDED(hResult))
      {
        hResult = pColorFrame->CopyConvertedFrameDataToArray(bufferSize,
          reinterpret_cast<BYTE*>(bufferMat.data),
          ColorImageFormat::ColorImageFormat_Bgra);
        if (SUCCEEDED(hResult))
        {
          cv::imshow("Frame", frameMat);
          core_calcuration(bufferMat, depthMat);
          cv::resize(bufferMat, frameMat, cv::Size(), display_scale, display_scale);
        }
      }
      SafeRelease(&pDepthFrame);
      SafeRelease( &pColorFrame );
    }

		//cv::imshow( "Depth", depthFrameMat );

		if( cv::waitKey( 100 ) == VK_ESCAPE ){
			break;
		}
	}

  return;
}