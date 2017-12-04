#include <dso_ros/ros_output_wrapper.h>

using namespace dso_ros;

ROSOutputWrapper::ROSOutputWrapper(ros::NodeHandle& n,
                                   ros::NodeHandle& n_private)
{
  ROS_INFO("ROSOutputWrapper created\n");
  if (!n_private.hasParam("dso_frame_id")) {
    ROS_WARN("No param named world_frame found!");
  }
  if (!n_private.hasParam("camera_frame_id")) {
    ROS_WARN("No param named camera_frame found!");
  }
  n_private.param<std::string>("dso_frame_id", dso_frame_id_, "dso_odom");
  n_private.param<std::string>("camera_frame_id", camera_frame_id_, "camera");
  n_private.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
  n_private.param<std::string>("base_frame_id", base_frame_id_, "base_link");
  ROS_INFO_STREAM("world_frame_id = " << dso_frame_id_ << "\n");
  ROS_INFO_STREAM("camera_frame_id = " << camera_frame_id_ << "\n");
  ROS_INFO_STREAM("base_frame_id = " << base_frame_id_ << "\n");
  ROS_INFO_STREAM("odom_frame_id = " << odom_frame_id_ << "\n");
  dso_odom_pub_ = n.advertise<nav_msgs::Odometry>("odom", 5, false);
  dso_depht_image_pub_ =
      n.advertise<sensor_msgs::Image>("image_rect", 5, false);
  pcl_pub_ = n.advertise<sensor_msgs::PointCloud2>("pcl", 5, false);
  last_pose_.setIdentity();
  pose_.setIdentity();
  reset_ = false;
  last_id_ = 10;
}

ROSOutputWrapper::~ROSOutputWrapper()
{
  ROS_INFO("ROSOutputPublisher destroyed\n");
}

void ROSOutputWrapper::publishKeyframes(std::vector<dso::FrameHessian*>& frames,
                                        bool final, dso::CalibHessian* HCalib)
{
  dso::FrameHessian* last_frame = frames.back();
  if (last_frame->shell->id == last_id_)
    return;
  last_id_ = last_frame->shell->id;
  DSOCameraParams params(HCalib);
  // camToWorld = frame->camToWorld;
  PointCloud::Ptr cloud(new PointCloud());
  for (dso::PointHessian* p : last_frame->pointHessians) {
    auto points = DSOtoPcl(p, params);
    std::move(points.begin(), points.end(), std::back_inserter(cloud->points));
  }

  for (dso::PointHessian* p : last_frame->pointHessiansMarginalized) {
    auto points = DSOtoPcl(p, params);
    std::move(points.begin(), points.end(), std::back_inserter(cloud->points));
  }

  sensor_msgs::PointCloud2::Ptr msg(new sensor_msgs::PointCloud2());
  pcl::toROSMsg(*cloud, *msg);
  msg->header.stamp = timestamp_;
  msg->header.frame_id = camera_frame_id_;
  pcl_pub_.publish(msg);
}

void ROSOutputWrapper::publishCamPose(dso::FrameShell* frame,
                                      dso::CalibHessian* HCalib)
{
  ROS_DEBUG_STREAM("publishCamPose called");
  tf::StampedTransform tf_odom_base;
  tf::StampedTransform tf_base_cam;
  try {
    tf_list_.waitForTransform(odom_frame_id_, base_frame_id_, timestamp_,
                              ros::Duration(5.0));
    tf_list_.lookupTransform(odom_frame_id_, base_frame_id_, timestamp_,
                             tf_odom_base);

  } catch (...) {
    ROS_ERROR_STREAM("DSO_ROS: Not sucessfull in retrieving tf tranform from "
                     << odom_frame_id_ << "->" << base_frame_id_);
    return;
  }
  try {
    tf_list_.waitForTransform(base_frame_id_, camera_frame_id_, timestamp_,
                              ros::Duration(5.0));
    tf_list_.lookupTransform(base_frame_id_, camera_frame_id_, timestamp_,
                             tf_base_cam);
  } catch (...) {
    ROS_ERROR_STREAM("DSO_ROS: Not sucessfull in retrieving tf tranform from "
                     << base_frame_id_ << "->" << camera_frame_id_);
    return;
  }

  /*
   * This function broadcasts tf transformation:
   * world->cam based on frame->camToWorld.matrix3x4()
   *
   * frame->camToWorld.matrix3x4() returns:
   *
   * m00 m01 m02 m03
   * m10 m11 m12 m13
   * m20 m21 m22 m23
   *
   * last column is translation vector
   * 3x3 matrix with diagonal m00 m11 and m22 is a rotation matrix
  */

  const Eigen::Matrix<Sophus::SE3Group<double>::Scalar, 3, 4> m =
      frame->camToWorld.matrix3x4();
  /* camera position */
  double camX = m(0, 3);
  double camY = m(1, 3);
  double camZ = m(2, 3);

  /* camera orientation */
  /*
  http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
   */
  double numX = 1 + m(0, 0) - m(1, 1) - m(2, 2);
  double numY = 1 - m(0, 0) + m(1, 1) - m(2, 2);
  double numZ = 1 - m(0, 0) - m(1, 1) + m(2, 2);
  double numW = 1 + m(0, 0) + m(1, 1) + m(2, 2);
  double camSX = sqrt(std::max(0.0, numX)) / 2;
  double camSY = sqrt(std::max(0.0, numY)) / 2;
  double camSZ = sqrt(std::max(0.0, numZ)) / 2;
  double camSW = sqrt(std::max(0.0, numW)) / 2;

  /* broadcast map -> cam_pose transformation */
  static tf::TransformBroadcaster br;
  tf::Transform current_pose;
  current_pose.setOrigin(tf::Vector3(camX, camY, camZ));
  tf::Quaternion q = tf::Quaternion(camSX, camSY, camSZ, camSW);
  current_pose.setRotation(q);

  tf::Transform movement;
  if (reset_) {
    // this part is needed in case that reset was called
    reset_ = false;
    last_pose_.setIdentity();
    movement.setIdentity();
  } else {
    last_pose_ = pose_;
    movement = last_pose_.inverse() * current_pose;
  }
  // pose stay same even when system crashes. A relative transformation is hold
  // in movement.
  pose_ = pose_ * movement;
  ROS_ERROR_STREAM("[DSO_NODE]: Current position: "
                   << pose_.getOrigin().getX() << ", "
                   << pose_.getOrigin().getY() << ", "
                   << pose_.getOrigin().getZ());

  tf::Transform tf_dso_base = pose_ * tf_base_cam.inverse();
  tf::Transform tf_dso_odom = tf_dso_base * tf_odom_base.inverse();
  br.sendTransform(tf::StampedTransform(tf_dso_odom, timestamp_, dso_frame_id_,
                                        odom_frame_id_));

  ROS_INFO_STREAM("ROSOutputWrapper:" << base_frame_id_ << "->" << dso_frame_id_
                                      << " tf broadcasted");
  nav_msgs::Odometry odom;
  odom.header.stamp = timestamp_;
  odom.header.frame_id = dso_frame_id_;
  tf::poseTFToMsg(tf_dso_base, odom.pose.pose);
  dso_odom_pub_.publish(odom);
  //  /* testing output */
  /*
        std::cout << frame->camToWorld.matrix3x4() << "\n";
  */
}

void ROSOutputWrapper::pushLiveFrame(dso::FrameHessian* image)
{
  // can be used to get the raw image / intensity pyramid.
}

void ROSOutputWrapper::pushDepthImage(dso::MinimalImageB3* image)
{
  // can be used to get the raw image with depth overlay.
}
bool ROSOutputWrapper::needPushDepthImage()
{
  return false;
}

void ROSOutputWrapper::pushDepthImageFloat(dso::MinimalImageF* image,
                                           dso::FrameHessian* KF)
{
  cv::Mat image_cv(image->h, image->w, CV_32FC1, image->data);
  image_cv.convertTo(image_cv, CV_8UC1, 255.0f);
  cv::Mat imverted_img;
  cv::bitwise_not(image_cv, imverted_img);
  std_msgs::Header header;
  header.frame_id = camera_frame_id_;
  header.stamp = timestamp_;
  header.seq = seq_image_;
  ++seq_image_;
  cv_bridge::CvImage bridge_img(header, "mono8", imverted_img);
  dso_depht_image_pub_.publish(bridge_img.toImageMsg());
}

std::vector<dso_ros::ROSOutputWrapper::Point>
dso_ros::ROSOutputWrapper::DSOtoPcl(
    const dso::PointHessian* pt,
    const dso_ros::ROSOutputWrapper::DSOCameraParams& params) const
{
  std::vector<dso_ros::ROSOutputWrapper::Point> res;
  float my_scaledTH = 1e10;
  float my_absTH = 1e10;
  float my_minRelBS = 0;
  float depth = 1.0f / pt->idepth_scaled;
  float depth4 = std::pow(depth, 4);
  float var = (1.0f / (pt->idepth_hessian + 0.01));

  if (pt->idepth_scaled < 0)
    return res;
  if (var * depth4 > my_scaledTH)
    return res;

  if (var > my_absTH)
    return res;

  if (pt->maxRelBaseline < my_minRelBS)
    return res;

  for (size_t i = 0; i < patternNum; ++i) {
    int dx =
        dso::staticPattern[8][i][0];  // reading from big matrix in settings.cpp
    int dy =
        dso::staticPattern[8][i][1];  // reading from big matrix in settings.cpp
    Point pcl_pt;
    pcl_pt.x = ((pt->u + dx) * params.fxi + params.cxi) * depth;
    pcl_pt.y = ((pt->v + dy) * params.fyi + params.cyi) * depth;
    pcl_pt.z = depth * (1 + 2 * params.fxi * (rand() / (float)RAND_MAX - 0.5f));
    res.emplace_back(pcl_pt);
  }
  return res;
}
