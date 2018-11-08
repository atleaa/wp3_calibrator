#include "wp3_calibrator/sensor.h"


namespace wp3{

// Constructor
Sensor::Sensor(std::string name) :
  cloudPtr_(new pcl::PointCloud<pcl::PointXYZ>),
  cloudCrPtr_(new pcl::PointCloud<pcl::PointXYZ>),
  cloud1Ptr_(new pcl::PointCloud<pcl::PointXYZ>),
  cloud1CrPtr_(new pcl::PointCloud<pcl::PointXYZ>),
  cloud2Ptr_(new pcl::PointCloud<pcl::PointXYZ>),
  cloud2CrPtr_(new pcl::PointCloud<pcl::PointXYZ>),
  cloud3Ptr_(new pcl::PointCloud<pcl::PointXYZ>),
  cloud3CrPtr_(new pcl::PointCloud<pcl::PointXYZ>),
  intrinsics_set_(false)
{
  name_ = name;
}


// Destructor
Sensor::~Sensor()
{
// do nothing..
}

void Sensor::clear()
{
  intrinsics_set_ = false;
//  cloudPtr_->clear();
//  cloud1Ptr_->clear();
//  cloud2Ptr_->clear();
//  cloud3Ptr_->clear();

//  cloudAccPtr_->clear();
//  cloud1AccPtr_->clear();
//  cloud2AccPtr_->clear();
//  cloud3AccPtr_->clear();
}

std::string Sensor::getDepthTopic() const {return depthTopic_;}
void Sensor::setDepthTopic(const std::string &value){depthTopic_ = value;}

std::string Sensor::getImageTopic() const {return imageTopic_;}
void Sensor::setImageTopic(const std::string &value){imageTopic_ = value;}

std::string Sensor::getCloudTopic() const{return cloudTopic_;}
void Sensor::setCloudTopic(const std::string &cloudTopic){cloudTopic_ = cloudTopic;}

cv::Mat Sensor::getImageMat() {return imageMat_;}
void Sensor::setImageMat(const cv::Mat &imageMat){imageMat_ = imageMat;}

cv::Mat Sensor::getDepthMat() const{return depthMat_;}
void Sensor::setDepthMat(const cv::Mat &depthMat){depthMat_ = depthMat;}

pcl::PointCloud<pcl::PointXYZ>::Ptr Sensor::getCloud() const{return cloudPtr_;}
void Sensor::setCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){cloudPtr_ = cloud;}

std::vector<cv::Mat> Sensor::getDepthMatVec() const
{
  return depthMatVec_;
}

std::string Sensor::getTfTopic() const
{
  return tfTopic_;
}

void Sensor::setTfTopic(const std::string &tfTopic)
{
  tfTopic_ = tfTopic;
}

std::vector<cv::Mat> Sensor::getImageMatVec() const
{
  return imageMatVec_;
}

void Sensor::setCamera_info_topic(const std::string &camera_info_topic)
{
  camera_info_topic_ = camera_info_topic;
}

Eigen::Matrix3d Sensor::getIntrinsics_matrix() const
{
  return intrinsics_matrix_;
}

std::vector<double> Sensor::getDistCoeffs() const
{
  return distCoeffs_;
}


void Sensor::cameraInfoCallback (const sensor_msgs::CameraInfo::ConstPtr & msg)
{
  if (!intrinsics_set_)
  {
    // http://docs.ros.org/jade/api/sensor_msgs/html/msg/CameraInfo.html
    // The distortion parameters, size depending on the distortion model.
    // For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
    //float64[] D
    distCoeffs_ = msg->D;

    // Intrinsic camera matrix for the raw (distorted) images.
    //     [fx  0 cx]
    // K = [ 0 fy cy]
    //     [ 0  0  1]
    // Projects 3D points in the camera coordinate frame to 2D pixel
    // coordinates using the focal lengths (fx, fy) and principal point
    // (cx, cy).
    // float64[9]  K # 3x3 row-major matrix
    intrinsics_matrix_ << msg->K.elems[0], msg->K.elems[1], msg->K.elems[2],
        msg->K.elems[3], msg->K.elems[4], msg->K.elems[5],
        msg->K.elems[6], msg->K.elems[7], msg->K.elems[8];
    intrinsics_set_ = true;

    std::stringstream str;
    for (std::vector<double>::const_iterator i = distCoeffs_.begin(); i != distCoeffs_.end(); ++i)
      str << *i << ' ';

    ROS_DEBUG_STREAM("Received camera information for " << name_ << std::endl
                     << "D:" << std::endl << str.str() << std::endl
                     << "K:" << std::endl << intrinsics_matrix_ << std::endl);
  }
}

void Sensor::readTopics(bool update = false)
{
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Parameters ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  wp3::imageConverter ic_depth(depthTopic_, "depth");
  wp3::imageConverter ic_color(imageTopic_, "color");

  depthProcessor dp = depthProcessor(cloudTopic_); // to fix: src_cloud in this class is single-linked in callback, quick fix-> create two src_clouds

  ros::NodeHandle n("~");
  ros::Subscriber camera_info_sub = n.subscribe(camera_info_topic_, 1, &Sensor::cameraInfoCallback, this);


  if (update)
  {
    imageMat_.release();
    depthMat_.release();
    cloudPtr_->clear();
    imageMatVec_.clear();
//    ROS_DEBUG_STREAM("emptied cloud, size now for " << cloudTopic_<< " is: " << cloudPtr_->size() << std::endl;);
  }

  // get color image
  ROS_DEBUG_STREAM("Reading " << imageTopic_ << " ... "<< std::flush);
  for(int i=0;i<ACCUMULATE;i++)
  {
    imageMat_.release();
    while(imageMat_.empty())
    {
      ic_color.getCurrentImage(&imageMat_);
    }
    imageMatVec_.push_back(imageMat_);
  }
  ROS_DEBUG_STREAM("Done reading " << imageTopic_ << std::endl);

  // get depth image
  ROS_DEBUG_STREAM("Reading " << depthTopic_ << " ... "<< std::flush);
  for(int i=0;i<ACCUMULATE;i++)
  {
    depthMat_.release();
    while(depthMat_.empty())
    {
      ic_depth.getCurrentDepthMap(&depthMat_);
    }
    depthMatVec_.push_back(depthMat_);
  }

  ROS_DEBUG_STREAM("Done reading " << depthTopic_ << std::endl);

  // get cloud
  ROS_DEBUG_STREAM("Reading " << cloudTopic_ << " ... "<< std::flush);
  while( cloudPtr_->size() == 0)
  {
    dp.get_filtered_PCL_cloud(cloudPtr_);
  }
  ROS_DEBUG_STREAM("Done reading " << cloudTopic_ << std::endl);
} // end readTopics

void Sensor::appendClouds()
{
  *cloudAccPtr_ += *cloudPtr_;
  *cloud1AccPtr_ += *cloud1Ptr_;
}


} // end namespace wp3
