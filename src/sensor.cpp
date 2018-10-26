#include "wp3_calibrator/sensor.h"
#include "wp3_calibrator/imageconverter.h"
#include "wp3_calibrator/defines.h"

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
  cloud3CrPtr_(new pcl::PointCloud<pcl::PointXYZ>)
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
  cloudPtr_->clear();
  cloud1Ptr_->clear();
  cloud2Ptr_->clear();
  cloud3Ptr_->clear();

  cloudAccPtr_->clear();
  cloud1AccPtr_->clear();
  cloud2AccPtr_->clear();
  cloud3AccPtr_->clear();
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


void Sensor::readTopics(bool update = false)
{
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Parameters ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  wp3::imageConverter IC_Depth_A(depthTopic_, "depth");
  //        std::cout << depthMatA << " converted" << std::endl; //TULL
  wp3::imageConverter IC_RGB_A(imageTopic_, "color");
  //        std::cout << rgbA << " converted" << std::endl; //TULL

  depthProcessor dp_A = depthProcessor(cloudTopic_); // to fix: src_cloud in this class is single-linked in callback, quick fix-> create two src_clouds

  if (update)
  {
    imageMat_.release();
    depthMat_.release();
    cloudPtr_->clear();
//    cloudPtr_.reset();

//    std::cout << "emptied cloud, size now for " << cloudTopic_<< " is: " << cloudPtr_->size() << std::endl;
    ROS_DEBUG_STREAM("emptied cloud, size now for " << cloudTopic_<< " is: " << cloudPtr_->size() << std::endl;);
  }

  // get color image
//  std::cout << "Reading " << imageTopic_ << " ... "<< std::flush;
  while(imageMat_.empty())
  {
    IC_RGB_A.getCurrentImage(&imageMat_);
  }
  ROS_DEBUG_STREAM("done" << std::endl);

  // get depth image
  ROS_DEBUG_STREAM("Reading " << depthTopic_ << " ... "<< std::flush);
  while(depthMat_.empty())
  {
    IC_Depth_A.getCurrentDepthMap(&depthMat_);
  }
  ROS_DEBUG_STREAM("done " << std::endl);

  // get cloud
  ROS_DEBUG_STREAM("Reading " << cloudTopic_ << " ... "<< std::flush);
  while( cloudPtr_->size() == 0)
  {
    dp_A.get_filtered_PCL_cloud(cloudPtr_);
  }
  ROS_DEBUG_STREAM("done" << std::endl);
} // end readTopics

void Sensor::appendClouds()
{
  *cloudAccPtr_ += *cloudPtr_;
  *cloud1AccPtr_ += *cloud1Ptr_;
//  *cloud2AccPtr_ += *cloud2Ptr_;
//  *cloud3AccPtr_ += *cloud3Ptr_;
}


} // end namespace wp3
