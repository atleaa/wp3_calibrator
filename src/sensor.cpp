/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Atle Aalerud
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Atle Aalerud [atle.aalerud@uia.no]
 *
 */


#include "wp3_calibrator/sensor.h"


namespace wp3{
//namespace calibrate{

// Constructor
Sensor::Sensor(std::string name, ros::NodeHandle & nodehandle) :
  nh_(nodehandle),
  it_(nodehandle),
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
  transCamToAruco_ = Eigen::Matrix4f::Identity();
  transArucoToICP_ = Eigen::Matrix4f::Identity();
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

void Sensor::clearImageMatVec()
{
  imageMatVec_.clear();
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
  wp3::imageConverter ic_depth(depthTopic_, "depth", it_, nh_);
  wp3::imageConverter ic_color(imageTopic_, "color", it_, nh_);

  depthProcessor dp = depthProcessor(cloudTopic_, nh_); // to fix: src_cloud in this class is single-linked in callback, quick fix-> create two src_clouds

//  ros::NodeHandle n("~");
  ros::Subscriber camera_info_sub = nh_.subscribe(camera_info_topic_, 1, &Sensor::cameraInfoCallback, this);


  if (update)
  {
    imageMat_.release();
    depthMat_.release();
    imageMatVec_.clear();
    depthMatVec_.clear();

    cloudPtr_->clear();
    cloudCrPtr_->clear();
    cloud1Ptr_->clear();
    cloud1CrPtr_->clear();
    cloud2Ptr_->clear();
    cloud2CrPtr_->clear();
    cloud3Ptr_->clear();
    cloud3CrPtr_->clear();

    cloudPtr_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloudCrPtr_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloud1Ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloud1CrPtr_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloud2Ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloud2CrPtr_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloud3Ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloud3CrPtr_.reset(new pcl::PointCloud<pcl::PointXYZ>);

    transCamToAruco_ = Eigen::Matrix4f::Identity();
    transArucoToICP_ = Eigen::Matrix4f::Identity();


//    ROS_DEBUG_STREAM("emptied cloud, size now for " << cloudTopic_<< " is: " << cloudPtr_->size() << std::endl;);
  }

  // get color image
  ROS_DEBUG_STREAM("Reading " << imageTopic_ << " ... "<< std::flush);
  for(int i=0;i<IMAGES_C;i++)
  {
    imageMat_.release();
    while(imageMat_.empty())
    {
      ic_color.getCurrentImage(&imageMat_);
    }
    imageMatVec_.push_back(imageMat_);
    std::cout << "." << std::flush;
  }
  ROS_DEBUG_STREAM("Done reading " << imageTopic_ << std::endl);

  // get depth image
  ROS_DEBUG_STREAM("Reading " << depthTopic_ << " ... "<< std::flush);
  for(int i=0;i<IMAGES_D;i++)
  {
    depthMat_.release();
    while(depthMat_.empty())
    {
      ic_depth.getCurrentDepthMap(&depthMat_);
    }
    depthMatVec_.push_back(depthMat_);
    std::cout << "." << std::flush;
  }

  ROS_DEBUG_STREAM("Done reading " << depthTopic_ << std::endl);

  // get cloud
  ROS_DEBUG_STREAM("Reading " << cloudTopic_ << " ... "<< std::flush);
  while( cloudPtr_->size() == 0)
  {
    dp.get_filtered_PCL_cloud(cloudPtr_);
    std::cout << "." << std::flush;
  }
  ROS_DEBUG_STREAM("Done reading " << cloudTopic_ << std::endl);
} // end readTopics

//void Sensor::appendClouds()
//{
//  *cloudAccPtr_ += *cloudPtr_;
//  *cloud1AccPtr_ += *cloud1Ptr_;
//}

//} // end namespace calibrate
} // end namespace wp3
