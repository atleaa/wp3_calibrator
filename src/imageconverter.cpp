#include "wp3_calibrator/imageconverter.h"



namespace wp3 {

// imageconverter -----------------------------------------------
// Constructor
imageConverter::imageConverter(const std::string& inputName,
                               const std::string&  type,
                               image_transport::ImageTransport & image_transport_nh,
                               ros::NodeHandle & nodehandle)
//  nh_(nodehandle),
//  it_(nodehandle)
{
  //r_mutex.lock();
  topic_type_ = inputName;
  if (type == "depth")
  {
    image_sub_ = image_transport_nh.subscribe(inputName, 1, &imageConverter::callback_depth, this, image_transport::TransportHints(IMAGE_TYPE));
//    image_sub_ = image_transport_nh.subscribe(inputName, 1, &imageConverter::callback_depth, this);
    ROS_DEBUG_STREAM("Subscribing to " << inputName);
  }

  else if (type == "color")
  {
    image_sub_ = image_transport_nh.subscribe(inputName, 1, &imageConverter::callback_color, this, image_transport::TransportHints(IMAGE_TYPE));
//    image_sub_ = image_transport_nh.subscribe(inputName, 1, &imageConverter::callback_color, this);
    ROS_DEBUG_STREAM("Subscribing to " << inputName);
  }
  //r_mutex.unlock();
}


// Deconstructor
imageConverter::~imageConverter()
{
  image_sub_.shutdown();
  ROS_DEBUG_STREAM("ROS stopped Topic Subscription of " << topic_type_);
}


void imageConverter::getCurrentImage(cv::Mat *input_image)
{
  while((timestamp_.toSec() - last_frame.toSec()) <= 0) {
    usleep(50); // changed from 2000
    ros::spinOnce();
  }
  i_mutex.lock();
  *input_image = src_image_;
  last_frame = timestamp_;
  i_mutex.unlock();
}


void imageConverter::getCurrentDepthMap(cv::Mat *input_image)
{
  while((timestamp_.toSec() - last_frame.toSec()) <= 0) {
    usleep(50); // changed from 2000
    ros::spinOnce();
  }
  d_mutex.lock();
  *input_image = src_depth_;
  last_frame = timestamp_;
  d_mutex.unlock();
}


void imageConverter::callback_depth(const sensor_msgs::ImageConstPtr& msg)
{
  timestamp_ = msg->header.stamp;
  cv_bridge::CvImageConstPtr pCvImage;

  try
  {
    pCvImage = cv_bridge::toCvShare(msg, msg->encoding);// same encoding of the message as the source,  sensor_msgs::image_encodings::16UC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  pCvImage->image.copyTo(src_depth_);
  src_depth_.convertTo(src_depth_,  CV_32F, 0.001);
}


void imageConverter::callback_color(const sensor_msgs::ImageConstPtr& msg)
{
  timestamp_ = msg->header.stamp;
  cv_bridge::CvImagePtr cv_ptr;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  i_mutex.lock();
  src_image_ = cv_ptr->image;
  i_mutex.unlock();

}



// depthprocessor ----------------------------------------------
// Constructor
depthProcessor::depthProcessor(const std::string& inputNameA,
                                ros::NodeHandle & nodehandle)
  : nh_(nodehandle)
{
  depth_subA_ = nh_.subscribe(inputNameA, 1, &depthProcessor::callback_cloudA, this);
  // this refers to the non static object  (here: a depthProcessor object used in main)
}


// Destructor
depthProcessor::~depthProcessor()
{
  depth_subA_.shutdown();
  ROS_DEBUG_STREAM("ROS stopped Depth Import");
}


void depthProcessor::get_filtered_PCL_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
{
  while((timestamp_depth_.toSec() - last_frame_depth_.toSec()) <= 0) {
    usleep(2000);
    ros::spinOnce();
  }

  pcl::copyPointCloud(tmp_cloud_read_, *cloud_ptr);
  //	    *cloud_ptr = tmp_cloud_read; // also a possibility
  last_frame_depth_ = timestamp_depth_;
}


void depthProcessor::callback_cloudA(const sensor_msgs::PointCloud2ConstPtr& input)
{

  ros::Time frame_time_depth = ros::Time::now();
  timestamp_depth_ = frame_time_depth;
  pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2);

  pcl_conversions::toPCL(*input, *cloud_filtered);
  pcl::fromPCLPointCloud2(*cloud_filtered, tmp_cloud_read_);
}


} // end namespace wp3
