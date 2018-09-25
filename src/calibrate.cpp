
#include "wp3_calibrator/defines.h"
#include "wp3_calibrator/classtest.h"
#include "wp3_calibrator/functions.h"
#include "wp3_calibrator/visualization.h"


// STD
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <fstream>
#include <string>
#include <unistd.h>
#include <mutex>
#include <thread>
#include <vector>

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
//#include <tf/transform_Broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/opencv.hpp"

#include <boost/thread/thread.hpp>

// PCL
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/filter.h>
//#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/ros/conversions.h>
// /usr/include/pcl-1.7/pcl/ros/conversions.h:44:2: warning: #warning The <pcl/ros/conversions.h> header is deprecated.
//please use <pcl/conversions.h> instead. [-Wcpp] #warning The <pcl/ros/conversions.h> header is deprecated. please use \
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_conversions/pcl_conversions.h>

// Registration
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>

// Filtering
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/conditional_removal.h>

// Libfreenect2
#include <libfreenect2/registration.h>

typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pclVisColorCustom;
typedef pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> pclVisColorGeneric;
//pcl::visualization::PCLVisualizer reg_viewer("Input clouds and registration");
//pcl::visualization::PCLVisualizer viewer1 ("Visualizer");
//pcl::visualization::PCLVisualizer viewer2 ("Visualizer2");

cv::Mat current_image_A;
cv::Mat current_depthMat_A;
cv::Mat current_depthMat_A_crop1;
cv::Mat current_depthMat_A_crop2;
cv::Mat current_depthMat_A_crop3;

cv::Mat current_image_B;
cv::Mat current_depthMat_B;
cv::Mat current_depthMat_B_crop1;
cv::Mat current_depthMat_B_crop2;
cv::Mat current_depthMat_B_crop3;


cv::Mat rot_mat(3, 3, cv::DataType<float>::type);

ros::Time timestamp;
ros::Time last_frame;
ros::Time timestamp_depth;
ros::Time last_frame_depth;

std::recursive_mutex i_mutex, d_mutex, r_mutex2;
pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_A(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_A_acc(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_B(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_B_acc(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_A_filtered(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_B_filtered(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA_to_B_cropped(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA_to_B_cropped_filtered(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA_to_B(new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloudA_cropTotal(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloudB(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloudB_cropTotal(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud2_acc(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud3(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud3_acc(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud4(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud5(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud5_acc(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud6(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud6_acc(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_crop1(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_crop2(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_crop3(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudICP (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudICP1_AtoB (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudICP2_aTob (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudICP2_aTob_crop (new pcl::PointCloud<pcl::PointXYZ>);

pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;



Eigen::Matrix4f transform_A = Eigen::Matrix4f::Identity();
Eigen::Matrix4f transform_B = Eigen::Matrix4f::Identity();
Eigen::Affine3f transform_ICP = Eigen::Affine3f::Identity();
Eigen::Affine3f transform_ICP2 = Eigen::Affine3f::Identity();
Eigen::Matrix4f transform_ATOb;
Eigen::Matrix4f world_to_reference = Eigen::Matrix4f::Identity();
Eigen::Matrix4f world_to_B;
Eigen::Matrix4f transform_ICP1_print = Eigen::Matrix4f::Identity();
Eigen::Matrix4f transform_reference_global = Eigen::Matrix4f::Identity();


double ICP1_fitness_to_print;

// Default RGB parameters -> will be used in rectified depthMatrix since it is mapped onto RGB
// Try to upscale IR intrinsics to RGB size--> use them to transform depth map to point cloud
float const fx_default = 1081.37;
float const fy_default = 1081.37;
float const cx_default = 959.5;
float const cy_default = 539.5;

std::string package_path = "/home/sfi/catkin_ws/src/wp3_calibrator/";

Eigen::Matrix3f ir_params;

void max4points(std::vector<cv::Point2f> cornerPoints, float & topx, float & topy, float & botx, float & boty, bool &flag)
{
  //	std::shared_ptr<Point2f> john;
  std::vector<cv::Point2f> temp_max_values = cornerPoints;
  std::vector<cv::Point2f> temp_min_values = cornerPoints;
  float max_factor_x = 5;
  float min_factor_x = 5;
  float max_factor_y = 5;
  float min_factor_y = 5;
  flag = false;
  // Turn the flag "true" for invalid points

  for (int z = 0; z < 4; z++)
  {
    if ((cornerPoints[z].x < 0.1 && cornerPoints[z].x > 3000 ) || (cornerPoints[z].y < 0.1 && cornerPoints[z].y > 3000 ))
    {
      flag = true;
    }
    if (flag)
    {
      break;
    }
  }

  if (!flag)
  {
    topx = std::max(std::max(max_factor_x+temp_max_values[0].x,max_factor_x+temp_max_values[1].x),std::max(max_factor_x+temp_max_values[2].x,max_factor_x+temp_max_values[3].x));
    topx = floor(topx);

    topy = std::max(std::max(max_factor_y+temp_max_values[0].y,max_factor_y+temp_max_values[1].y),std::max(max_factor_y+temp_max_values[2].y,max_factor_y+temp_max_values[3].y));
    topy = floor(topy);

    botx = std::min(std::min(-min_factor_x+temp_min_values[0].x,-min_factor_x+temp_min_values[1].x),std::min(-min_factor_x+temp_min_values[2].x,-min_factor_x+temp_min_values[3].x));
    botx = floor(botx);

    boty = std::min(std::min(-min_factor_y+temp_min_values[0].y,-min_factor_y+temp_min_values[1].y),std::min(-min_factor_y+temp_min_values[2].y,-min_factor_y+temp_min_values[3].y));
    boty = floor(boty);
  }

  // Check if the padding is out of bound
  if ((topx > 1920) || (topy > 1080) || (topx <= 0) || (topy <= 0))
  {
    flag = true;
  }

  // Deallocate memory
  std::vector<cv::Point2f>().swap(temp_min_values);
  std::vector<cv::Point2f>().swap(temp_max_values);
}


void pointcloudFromDepthImage (cv::Mat& depth_image, Eigen::Matrix3f& depth_intrinsics,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud, bool crop = false, std::vector<float> c_ext = {}, cv::Mat depth_aruco_dummy = {})
{
  // For point clouds XYZ
  float depth_focal_inverted_x = 1/depth_intrinsics(0,0);  // 1/fx
  float depth_focal_inverted_y = 1/depth_intrinsics(1,1);  // 1/fy

  pcl::PointXYZ new_point;


  if (!crop)
  {
    output_cloud->points.resize(depth_image.cols*depth_image.rows, new_point);
    output_cloud->width = depth_image.cols;
    output_cloud->height = depth_image.rows;
    output_cloud->is_dense = false;

    for (int i=0;i<depth_image.rows;i++)
    {
      for (int j=0;j<depth_image.cols;j++)
      {
        float depth_value = depth_image.at<float>(i,j);

        if (depth_value > 0)
        {
          // Find 3D position with respect to depth frame:
          new_point.z = depth_value;
          new_point.x = (j - depth_intrinsics(0,2)) * new_point.z * depth_focal_inverted_x;
          new_point.y = (i - depth_intrinsics(1,2)) * new_point.z * depth_focal_inverted_y;
          output_cloud->at(j,i) = new_point;
        }

        else
        {
          new_point.z = std::numeric_limits<float>::quiet_NaN();
          new_point.x = std::numeric_limits<float>::quiet_NaN();
          new_point.y = std::numeric_limits<float>::quiet_NaN();
          output_cloud->at(j,i) = new_point;
        }
      }
    }
  }
  else
  {
    //	  aruco_cornerpoints = {botx, boty, topx, topy};
    output_cloud->points.resize((depth_aruco_dummy.cols)*(depth_aruco_dummy.rows), new_point);
    output_cloud->width = depth_aruco_dummy.cols;
    output_cloud->height = depth_aruco_dummy.rows;
    output_cloud->is_dense = false;
    float bot_row = c_ext[0];
    float top_row = c_ext[2];
    float bot_col = c_ext[1];
    float top_col = c_ext[3];

    for (int k=bot_col;k<top_col;k++)
    {
      for (int m=bot_row;m<top_row;m++)
      {
        float depth_value = depth_image.at<float>(k,m);

        if (depth_value > 0)
        {
          // Find 3D position with respect to depth frame:
          new_point.z = depth_value;
          new_point.x = (m - depth_intrinsics(0,2)) * new_point.z * depth_focal_inverted_x;
          new_point.y = (k - depth_intrinsics(1,2)) * new_point.z * depth_focal_inverted_y;
          output_cloud->at(m-bot_row, k-bot_col) = new_point;
        }

        else
        {
          new_point.z = std::numeric_limits<float>::quiet_NaN();
          new_point.x = std::numeric_limits<float>::quiet_NaN();
          new_point.y = std::numeric_limits<float>::quiet_NaN();
          output_cloud->at(m-bot_row, k-bot_col) = new_point;
        }
      }
    }
  }
}

void ICP_allign(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_xyz_org, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target_xyz_org, Eigen::Affine3f & transform_ICP, bool & converge_flag, float distThresh, double & fitnessScore)
{
  // >>>>>>>> Registration by ICP <<<<<<<<<
  // Declare output point cloud and initialize ICP object
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_robot_ICP(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  std::vector<int> indices1;
  std::vector<int> indices2;
  pcl::removeNaNFromPointCloud(*cloud_source_xyz_org,*cloud_source_xyz_org, indices1);
  pcl::removeNaNFromPointCloud(*cloud_target_xyz_org,*cloud_target_xyz_org, indices2);


  // Declare ICP parameters
  icp.setInputSource(cloud_source_xyz_org);
  icp.setInputTarget(cloud_target_xyz_org);
//  icp.setUseReciprocalCorrespondences(true);  //TULL testing AAA
//  icp.setRANSACOutlierRejectionThreshold(distThresh);  //TULL testing AAA
  icp.setMaxCorrespondenceDistance (distThresh);
  icp.setTransformationEpsilon(0.001);
  icp.setMaximumIterations (1000);
  // Perform the aligment
  icp.align(*cloud_robot_ICP);

  // Check for convergence
  fitnessScore = 1000; // if not converged
  converge_flag = icp.hasConverged();
  if (converge_flag)
  {
    transform_ICP = icp.getFinalTransformation();
    fitnessScore = icp.getFitnessScore();
    std::cout << "ICP converged with fitness score: " <<  icp.getFitnessScore() << std::endl;
  }
  else
  {
    std::cout << "ICP did not converge!" << std::endl;
  }

  cloud_robot_ICP->empty();
}



class imageConverter
{
  cv::Mat src_image, src_depth;
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;
  std::string topic_type;

private:
  std::string topicName;

public:
  imageConverter(const std::string& inputName, const std::string&  type) : it(nh)
  {
    //subscribe to the input video stream "/kinect2/hd/image_color"
    //r_mutex.lock();
    topic_type = inputName;
    if (type == "depth")
    {
      image_sub = it.subscribe(inputName, 1, &imageConverter::callback_depth, this, image_transport::TransportHints("compressed"));
    }

    else if (type == "color")
    {
      image_sub = it.subscribe(inputName, 1, &imageConverter::callback_color, this, image_transport::TransportHints("compressed"));
    }
    //r_mutex.unlock();
  }

  ~imageConverter()
  {
    image_sub.shutdown();
    std::cout << "ROS stopped Topic Subscription of " << topic_type <<  std::endl;
  }

  void getCurrentImage(cv::Mat *input_image)
  {
    while((timestamp.toSec() - last_frame.toSec()) <= 0) {
      usleep(2000);
      ros::spinOnce();
    }
    std::cout << "got new timestamp " <<  std::endl;
    i_mutex.lock();
    *input_image = src_image;
    last_frame = timestamp;
    i_mutex.unlock();
  }

  void getCurrentDepthMap(cv::Mat *input_image)
  {
    while((timestamp.toSec() - last_frame.toSec()) <= 0) {
      usleep(50); // changed from 2000
      ros::spinOnce();
    }
    d_mutex.lock();
    *input_image = src_depth;
    last_frame = timestamp;
    d_mutex.unlock();
  }


  void callback_depth(const sensor_msgs::ImageConstPtr& msg)
  {
    ros::Time frame_time = ros::Time::now();
    timestamp = frame_time;
    cv_bridge::CvImageConstPtr pCvImage;

    //                std::cout << "D1";

    try
    {
      pCvImage = cv_bridge::toCvShare(msg, msg->encoding);// same encoding of the message as the source,  sensor_msgs::image_encodings::16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //                std::cout << "D2";
    pCvImage->image.copyTo(src_depth);
    src_depth.convertTo(src_depth,  CV_32F, 0.001);


    // TULL Resizing
    //        cv::Mat tmp_depth;
    //        pCvImage->image.copyTo(tmp_depth);
    //        tmp_depth.convertTo(tmp_depth,  CV_32F, 0.001);
    //        // scale open cv image: TULL
    //        d_mutex.lock();
    //        cv::resize(tmp_depth, src_depth, cv::Size(1920,1080), 0, 0, cv::INTER_CUBIC); // resize to 1920x1080 resolution
    //        d_mutex.unlock();

    //                std::cout << "D3" << std::endl;
  }

  void callback_color(const sensor_msgs::ImageConstPtr& msg)
  {
    ros::Time frame_time = ros::Time::now();
    timestamp = frame_time;
    cv_bridge::CvImagePtr cv_ptr;

    //                std::cout << "C1";

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      //r_mutex.unlock();
      return;
    }

    //                std::cout << "C2";

    i_mutex.lock();
    src_image = cv_ptr->image;
    i_mutex.unlock();

    //                std::cout << "C3" << std::endl;
  }
};

class depthProcessor
{
  pcl::PointCloud<pcl::PointXYZ> tmp_cloud_read;
  ros::Subscriber depth_subA;
  ros::NodeHandle nh_depthA;

public:
  depthProcessor(const std::string& inputNameA)
  {
    depth_subA = nh_depthA.subscribe(inputNameA, 1, &depthProcessor::callback_cloudA, this);
    // this refers to the non static object  (here: a depthProcessor object used in main)
  }

  ~depthProcessor()
  {
    depth_subA.shutdown();
    std::cout << "ROS stopped Depth Import" << std::endl;
  }

  void get_filtered_PCL_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
  {
    while((timestamp_depth.toSec() - last_frame_depth.toSec()) <= 0) {
      usleep(2000);
      ros::spinOnce();
    }

    pcl::copyPointCloud(tmp_cloud_read, *cloud_ptr);
    //	    *cloud_ptr = tmp_cloud_read; // also a possibility
    last_frame_depth = timestamp_depth;
  }

  void callback_cloudA(const sensor_msgs::PointCloud2ConstPtr& input)
  {

    ros::Time frame_time_depth = ros::Time::now();
    timestamp_depth = frame_time_depth;
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2);

    pcl_conversions::toPCL(*input, *cloud_filtered);
    pcl::fromPCLPointCloud2(*cloud_filtered, tmp_cloud_read);
  }
};



class arucoProcessor
{
  float topx, topy, botx, boty;
  bool invalid_points = true;

public:

  void detectMarkers(cv::Mat &inputImage,cv::Mat &inputDepth, Eigen::Matrix4f & transform4x4, std::string kinect_number)
  {

    std::string camera_name = "/kinect" + kinect_number;
    std::string filename_inputPicture = "Input Picture" + camera_name;
    std::string filename_crop1 = "cropped image 1" + camera_name;
    std::string filename_crop2 = "cropped image 2" + camera_name;
    std::string rgb_intrinsics_dir = package_path + "kinect_calibrations" + camera_name + "/calib_color.yaml";
    std::string filename_crop3 = "cropped image 3" + camera_name;

    std::cout << rgb_intrinsics_dir << std::endl;

    ir_params(0,0) = fx_default;
    ir_params(1,1) = fy_default;
    ir_params(0,2) = cx_default;
    ir_params(1,2) = cy_default;

    src_cloud_crop1->clear();
    src_cloud_crop2->clear();
    src_cloud_crop3->clear();

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Parameters ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||

    float topx, topy, botx, boty;
    bool invalid_points = true;

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Cam. coeff. ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||
    cv::Mat cameraMatrix;
    cv::Mat dist_coeffs;

    cv::FileStorage fs_calibration(rgb_intrinsics_dir, cv::FileStorage::READ);
    fs_calibration["cameraMatrix"] >> cameraMatrix;
    fs_calibration["distortionCoefficients"] >> dist_coeffs;
    fs_calibration.release();

    std::cout << "camera matrix aruco : " << cameraMatrix << endl << "distortion coeffs: " << dist_coeffs << std::endl;

    //		ir_params(0,0) = cameraMatrix.at<double>(0,0);
    //		ir_params(1,1) = cameraMatrix.at<double>(1,1);
    //		ir_params(0,2) = cameraMatrix.at<double>(0,2);
    //		ir_params(1,2) = cameraMatrix.at<double>(1,2);

    //		cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    //		cameraMatrix.at<double>(0,0) = 1057.932708427644;
    //		cameraMatrix.at<double>(1,1) = 1061.07748780408;
    //		cameraMatrix.at<double>(0,2) = 975.901271112996;
    //		cameraMatrix.at<double>(1,2) = 524.072951512137;
    //		dist_coeffs = (cv::Mat_<double>(5,1) <<  0.0548992182401387,-0.0805160815549322,
    //				-0.000938325112541061, 0.00181830067859300, 0.0290103875818698);
    //		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ aruco param. ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||
    double arucoSquareDimension = 0.60; // in meters
    std::vector<int> markerIds; // markerIds.size() = number of Ids found
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
    //		params->doCornerRefinement = true;
    params->cornerRefinementWinSize = 5;
    params->cornerRefinementMaxIterations = 2000;
    params->adaptiveThreshConstant = true;
    params->cornerRefinementMinAccuracy = 0.001f;
    std::vector<cv::Vec3d> rotationVectors, translationVectors;
    cv::Ptr<cv::aruco::Dictionary> markerDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Marker Detection ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||
    //	r_mutex.lock();
    cv::aruco::detectMarkers(inputImage,markerDictionary,markerCorners,markerIds, params);
    cv::aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimension, cameraMatrix, dist_coeffs, rotationVectors, translationVectors);
    //		if (markerIds.size() != number_of_aruco)
    //		{
    //			return;
    //		}
    //	r_mutex.unlock();
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Marker Visualization ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||
    cv::aruco::drawDetectedMarkers(inputImage,markerCorners, markerIds);
    imshow(filename_inputPicture,inputImage);


    /* TODO - CREATE CUSTOM CROPPING LIKE THIS
                 * ROI by creating mask for the parallelogram
                Mat mask = cvCreateMat(480, 640, CV_8UC1);
                // Create black image with the same size as the original
                for(int i=0; i<mask.cols; i++)
                   for(int j=0; j<mask.rows; j++)
                       mask.at<uchar>(Point(i,j)) = 0;

                // Create Polygon from vertices
                vector<Point> ROI_Poly;
                approxPolyDP(ROI_Vertices, ROI_Poly, 1.0, true);

                // Fill polygon white
                fillConvexPoly(mask, &ROI_Poly[0], ROI_Poly.size(), 255, 8, 0);

                // Create new image for result storage
                Mat imageDest = cvCreateMat(480, 640, CV_8UC3);

                // Cut out ROI and store it in imageDest
                image->copyTo(imageDest, mask);
                */



    for (size_t i = 0; i < markerIds.size(); i++)
    {
      if (markerIds[i] == 1)
      {
        cv::aruco::drawAxis(inputImage, cameraMatrix, dist_coeffs, rotationVectors[i], translationVectors[i], 0.1);
        max4points(markerCorners[i], topx, topy, botx, boty, invalid_points);
        //				std::cout << "invalid points? " << markerCorners[i] << std::endl;
        if (!invalid_points)
        {
          cv::Rect myROI(botx,boty,topx-botx, topy-boty);
          cv::Mat croppedImage = inputImage(myROI);
          cv::imshow(filename_crop1,croppedImage);
          cv::waitKey(30);

          current_depthMat_A_crop1 = inputDepth(myROI);

          std::vector<float> aruco_cornerpoints = {botx, boty, topx, topy};
          pointcloudFromDepthImage(inputDepth, ir_params, src_cloud_crop1, true, aruco_cornerpoints, current_depthMat_A_crop1);

          // Create the transformation matrix
          cv::Mat rotation3x3 = cv::Mat::eye(3, 3, CV_64F);
          cv::Rodrigues(rotationVectors[i], rotation3x3);
          transform4x4(0,0) = rotation3x3.at<double>(0,0);
          transform4x4(1,0) = rotation3x3.at<double>(1,0);
          transform4x4(2,0) = rotation3x3.at<double>(2,0);
          transform4x4(0,1) = rotation3x3.at<double>(0,1);
          transform4x4(1,1) = rotation3x3.at<double>(1,1);
          transform4x4(2,1) = rotation3x3.at<double>(2,1);
          transform4x4(0,2) = rotation3x3.at<double>(0,2);
          transform4x4(1,2) = rotation3x3.at<double>(1,2);
          transform4x4(2,2) = rotation3x3.at<double>(2,2);

          transform4x4(0,3) = translationVectors[i].val[0]*1.0f;
          transform4x4(1,3) = translationVectors[i].val[1]*1.0f;
          transform4x4(2,3) = translationVectors[i].val[2]*1.0f;

        }
      }

      if (markerIds[i] == 13)
      {
        cv::aruco::drawAxis(inputImage, cameraMatrix, dist_coeffs, rotationVectors[i], translationVectors[i], 0.1);
        max4points(markerCorners[i], topx, topy, botx, boty, invalid_points);
        //				std::cout << "invalid points? " << markerCorners[i] << std::endl;
        if (!invalid_points)
        {
          cv::Rect myROI(botx,boty,topx-botx, topy-boty);
          cv::Mat croppedImage = inputImage(myROI);
          cv::imshow(filename_crop2,croppedImage);
          cv::waitKey(30);
          current_depthMat_A_crop2 = inputDepth(myROI);
          std::vector<float> aruco_cornerpoints = {botx, boty, topx, topy};
          pointcloudFromDepthImage(inputDepth, ir_params, src_cloud_crop2, true, aruco_cornerpoints, current_depthMat_A_crop2);

          //					sor.setInputCloud (src_cloud_crop2);
          //					sor.setMeanK (50);
          //					sor.setStddevMulThresh (0.4);
          //					sor.filter (*src_cloud_crop2);
          Eigen::Matrix4f tmatTemp = Eigen::Matrix4f::Identity();
          createTransMatrix(rotationVectors[i],translationVectors[i], tmatTemp);

        }
      }

      if (markerIds[i] == 40)
      {
        cv::aruco::drawAxis(inputImage, cameraMatrix, dist_coeffs, rotationVectors[i], translationVectors[i], 0.1);
        max4points(markerCorners[i], topx, topy, botx, boty, invalid_points);
        //				std::cout << "invalid points? " << markerCorners[i] << std::endl;
        if (!invalid_points)
        {
          cv::Rect myROI(botx,boty,topx-botx, topy-boty);
          cv::Mat croppedImage = inputImage(myROI);
          cv::imshow(filename_crop3,croppedImage);
          cv::waitKey(30);

          current_depthMat_A_crop3 = inputDepth(myROI);

          std::vector<float> aruco_cornerpoints = {botx, boty, topx, topy};
          pointcloudFromDepthImage(inputDepth, ir_params, src_cloud_crop3, true, aruco_cornerpoints, current_depthMat_A_crop3);
          //					sor.setInputCloud (src_cloud_crop3);
          //					sor.setMeanK (50);
          //					sor.setStddevMulThresh (0.4);
          //					sor.filter (*src_cloud_crop3);

          Eigen::Matrix4f tmatTemp = Eigen::Matrix4f::Identity();
          createTransMatrix(rotationVectors[i],translationVectors[i], tmatTemp);

        }
      }
    }
  }

  void getCroppedCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    *cloud += *src_cloud_crop1;
    *cloud += *src_cloud_crop2;
    *cloud += *src_cloud_crop3;
  }

  void createTransMatrix(cv::Vec3d rotationVectors, cv::Vec3d translationVectors, Eigen::Matrix4f& tMat)
  {
    // Create the transformation matrix
    cv::Mat rotation3x3 = cv::Mat::eye(3, 3, CV_64F);
    cv::Rodrigues(rotationVectors, rotation3x3);
    tMat(0,0) = rotation3x3.at<double>(0,0);
    tMat(1,0) = rotation3x3.at<double>(1,0);
    tMat(2,0) = rotation3x3.at<double>(2,0);
    tMat(0,1) = rotation3x3.at<double>(0,1);
    tMat(1,1) = rotation3x3.at<double>(1,1);
    tMat(2,1) = rotation3x3.at<double>(2,1);
    tMat(0,2) = rotation3x3.at<double>(0,2);
    tMat(1,2) = rotation3x3.at<double>(1,2);
    tMat(2,2) = rotation3x3.at<double>(2,2);

    tMat(0,3) = translationVectors.val[0]*1.0f;
    tMat(1,3) = translationVectors.val[1]*1.0f;
    tMat(2,3) = translationVectors.val[2]*1.0f;

  }
};

void cloudPassthroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_filtered, float* filter_limits)
{
  pcl::PassThrough<pcl::PointXYZ> passthrough; // note passthrough works only one filter at a time
  //	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_x(new pcl::PointCloud<pcl::PointXYZ>);
  passthrough.setInputCloud (cloud);
  passthrough.setFilterFieldName ("x");
  passthrough.setFilterLimits (filter_limits[0], filter_limits[1]);
  passthrough.filter (*cloud_filtered);

  passthrough.setInputCloud (cloud_filtered);
  passthrough.setFilterFieldName ("y");
  passthrough.setFilterLimits (filter_limits[2], filter_limits[3]);
  passthrough.filter (*cloud_filtered);

  passthrough.setInputCloud (cloud_filtered);
  passthrough.setFilterFieldName ("z");
  passthrough.setFilterLimits (filter_limits[4], filter_limits[5]);
  passthrough.filter (*cloud_filtered);
}





// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(cv::Mat &R)
{
  cv::Mat Rt;
  cv::transpose(R, Rt);
  cv::Mat shouldBeIdentity = Rt * R;
  std::cout << "should be identity: " << shouldBeIdentity << std::endl;
  cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());

  return  norm(I, shouldBeIdentity) < 1e-6;

}
cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R)
{


  //    assert(isRotationMatrix(R));
  float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

  bool singular = sy < 1e-6; // If

  float x, y, z;
  if (!singular)
  {
    x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
    y = atan2(-R.at<double>(2,0), sy);
    z = atan2(R.at<double>(1,0), R.at<double>(0,0));
  }
  else
  {
    x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
    y = atan2(-R.at<double>(2,0), sy);
    z = 0;
  }
  return cv::Vec3f(x, y, z);



}
void readGlobalPose(std::string kinect_number, Eigen::Matrix4f & tMat)
{
  std::string calibration_dir= package_path + "kinect_calibrations/" + kinect_number;

  cv::Mat kinect_number_global_pose = cv::Mat::eye(4, 4, CV_64F);
  cv::FileStorage fs_calibration(calibration_dir, cv::FileStorage::READ);

  fs_calibration["globalPose"] >> kinect_number_global_pose;
  fs_calibration.release();
  std::cout << kinect_number << kinect_number_global_pose << std::endl ;

  tMat(0,0) = kinect_number_global_pose.at<double>(0,0);
  tMat(1,0) = kinect_number_global_pose.at<double>(1,0);
  tMat(2,0) = kinect_number_global_pose.at<double>(2,0);
  tMat(0,1) = kinect_number_global_pose.at<double>(0,1);
  tMat(1,1) = kinect_number_global_pose.at<double>(1,1);
  tMat(2,1) = kinect_number_global_pose.at<double>(2,1);
  tMat(0,2) = kinect_number_global_pose.at<double>(0,2);
  tMat(1,2) = kinect_number_global_pose.at<double>(1,2);
  tMat(2,2) = kinect_number_global_pose.at<double>(2,2);

  tMat(0,3) = kinect_number_global_pose.at<double>(0,3);;
  tMat(1,3) = kinect_number_global_pose.at<double>(1,3);;
  tMat(2,3) = kinect_number_global_pose.at<double>(2,3);;
}
void readTopics(std::string nodeA, std::string nodeB, bool update = false)
{
  // Create char arrays
  std::string depthMatA = "/jetson" + nodeA + "/hd/image_depth_rect";
  //    std::string depthMatA = "/jetson" + nodeA + "/sd/image_depth_rect"; // TULL
  std::string rgbA = "/jetson" + nodeA + "/hd/image_color_rect";
  std::string point_cloud_A = "/master/jetson" + nodeA + "/points";

  std::string depthMatB = "/jetson" + nodeB + "/hd/image_depth_rect";
  //    std::string depthMatB = "/jetson" + nodeB + "/sd/image_depth_rect"; // TULL
  std::string rgbB = "/jetson" + nodeB + "/hd/image_color_rect";
  std::string point_cloud_B = "/master/jetson" + nodeB + "/points";

  std::cout << "pointA_name: " << point_cloud_A << std::endl;  //eg: /master/jetson1/points
  std::cout << "pointB_name: " << point_cloud_B << std::endl;

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Parameters ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  imageConverter IC_Depth_A = imageConverter(depthMatA, "depth");
  //        std::cout << depthMatA << " converted" << std::endl; //TULL
  imageConverter IC_RGB_A = imageConverter(rgbA, "color");
  //        std::cout << rgbA << " converted" << std::endl; //TULL

  imageConverter IC_Depth_B = imageConverter(depthMatB, "depth");
  //        std::cout << depthMatB << " converted" << std::endl; //TULL
  imageConverter IC_RGB_B = imageConverter(rgbB, "color"); // didnt use rect on clor image before
  //        std::cout << rgbB << " converted" << std::endl; //TULL

  depthProcessor dp_A = depthProcessor(point_cloud_A); // to fix: src_cloud in this class is single-linked in callback, quick fix-> create two src_clouds
  depthProcessor dp_B = depthProcessor(point_cloud_B); // to fix: src_cloud in this class is single-linked in callback, quick fix-> create two src_clouds

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  //// World data from Atle
  //
  //	//0.185 7.360 5.07 -1.590 0 -2.59
  //	Eigen::Matrix4f world_to_reference = Eigen::Matrix4f::Identity();
  //	Eigen::Matrix4f world_to_B;
  //	Eigen::Matrix4f world_to_B_inverse;
  //	Eigen::Affine3f tf4cb = Eigen::Affine3f::Identity();
  //	tf4cb.translation() << 0.138, 7.437, 4.930;
  //	tf4cb.rotate (Eigen::AngleAxisf (-1.565, Eigen::Vector3f::UnitZ()));
  //	tf4cb.rotate (Eigen::AngleAxisf (0, Eigen::Vector3f::UnitY()));
  //	tf4cb.rotate (Eigen::AngleAxisf (-2.590, Eigen::Vector3f::UnitX()));
  //	world_to_reference = tf4cb.matrix();
  //	kinect6_to_4 =  world_to_reference.inverse()* global_pose_kinect6;
  //	kinect3_to_4 =  world_to_reference.inverse()* global_pose_kinect3;
  //	kinect5_to_4 =  world_to_reference.inverse()* global_pose_kinect5;
  //	kinect2_to_4 =  world_to_reference.inverse()* global_pose_kinect2;
  //
  //
  //// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Data acquisition ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //	bool reading_complete_a;
  if (update)
  {
    current_image_A.release();
    current_depthMat_A.release();
    current_image_B.release();
    current_depthMat_B.release();
    current_cloud_A->clear();
    current_cloud_B->clear();
    src_cloudA_cropTotal->clear();
    src_cloudB_cropTotal->clear();

    std::cout << "emptied cloud, size now: " << current_cloud_A->size() << std::endl;
  }
  std::cout << "Reading RGB image A... "<< std::flush;
  while(current_image_A.empty())
  {
    IC_RGB_A.getCurrentImage(&current_image_A);

  }
  std::cout << "done" << std::endl;

  std::cout << "Reading Depth image A... "<< std::flush;
  while(current_depthMat_A.empty())
  {
    IC_Depth_A.getCurrentDepthMap(&current_depthMat_A);
  }
  std::cout << "done " << std::endl;

  std::cout << "Reading RGB image B... "<< std::flush;
  while(current_image_B.empty())
  {
    IC_RGB_B.getCurrentImage(&current_image_B);
  }
  std::cout << "done" << std::endl;

  std::cout << "Reading Depth image B... "<< std::flush;
  while(current_depthMat_B.empty())
  {
    IC_Depth_B.getCurrentDepthMap(&current_depthMat_B);
  }
  std::cout << "done" << std::endl;

  std::cout << "Reading cloud A... "<< std::flush;
  while( current_cloud_A->size() == 0)
  {
    dp_A.get_filtered_PCL_cloud(current_cloud_A);
  }
  std::cout << "done" << std::endl;

  std::cout << "Reading cloud B... "<< std::flush;
  while( current_cloud_B->size() == 0)
  {
    dp_B.get_filtered_PCL_cloud(current_cloud_B);
  }
  std::cout << "done" << std::endl;

  //    std::cout << "Reading point clouds from A and B 5 times... "<< std::endl;
}

void visualizeRegisSteps(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloudA_init, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloudB_init,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr & cloudA_Aruco, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloudA_ROI,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr & cloudA_ICP)
{

}
void calcTransMats(Eigen::Matrix4f transform_A, Eigen::Matrix4f transform_B, Eigen::Matrix4f transform_reference_global, Eigen::Matrix4f & world_to_B, double & fitnessScore_to_print)
{

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Camera A to camera B (Aruco) ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  transform_ATOb = transform_B*transform_A.inverse();
  pcl::transformPointCloud (*src_cloudA_cropTotal, *cloudA_to_B_cropped, transform_ATOb);
  pcl::transformPointCloud (*current_cloud_A, *cloudA_to_B, transform_ATOb);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Camera A to camera B (ROI ICP) ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  bool ICP1_converged;
  double fitnessScore1;
  ICP_allign(cloudA_to_B_cropped,src_cloudB_cropTotal,transform_ICP, ICP1_converged, 0.6, fitnessScore1);
  if (ICP1_converged)
  {
    Eigen::Matrix4f transform_AtoB_ICP = transform_ICP.matrix()*transform_B*transform_A.inverse();
    pcl::transformPointCloud (*src_cloudA_cropTotal, *cloudICP, transform_AtoB_ICP);
    pcl::transformPointCloud (*current_cloud_A, *cloudICP1_AtoB, transform_AtoB_ICP);
    world_to_B = transform_reference_global*transform_AtoB_ICP.inverse(); // value to be written
    //		std::cout << "world_to_reference: "<< transform_reference_global << std::endl;
    //		std::cout << "transform_AtoB_ICP: "<< transform_AtoB_ICP << std::endl;
    //		std::cout << "world_to b: "<< world_to_B << std::endl;

  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Camera A to Camera B (Final Fine ICP) ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ TEMP TULL
  bool ICP2_converged;
  double fitnessScore2;
  ICP_allign(cloudICP1_AtoB,current_cloud_B,transform_ICP2, ICP2_converged,0.3, fitnessScore2);
  fitnessScore_to_print = fitnessScore2;
  Eigen::Matrix4f transform_AtoB_ICP2;
  if (ICP2_converged)
  {
    transform_AtoB_ICP2 = transform_ICP2.matrix()*transform_ICP.matrix()*transform_B*transform_A.inverse();
    std::cout << "Transformation ICP2: "<< transform_AtoB_ICP2 << std::endl;
    pcl::transformPointCloud (*src_cloudA_cropTotal, *cloudICP2_aTob_crop, transform_AtoB_ICP2); // TULL
    pcl::transformPointCloud (*current_cloud_A, *cloudICP2_aTob, transform_AtoB_ICP2);

  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

void make4_happen_baby(std::string kinect_number)
{
  std::string fs_filename = package_path + "kinect_ICP1_tMat/kinect" + kinect_number + "/results.yaml";
  std::cout << "Creating global reference in file " << fs_filename << "  ... " << std::flush;

  //0.185 7.360 5.07 -1.590 0 -2.59 // lab before summer 2018
  // 0.5 5.05 4.4 -1.58 -0.010 -2.47 // outdoor test at MH summer 2018
  Eigen::Matrix4f world_to_reference = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f world_to_B;
  Eigen::Matrix4f world_to_B_inverse;
  Eigen::Affine3f tf4cb = Eigen::Affine3f::Identity();
  tf4cb.translation() << 0.5, 5.05, 4.4;
  tf4cb.rotate (Eigen::AngleAxisf (-1.58, Eigen::Vector3f::UnitZ()));
  tf4cb.rotate (Eigen::AngleAxisf (-0.01, Eigen::Vector3f::UnitY()));
  tf4cb.rotate (Eigen::AngleAxisf (-2.47, Eigen::Vector3f::UnitX()));
  world_to_reference = tf4cb.matrix();

  cv::Mat transf_to_save_openCV = cv::Mat::eye(4, 4, CV_64F);
  transf_to_save_openCV.at<double>(0,0) = world_to_reference(0,0);
  transf_to_save_openCV.at<double>(1,0) = world_to_reference(1,0);
  transf_to_save_openCV.at<double>(2,0) = world_to_reference(2,0);
  transf_to_save_openCV.at<double>(0,1) = world_to_reference(0,1);
  transf_to_save_openCV.at<double>(1,1) = world_to_reference(1,1);
  transf_to_save_openCV.at<double>(2,1) = world_to_reference(2,1);
  transf_to_save_openCV.at<double>(0,2) = world_to_reference(0,2);
  transf_to_save_openCV.at<double>(1,2) = world_to_reference(1,2);
  transf_to_save_openCV.at<double>(2,2) = world_to_reference(2,2);
  transf_to_save_openCV.at<double>(0,3) = world_to_reference(0,3);
  transf_to_save_openCV.at<double>(1,3) = world_to_reference(1,3);
  transf_to_save_openCV.at<double>(2,3) = world_to_reference(2,3);

  cv::FileStorage fs_result(fs_filename, cv::FileStorage::WRITE);
  fs_result << "Global_transformation" << transf_to_save_openCV;
  fs_result.release();
  std::cout << "done" << std::endl;
}
void saveResults(Eigen::Matrix4f transf_to_save, double ICP2_fitnessScore, std::string kinect_number)
{
  // The thought here is to save the final fine ICP results and the clouds before this transformation. Based on
  // the algorithm presented in the paper, the cloud with the lowest (best) ICP score is merged with the reference cloud.
  // New ICP scores are then calculated, and the selection process continues until all clouds have been merged.

  std::cout << " saving result  ... " << std::flush;

  // Convert to openCV matrix first
  cv::Mat transf_to_save_openCV = cv::Mat::eye(4, 4, CV_64F);
  transf_to_save_openCV.at<double>(0,0) = transf_to_save(0,0);
  transf_to_save_openCV.at<double>(1,0) = transf_to_save(1,0);
  transf_to_save_openCV.at<double>(2,0) = transf_to_save(2,0);
  transf_to_save_openCV.at<double>(0,1) = transf_to_save(0,1);
  transf_to_save_openCV.at<double>(1,1) = transf_to_save(1,1);
  transf_to_save_openCV.at<double>(2,1) = transf_to_save(2,1);
  transf_to_save_openCV.at<double>(0,2) = transf_to_save(0,2);
  transf_to_save_openCV.at<double>(1,2) = transf_to_save(1,2);
  transf_to_save_openCV.at<double>(2,2) = transf_to_save(2,2);
  transf_to_save_openCV.at<double>(0,3) = transf_to_save(0,3);
  transf_to_save_openCV.at<double>(1,3) = transf_to_save(1,3);
  transf_to_save_openCV.at<double>(2,3) = transf_to_save(2,3);

  std::string fs_filename = package_path + "kinect_ICP1_tMat/kinect" + kinect_number + "/results.yaml";
  cv::FileStorage fs_result(fs_filename, cv::FileStorage::WRITE);

  fs_result << "ICP2FitnessScore" << ICP2_fitnessScore;
  fs_result << "Global_transformation_ICP1" << transf_to_save_openCV;
  fs_result.release();

  std::cout << "Stored results in: " << fs_filename << std::endl;
}

void openGlobalReference(Eigen::Matrix4f & transf_to_open, std::string kinect_number)
{
  // The thought here is to open the final fine ICP results and the transformation from the first ICP. Based on
  // the algorithm presented in the paper, the cloud with the lowest (best) ICP score is merged with the reference cloud.
  // New ICP scores are then calculated, and the selection process continues until all clouds have been merged.

  std::string fs_filename = package_path + "kinect_ICP1_tMat/kinect" + kinect_number + "/results.yaml";
  std::cout << "Opening global reference: " << fs_filename << "  ... " << std::flush;

  cv::Mat transf_to_open_openCV = cv::Mat::eye(4, 4, CV_64F);

  cv::FileStorage fs_result(fs_filename, cv::FileStorage::READ);
  fs_result["Global_transformation"] >> transf_to_open_openCV;
  fs_result.release();
  // Convert to eigen matrix

  transf_to_open(0,0) = transf_to_open_openCV.at<double>(0,0);
  transf_to_open(1,0) = transf_to_open_openCV.at<double>(1,0);
  transf_to_open(2,0) = transf_to_open_openCV.at<double>(2,0);
  transf_to_open(0,1) = transf_to_open_openCV.at<double>(0,1);
  transf_to_open(1,1) = transf_to_open_openCV.at<double>(1,1);
  transf_to_open(2,1) = transf_to_open_openCV.at<double>(2,1);
  transf_to_open(0,2) = transf_to_open_openCV.at<double>(0,2);
  transf_to_open(1,2) = transf_to_open_openCV.at<double>(1,2);
  transf_to_open(2,2) = transf_to_open_openCV.at<double>(2,2);
  transf_to_open(0,3) = transf_to_open_openCV.at<double>(0,3);
  transf_to_open(1,3) = transf_to_open_openCV.at<double>(1,3);
  transf_to_open(2,3) = transf_to_open_openCV.at<double>(2,3);

  std::cout << "done" << std::endl;
}


int main (int argc, char** argv)
{
  sleep(3);

  std::cout << "n   - new image "<<  std::endl
            << "s   - save image "<<  std::endl
            << "+   - next sensor "<<  std::endl
            << "esc - quit "<<  std::endl;

  // ROS messaging init
  std::cout << "Starting: "<<  std::endl;
  ros::init(argc, argv, "aruco_tf_publisher");
  ros::NodeHandle nh;
  ros::spinOnce();
  arucoProcessor aruco_A;
  arucoProcessor aruco_B;
  wp3::Visualization viewer;

  std::string reference_node = "4";
  std::string calibration_order_initial[] = {"6", "3", "5", "2", "1"};

  //size_t numel_calib = sizeof(calibration_order_initial)/sizeof(calibration_order_initial[0]);
  size_t numel_calib = 5;

  std::cout << calibration_order_initial[0] << std::endl;

  make4_happen_baby(reference_node); // create first transformation
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Visualizer ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  pclVisColorCustom color_A(current_cloud_A, 255, 0, 255);
//  pclVisColorCustom color_B(current_cloud_B, 0, 255, 10);

  r_mutex2.lock();
  int key = cv::waitKey(30);
  bool init = true;
  size_t calib_counter = 0;
  size_t viz_counter = 0;

  openGlobalReference(transform_reference_global, reference_node);
  while ((key != 27) && ros::ok())  // ESC
  {
    key = cv::waitKey(30);

    // the process below is performed initially and updated every time the user presses the "n" key on the RGB image
    if (key == 110 || init == true) // n
    {
      init = false;
      readTopics(reference_node,calibration_order_initial[calib_counter],true);
      aruco_A.detectMarkers(current_image_A, current_depthMat_A, transform_A, reference_node);
      aruco_A.getCroppedCloud(src_cloudA_cropTotal);
      aruco_B.detectMarkers(current_image_B, current_depthMat_B, transform_B, calibration_order_initial[calib_counter]);
      aruco_B.getCroppedCloud(src_cloudB_cropTotal);
      calcTransMats(transform_A, transform_B, transform_reference_global, transform_ICP1_print, ICP1_fitness_to_print);
      viewer.run(current_cloud_B);
      //      wp3::runVisualizer(current_cloud_B, reg_viewer);
    }
    // if "s" is pressed on the RGB image the transformation from ICP1 and fitness result of ICP2 are saved
    if (key == 115) // s
    {
      saveResults(transform_ICP1_print, ICP1_fitness_to_print, calibration_order_initial[calib_counter]);
      std::cout << numel_calib << std::endl;
    }


    if (key == 43) // +
    {
      calib_counter += 1;

      if (calib_counter >= numel_calib)
      {
        calib_counter = 0;
      }
      std::cout << "evaluating node " << reference_node << "vs" << calibration_order_initial[calib_counter] << std::endl;
    }

// VIEWER -------------------------------------------------------------------------
//    wp3::updateVisualizer(reg_viewer);
    viewer.update();
// VIEWER -------------------------------------------------------------------------
    //                        visualizeCloud(cloudICP1_AtoB, "cloud1",  &viewer1, &color_A, 0.1);
    //                        visualizeCloud(current_cloud_B, "cloud2",  &viewer1, &color_B, 0.1);
    //                        visualizeCloud(src_cloudA_cropTotal, "cloud1_cropped", &viewer2, &color_A, 0.1);
    //                        visualizeCloud(src_cloudB_cropTotal, "cloud2_cropped", &viewer2, &color_B, 0.1);
//    visualizeRegisSteps(src_cloudA_cropTotal, src_cloudB_cropTotal,
//                        cloudA_to_B_cropped, cloudICP, cloudICP2_aTob_crop);

//    visualizeCloud(src_cloudA_cropTotal, "cloudA_init", "cloudA_init", &viewer2, &color_A, 0.1);
//    visualizeCloud(current_cloud_B, "cloudB_init","cloudB_init", &viewer1, &color_B, 0.01);
//    visualizeCloud(src_cloudB_cropTotal, "cloudB", "cloudB", &viewer2, &color_B, 0.01);

//    if (key == 42) // *
//    {
//      viz_counter += 1;
//      if (viz_counter > 3) {viz_counter=0;}
//      std::cout << "viz_counter: " << viz_counter << std::endl;
//      if(viz_counter==0) {visualizeCloud(current_cloud_A, "cloudA_init","BIG_ICP", &viewer1, &color_A, 0.01);}
//      if(viz_counter==1) {visualizeCloud(cloudA_to_B, "Aruco", "cloudA_init", &viewer1, &color_A, 0.01);}
//      if(viz_counter==2) {visualizeCloud(cloudICP1_AtoB, "ICP_ROI", "Aruco", &viewer1, &color_A, 0.01);}
//      if(viz_counter==3) {visualizeCloud(cloudICP2_aTob, "BIG_ICP", "ICP_ROI", &viewer1, &color_A, 0.01);}

//      if(viz_counter==0) {visualizeCloud(src_cloudA_cropTotal, "cloudA_init","BIG_ICP", &viewer2, &color_A, 0.01);}
//      if(viz_counter==1) {visualizeCloud(cloudA_to_B_cropped, "Aruco", "cloudA_init", &viewer2, &color_A, 0.01);}
//      if(viz_counter==2) {visualizeCloud(cloudICP, "ICP_ROI", "Aruco", &viewer2, &color_A, 0.01);}
//      if(viz_counter==3) {visualizeCloud(cloudICP2_aTob_crop, "BIG_ICP", "ICP_ROI", &viewer2, &color_A, 0.01);}
//    }
//    else{
//      if(viz_counter==0) {visualizeCloud(current_cloud_A, "cloudA_init","cloudA_init", &viewer1, &color_A, 0.01);}
//      if(viz_counter==1) {visualizeCloud(cloudA_to_B, "Aruco", "Aruco", &viewer1, &color_A, 0.01);}
//      if(viz_counter==2) {visualizeCloud(cloudICP1_AtoB, "ICP_ROI", "ICP_ROI", &viewer1, &color_A, 0.01);}
//      if(viz_counter==3) {visualizeCloud(cloudICP2_aTob, "BIG_ICP", "BIG_ICP", &viewer1, &color_A, 0.01);}

//      if(viz_counter==0) {visualizeCloud(src_cloudA_cropTotal, "cloudA_init","cloudA_init", &viewer2, &color_A, 0.01);}
//      if(viz_counter==1) {visualizeCloud(cloudA_to_B_cropped, "Aruco", "Aruco", &viewer2, &color_A, 0.01);}
//      if(viz_counter==2) {visualizeCloud(cloudICP, "ICP_ROI", "ICP_ROI", &viewer2, &color_A, 0.01);}
//      if(viz_counter==3) {visualizeCloud(cloudICP2_aTob_crop, "BIG_ICP", "BIG_ICP", &viewer2, &color_A, 0.01);}
//    }

//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA_to_B_cropped(new pcl::PointCloud<pcl::PointXYZ>);
//        pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloudA_cropTotal(new pcl::PointCloud<pcl::PointXYZ>);
//        pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloudB(new pcl::PointCloud<pcl::PointXYZ>);
//        pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloudB_cropTotal(new pcl::PointCloud<pcl::PointXYZ>);
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudICP (new pcl::PointCloud<pcl::PointXYZ>);
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudICP1_AtoB (new pcl::PointCloud<pcl::PointXYZ>);
//        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudICP2_aTob (new pcl::PointCloud<pcl::PointXYZ>);

  }
  //
  //	cv::Mat finalRotationMatrix = cv::Mat::eye(3, 3, CV_64F);
  //
  //	finalRotationMatrix.at<double>(0,0) = world_to_B_inverse(0,0);
  //	finalRotationMatrix.at<double>(1,0) = world_to_B_inverse(1,0);
  //	finalRotationMatrix.at<double>(2,0) = world_to_B_inverse(2,0);
  //	finalRotationMatrix.at<double>(0,1) = world_to_B_inverse(0,1);
  //    finalRotationMatrix.at<double>(1,1) = world_to_B_inverse(1,1);
  //    finalRotationMatrix.at<double>(2,1) = world_to_B_inverse(2,1);
  //    finalRotationMatrix.at<double>(0,2) = world_to_B_inverse(0,2);
  //    finalRotationMatrix.at<double>(1,2) = world_to_B_inverse(1,2);
  //    finalRotationMatrix.at<double>(2,2) = world_to_B_inverse(2,2);
  //
  ////	std::cout << "Camera A Aruco: "<< transform_A << std::endl;
  ////	std::cout << "Camera B Aruco: "<< transform_B << std::endl;
  //	std::cout << "Transformation from a to b: "<<transform_ATOb << std::endl;
  //	cv::Vec3f world_to_b_Angles = rotationMatrixToEulerAngles(finalRotationMatrix);
  //
  //
  //	std::cout << "world_to_B_inverse: "<< world_to_B_inverse << std::endl;
  //	std::cout << "world_to_B_inverse angles(custom_xyz): "<< world_to_b_Angles << std::endl;
  //
  //	std::cout << "Result: "<< world_to_B_inverse(0,3)<< " " << world_to_B_inverse(1,3)<< " " << world_to_B_inverse(2,3)<< " ";
  //	std::cout << world_to_b_Angles(2) << " " << world_to_b_Angles(1) << " " << world_to_b_Angles(0) << std::endl;


  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  r_mutex2.unlock();
  }
