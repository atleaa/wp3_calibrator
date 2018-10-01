
#include "wp3_calibrator/defines.h"
#include "wp3_calibrator/classtest.h"
#include "wp3_calibrator/functions.h"
#include "wp3_calibrator/visualization.h"
#include "wp3_calibrator/arucoprocessor.h"
#include "wp3_calibrator/imageconverter.h"
#include "wp3_calibrator/sensor.h"


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
//#include "opencv2/opencv.hpp"

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
//please use <pcl/conversions.h> instead. [-Wcpp] #warning The <pcl/ros/conversions.h> header is deprecated. please use #include <pcl_conversions/pcl_conversions.h>


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



// Global variables:
//std::vector<cv::Vec3f> camera_colors;     // vector containing colors to use to identify cameras in the network
std::map<std::string, int> color_map;     // map between camera frame_id and color
//std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vector_1, cloud_vector_2, cloud_vector_3, cloud_vector_4, cloud_vector_5, cloud_vector_6;



//cv::Mat current_depthMat_A_crop1;
//cv::Mat current_depthMat_A_crop2;
//cv::Mat current_depthMat_A_crop3;


//cv::Mat current_depthMat_B_crop1;
//cv::Mat current_depthMat_B_crop2;
//cv::Mat current_depthMat_B_crop3;


cv::Mat rot_mat(3, 3, cv::DataType<float>::type);


std::recursive_mutex r_mutex2;


//pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_A(new pcl::PointCloud<pcl::PointXYZ>);
//pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_A_acc(new pcl::PointCloud<pcl::PointXYZ>);
//pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_B(new pcl::PointCloud<pcl::PointXYZ>);
//pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_B_acc(new pcl::PointCloud<pcl::PointXYZ>);
//pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_A_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_B_filtered(new pcl::PointCloud<pcl::PointXYZ>);
////pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA_to_B_cropped(new pcl::PointCloud<pcl::PointXYZ>);
//pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA_to_B_cropped_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA_to_B(new pcl::PointCloud<pcl::PointXYZ>);

////pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloudA_cropTotal(new pcl::PointCloud<pcl::PointXYZ>);
//pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloudB(new pcl::PointCloud<pcl::PointXYZ>);
////pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloudB_cropTotal(new pcl::PointCloud<pcl::PointXYZ>);
//pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
//pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud2_acc(new pcl::PointCloud<pcl::PointXYZ>);
//pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud3(new pcl::PointCloud<pcl::PointXYZ>);
//pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud3_acc(new pcl::PointCloud<pcl::PointXYZ>);
//pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud4(new pcl::PointCloud<pcl::PointXYZ>);
//pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud5(new pcl::PointCloud<pcl::PointXYZ>);
//pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud5_acc(new pcl::PointCloud<pcl::PointXYZ>);
//pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud6(new pcl::PointCloud<pcl::PointXYZ>);
//pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud6_acc(new pcl::PointCloud<pcl::PointXYZ>);
//pcl::PointCloud<pcl::PointXYZ>::Ptr cloudICP (new pcl::PointCloud<pcl::PointXYZ>);
//pcl::PointCloud<pcl::PointXYZ>::Ptr cloudICP1_AtoB (new pcl::PointCloud<pcl::PointXYZ>);
//pcl::PointCloud<pcl::PointXYZ>::Ptr cloudICP2_aTob (new pcl::PointCloud<pcl::PointXYZ>);
//pcl::PointCloud<pcl::PointXYZ>::Ptr cloudICP2_aTob_crop (new pcl::PointCloud<pcl::PointXYZ>);

pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;




// ==============================================================
// init images
cv::Mat current_image_A;
cv::Mat current_depthMat_A;
cv::Mat current_image_B;
cv::Mat current_depthMat_B;

// init clouds
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
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudICP (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudICP1_AtoB (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudICP2_aTob (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudICP2_aTob_crop (new pcl::PointCloud<pcl::PointXYZ>);

// init cloud vectors
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vector_1, cloud_vector_2, cloud_vector_3, cloud_vector_4, cloud_vector_5, cloud_vector_6;
//  pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloudA_cropTotal(new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloudB_cropTotal(new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudA_to_B_cropped(new pcl::PointCloud<pcl::PointXYZ>);

// init transforms
Eigen::Matrix4f transform_A = Eigen::Matrix4f::Identity();
Eigen::Matrix4f transform_B = Eigen::Matrix4f::Identity();
Eigen::Affine3f transform_ICP = Eigen::Affine3f::Identity();
Eigen::Affine3f transform_ICP2 = Eigen::Affine3f::Identity();
//Eigen::Matrix4f transform_ATOb;
Eigen::Matrix4f world_to_reference = Eigen::Matrix4f::Identity();
Eigen::Matrix4f world_to_B;
Eigen::Matrix4f transform_ICP1_print = Eigen::Matrix4f::Identity();
Eigen::Matrix4f transform_reference_global = Eigen::Matrix4f::Identity();

// init variables
double ICP1_fitness_to_print;
// ================================================================













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
  std::string calibration_dir= wp3::package_path + "kinect_calibrations/" + kinect_number;

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

//void calcTransMats(Eigen::Matrix4f transform_A, Eigen::Matrix4f transform_B, Eigen::Matrix4f transform_reference_global, Eigen::Matrix4f & world_to_B, double & fitnessScore_to_print)
void calcTransMats(wp3::Sensor &nodeA_local, wp3::Sensor &nodeB_local, Eigen::Matrix4f transform_A, Eigen::Matrix4f transform_B, Eigen::Matrix4f transform_reference_global, Eigen::Matrix4f & world_to_B, double & fitnessScore_to_print)
{
  Eigen::Matrix4f transform_ATOb;

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Camera A to camera B (Aruco) ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  transform_ATOb = transform_B*transform_A.inverse();
  pcl::transformPointCloud (*src_cloudA_cropTotal, *cloudA_to_B_cropped, transform_ATOb);
  pcl::transformPointCloud (*nodeA_local.cloudPtr_, *cloudA_to_B, transform_ATOb);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Camera A to camera B (ROI ICP) ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  bool ICP1_converged;
  double fitnessScore1;
  ICP_allign(cloudA_to_B_cropped,src_cloudB_cropTotal,transform_ICP, ICP1_converged, 0.6, fitnessScore1);
  if (ICP1_converged)
  {
    Eigen::Matrix4f transform_AtoB_ICP = transform_ICP.matrix()*transform_B*transform_A.inverse();
    pcl::transformPointCloud (*src_cloudA_cropTotal, *cloudICP, transform_AtoB_ICP);
    pcl::transformPointCloud (*nodeA_local.cloudPtr_, *cloudICP1_AtoB, transform_AtoB_ICP);
    world_to_B = transform_reference_global*transform_AtoB_ICP.inverse(); // value to be written
    //		std::cout << "world_to_reference: "<< transform_reference_global << std::endl;
    //		std::cout << "transform_AtoB_ICP: "<< transform_AtoB_ICP << std::endl;
    //		std::cout << "world_to b: "<< world_to_B << std::endl;

  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Camera A to Camera B (Final Fine ICP) ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ TEMP TULL
  bool ICP2_converged;
  double fitnessScore2;
  ICP_allign(cloudICP1_AtoB,nodeB_local.cloudPtr_,transform_ICP2, ICP2_converged,0.3, fitnessScore2);
  fitnessScore_to_print = fitnessScore2;
  Eigen::Matrix4f transform_AtoB_ICP2;
  if (ICP2_converged)
  {
    transform_AtoB_ICP2 = transform_ICP2.matrix()*transform_ICP.matrix()*transform_B*transform_A.inverse();
    std::cout << "Transformation ICP2: "<< transform_AtoB_ICP2 << std::endl;
    pcl::transformPointCloud (*src_cloudA_cropTotal, *cloudICP2_aTob_crop, transform_AtoB_ICP2); // TULL
    pcl::transformPointCloud (*nodeA_local.cloudPtr_, *cloudICP2_aTob, transform_AtoB_ICP2);

  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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

  std::string fs_filename = wp3::package_path + "kinect_ICP1_tMat/kinect" + kinect_number + "/results.yaml";
  cv::FileStorage fs_result(fs_filename, cv::FileStorage::WRITE);

  fs_result << "ICP2FitnessScore" << ICP2_fitnessScore;
  fs_result << "Global_transformation_ICP1" << transf_to_save_openCV;
  fs_result.release();

  std::cout << "Stored results in: " << fs_filename << std::endl;
}







// begin main  --------------------------------------------------------------------------------------------
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

  // TODO: move creation of arucoprocessor?
  wp3::arucoProcessor aruco_A;
  wp3::arucoProcessor aruco_B;

  wp3::Visualization viewer;

  std::string reference_node = "4";
  std::string calibration_order_initial[] = {"6", "3", "5", "2", "1"};

  //size_t numel_calib = sizeof(calibration_order_initial)/sizeof(calibration_order_initial[0]);
  size_t numel_calib = 5;

  std::cout << calibration_order_initial[0] << std::endl;

  // TODO: make node vector
  wp3::Sensor nodeA;
  nodeA.setDepthTopic("/jetson4/hd/image_depth_rect");
  nodeA.setImageTopic("/jetson4/hd/image_color_rect");
  nodeA.setCloudTopic("/master/jetson4/points");

  wp3::Sensor nodeB;
  nodeB.setDepthTopic("/jetson6/hd/image_depth_rect");
  nodeB.setImageTopic("/jetson6/hd/image_color_rect");
  nodeB.setCloudTopic("/master/jetson6/points");


  wp3::init_reference(reference_node); // create first initial transformation

  r_mutex2.lock();
  int key = cv::waitKey(30);
  bool init = true;
  size_t calib_counter = 0;
  size_t viz_counter = 0;



  wp3::openGlobalReference(transform_reference_global, reference_node);

  //  begin main while ------------------------------------------------------------------------------------------
  while ((key != 27) && ros::ok())  // ESC
  {
    key = cv::waitKey(30);

    // the process below is performed initially and updated every time the user presses the "n" key on the RGB image
    if (key == 110 || init == true) // n
    {
      init = false;
//      wp3::readTopics(reference_node,
//                      calibration_order_initial[calib_counter],
//                      &current_image_A,
//                      &current_depthMat_A,
//                      &current_image_B,
//                      &current_depthMat_B,
//                      current_cloud_A,
//                      current_cloud_B,
//                      src_cloudA_cropTotal,
//                      src_cloudB_cropTotal,
//                      true);
      nodeA.readTopics(true);
      nodeB.readTopics(true);

//      aruco_A.detectMarkers(current_image_A, current_depthMat_A, transform_A, reference_node);
//      aruco_A.getCroppedCloud(src_cloudA_cropTotal);
//      aruco_B.detectMarkers(current_image_B, current_depthMat_B, transform_B, calibration_order_initial[calib_counter]);
//      aruco_B.getCroppedCloud(src_cloudB_cropTotal);
//      calcTransMats(transform_A, transform_B, transform_reference_global, transform_ICP1_print, ICP1_fitness_to_print);
//      cv::Mat tmpMat1 = nodeA.getImageMat();
//      aruco_A.detectMarkers(nodeA.getImageMat(), current_depthMat_A, transform_A, reference_node);

      aruco_A.detectMarkers(nodeA.imageMat_, nodeA.depthMat_, transform_A, reference_node);
      aruco_A.getCroppedCloud(src_cloudA_cropTotal);

      aruco_B.detectMarkers(nodeB.imageMat_, nodeB.depthMat_, transform_B, calibration_order_initial[calib_counter]);
      aruco_B.getCroppedCloud(src_cloudB_cropTotal);

      calcTransMats(nodeA, nodeB, transform_A, transform_B, transform_reference_global, transform_ICP1_print, ICP1_fitness_to_print);


      //make cloud vector --> TODO: use a loop to create vectors
//      for (unsigned int i = 0; i < curr_cloud_vector.size(); i++)
//      {
//        pcl::PointCloud<pcl::PointXYZRGB>::Ptr empty_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

      // cropped clouds
      cloud_vector_1.clear();
      cloud_vector_1.push_back(cloudA_to_B_cropped);
      cloud_vector_1.push_back(src_cloudB_cropTotal);
      cloud_vector_2.clear();
      cloud_vector_2.push_back(cloudICP);
      cloud_vector_2.push_back(src_cloudB_cropTotal);
      cloud_vector_3.clear();
      cloud_vector_3.push_back(cloudICP2_aTob_crop);
      cloud_vector_3.push_back(src_cloudB_cropTotal);

      // full clouds
      cloud_vector_4.clear();
      cloud_vector_4.push_back(cloudA_to_B);
      cloud_vector_4.push_back(nodeB.cloudPtr_);
      cloud_vector_5.clear();
      cloud_vector_5.push_back(cloudICP1_AtoB);
      cloud_vector_5.push_back(nodeB.cloudPtr_);
      cloud_vector_6.clear();
      cloud_vector_6.push_back(cloudICP2_aTob);
      cloud_vector_6.push_back(nodeB.cloudPtr_);

      viewer.run(cloud_vector_1, cloud_vector_2, cloud_vector_3, cloud_vector_4, cloud_vector_5, cloud_vector_6);
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

    viewer.update();
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
  } // end main
