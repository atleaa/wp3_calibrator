/*
 * functions.cpp
 *
 *  Created on: Sep, 2018
 *      Author: Atle Aalerud
 */

#include "wp3_calibrator/functions.h"
#include "wp3_calibrator/defines.h"
#include "wp3_calibrator/imageconverter.h"


namespace wp3 {

void init_reference(std::string kinect_number)
{
  std::string fs_filename = wp3::package_path + "kinect_ICP1_tMat/kinect" + kinect_number + "/results.yaml";
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


void openGlobalReference(Eigen::Matrix4f & transf_to_open, std::string kinect_number)
{
  // The thought here is to open the final fine ICP results and the transformation from the first ICP. Based on
  // the algorithm presented in the paper, the cloud with the lowest (best) ICP score is merged with the reference cloud.
  // New ICP scores are then calculated, and the selection process continues until all clouds have been merged.

  std::string fs_filename = wp3::package_path + "kinect_ICP1_tMat/kinect" + kinect_number + "/results.yaml";
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





void readTopics(std::string nodeA,
                std::string nodeB,
                cv::Mat* rgb_A,
                cv::Mat* depth_A,
                cv::Mat* rgb_B,
                cv::Mat* depth_B,
                pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_A,
                pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_B,
                pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloudA_cropTotal,
                pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloudB_cropTotal,
                bool update = false)
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
  wp3::imageConverter IC_Depth_A(depthMatA, "depth");
  //        std::cout << depthMatA << " converted" << std::endl; //TULL
  wp3::imageConverter IC_RGB_A(rgbA, "color");
  //        std::cout << rgbA << " converted" << std::endl; //TULL

  imageConverter IC_Depth_B(depthMatB, "depth");
  //        std::cout << depthMatB << " converted" << std::endl; //TULL
  imageConverter IC_RGB_B(rgbB, "color"); // didnt use rect on clor image before
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
    rgb_A->release();
    depth_A->release();
    rgb_B->release();
    depth_B->release();
    current_cloud_A->clear();
    current_cloud_B->clear();
    src_cloudA_cropTotal->clear();
    src_cloudB_cropTotal->clear();

    std::cout << "emptied cloud, size now: " << current_cloud_A->size() << std::endl;
  }
  std::cout << "Reading RGB image A... "<< std::flush;
  while(rgb_A->empty())
  {
    IC_RGB_A.getCurrentImage(rgb_A);

  }
  std::cout << "done" << std::endl;

  std::cout << "Reading Depth image A... "<< std::flush;
  while(depth_A->empty())
  {
    IC_Depth_A.getCurrentDepthMap(depth_A);
  }
  std::cout << "done " << std::endl;

  std::cout << "Reading RGB image B... "<< std::flush;
  while(rgb_B->empty())
  {
    IC_RGB_B.getCurrentImage(rgb_B);
  }
  std::cout << "done" << std::endl;

  std::cout << "Reading Depth image B... "<< std::flush;
  while(depth_B->empty())
  {
    IC_Depth_B.getCurrentDepthMap(depth_B);
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


} // end namespace wp3
