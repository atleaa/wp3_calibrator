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

void loadSensors(ros::NodeHandle & node_handle, std::vector<wp3::Sensor::Ptr> & sensorVec, int & num_sensors)
{
//  int num_sensors;
  node_handle.param("num_sensors", num_sensors, 0);

  for (int i = 0; i < num_sensors; ++i)
  {
    std::string tmpString;

    tmpString = "sensor_" + std::to_string(i) + "/name";
    if (node_handle.param(tmpString, tmpString, tmpString))
      ROS_DEBUG_STREAM("Loading parameters for " <<  tmpString);
    else
      ROS_FATAL_STREAM("No name parameter found for " << tmpString);
    wp3::Sensor::Ptr sensorNode = boost::make_shared<wp3::Sensor>(tmpString, node_handle);

    tmpString = "sensor_" + std::to_string(i) + "/camera_info";
    node_handle.param(tmpString, tmpString, tmpString);
    sensorNode->setCamera_info_topic(tmpString);

    tmpString = "sensor_" + std::to_string(i) + "/color";
    node_handle.param(tmpString, tmpString, tmpString);
    sensorNode->setImageTopic(tmpString);

    tmpString = "sensor_" + std::to_string(i) + "/depth";
    node_handle.param(tmpString, tmpString, tmpString);
    sensorNode->setDepthTopic(tmpString);

    tmpString = "sensor_" + std::to_string(i) + "/cloud";
    node_handle.param(tmpString, tmpString, tmpString);
    sensorNode->setCloudTopic(tmpString);

    tmpString = "sensor_" + std::to_string(i) + "/tf";
    node_handle.param(tmpString, tmpString, tmpString);
    sensorNode->setTfTopic(tmpString);

    sensorVec.push_back(sensorNode);
  }
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


//void readTopics(std::string nodeA,
//                std::string nodeB,
//                cv::Mat* rgb_A,
//                cv::Mat* depth_A,
//                cv::Mat* rgb_B,
//                cv::Mat* depth_B,
//                pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_A,
//                pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud_B,
//                pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloudA_cropTotal,
//                pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloudB_cropTotal,
//                bool update = false)
//{
//  // Create char arrays
//  std::string depthMatA = "/jetson" + nodeA + "/hd/image_depth_rect";
//  //    std::string depthMatA = "/jetson" + nodeA + "/sd/image_depth_rect"; // TULL
//  std::string rgbA = "/jetson" + nodeA + "/hd/image_color_rect";
//  std::string point_cloud_A = "/master/jetson" + nodeA + "/points";

//  std::string depthMatB = "/jetson" + nodeB + "/hd/image_depth_rect";
//  //    std::string depthMatB = "/jetson" + nodeB + "/sd/image_depth_rect"; // TULL
//  std::string rgbB = "/jetson" + nodeB + "/hd/image_color_rect";
//  std::string point_cloud_B = "/master/jetson" + nodeB + "/points";

//  std::cout << "pointA_name: " << point_cloud_A << std::endl;  //eg: /master/jetson1/points
//  std::cout << "pointB_name: " << point_cloud_B << std::endl;

//  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Parameters ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  wp3::imageConverter IC_Depth_A(depthMatA, "depth");
//  //        std::cout << depthMatA << " converted" << std::endl; //TULL
//  wp3::imageConverter IC_RGB_A(rgbA, "color");
//  //        std::cout << rgbA << " converted" << std::endl; //TULL

//  imageConverter IC_Depth_B(depthMatB, "depth");
//  //        std::cout << depthMatB << " converted" << std::endl; //TULL
//  imageConverter IC_RGB_B(rgbB, "color"); // didnt use rect on clor image before
//  //        std::cout << rgbB << " converted" << std::endl; //TULL

//  depthProcessor dp_A = depthProcessor(point_cloud_A); // to fix: src_cloud in this class is single-linked in callback, quick fix-> create two src_clouds
//  depthProcessor dp_B = depthProcessor(point_cloud_B); // to fix: src_cloud in this class is single-linked in callback, quick fix-> create two src_clouds

//  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//  //// World data from Atle
//  //
//  //	//0.185 7.360 5.07 -1.590 0 -2.59
//  //	Eigen::Matrix4f world_to_reference = Eigen::Matrix4f::Identity();
//  //	Eigen::Matrix4f world_to_B;
//  //	Eigen::Matrix4f world_to_B_inverse;
//  //	Eigen::Affine3f tf4cb = Eigen::Affine3f::Identity();
//  //	tf4cb.translation() << 0.138, 7.437, 4.930;
//  //	tf4cb.rotate (Eigen::AngleAxisf (-1.565, Eigen::Vector3f::UnitZ()));
//  //	tf4cb.rotate (Eigen::AngleAxisf (0, Eigen::Vector3f::UnitY()));
//  //	tf4cb.rotate (Eigen::AngleAxisf (-2.590, Eigen::Vector3f::UnitX()));
//  //	world_to_reference = tf4cb.matrix();
//  //	kinect6_to_4 =  world_to_reference.inverse()* global_pose_kinect6;
//  //	kinect3_to_4 =  world_to_reference.inverse()* global_pose_kinect3;
//  //	kinect5_to_4 =  world_to_reference.inverse()* global_pose_kinect5;
//  //	kinect2_to_4 =  world_to_reference.inverse()* global_pose_kinect2;
//  //
//  //
//  //// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Data acquisition ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  //	bool reading_complete_a;
//  if (update)
//  {
//    rgb_A->release();
//    depth_A->release();
//    rgb_B->release();
//    depth_B->release();
//    current_cloud_A->clear();
//    current_cloud_B->clear();
//    src_cloudA_cropTotal->clear();
//    src_cloudB_cropTotal->clear();

//    std::cout << "emptied cloud, size now: " << current_cloud_A->size() << std::endl;
//  }
//  std::cout << "Reading RGB image A... "<< std::flush;
//  while(rgb_A->empty())
//  {
//    IC_RGB_A.getCurrentImage(rgb_A);

//  }
//  std::cout << "done" << std::endl;

//  std::cout << "Reading Depth image A... "<< std::flush;
//  while(depth_A->empty())
//  {
//    IC_Depth_A.getCurrentDepthMap(depth_A);
//  }
//  std::cout << "done " << std::endl;

//  std::cout << "Reading RGB image B... "<< std::flush;
//  while(rgb_B->empty())
//  {
//    IC_RGB_B.getCurrentImage(rgb_B);
//  }
//  std::cout << "done" << std::endl;

//  std::cout << "Reading Depth image B... "<< std::flush;
//  while(depth_B->empty())
//  {
//    IC_Depth_B.getCurrentDepthMap(depth_B);
//  }
//  std::cout << "done" << std::endl;

//  std::cout << "Reading cloud A... "<< std::flush;
//  while( current_cloud_A->size() == 0)
//  {
//    dp_A.get_filtered_PCL_cloud(current_cloud_A);
//  }
//  std::cout << "done" << std::endl;

//  std::cout << "Reading cloud B... "<< std::flush;
//  while( current_cloud_B->size() == 0)
//  {
//    dp_B.get_filtered_PCL_cloud(current_cloud_B);
//  }
//  std::cout << "done" << std::endl;

//  //    std::cout << "Reading point clouds from A and B 5 times... "<< std::endl;
//}


void ICP_allign(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_xyz_org,
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target_xyz_org,
                Eigen::Affine3f & transform_ICP,
                bool & converge_flag, float maxCorrDist, int iter, double & fitnessScore)
{
  // >>>>>>>> Registration by ICP <<<<<<<<<
  // Declare output point cloud and initialize ICP object
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ICP(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  std::vector<int> indices1;
  std::vector<int> indices2;
  pcl::removeNaNFromPointCloud(*cloud_source_xyz_org,*cloud_source_xyz_org, indices1);
  pcl::removeNaNFromPointCloud(*cloud_target_xyz_org,*cloud_target_xyz_org, indices2);


  // Declare ICP parameters
  icp.setInputSource(cloud_source_xyz_org);
  icp.setInputTarget(cloud_target_xyz_org);
//  icp.setUseReciprocalCorrespondences(true);  //TULL testing AAA
  icp.setRANSACOutlierRejectionThreshold(maxCorrDist);  //TULL testing AAA
  icp.setMaxCorrespondenceDistance (maxCorrDist);
  // Termination criteria
  icp.setMaximumIterations (iter);
  icp.setTransformationEpsilon (ICP_TRANS_EPSILON);
//  icp.setEuclideanFitnessEpsilon (1e-8);

  // Perform the aligment
  icp.align(*cloud_ICP);

  // Check for convergence
  fitnessScore = 1000; // if not converged
  converge_flag = icp.hasConverged();
  if (converge_flag)
  {
    transform_ICP = icp.getFinalTransformation();
    fitnessScore = icp.getFitnessScore();
//    ROS_DEBUG_STREAM("ICP converged with fitness score: " <<  icp.getFitnessScore());
  }
  else
  {
//    ROS_ERROR_STREAM("ICP did not converge!");
  }

  cloud_ICP->empty();
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
//  std::cout << "should be identity: " << shouldBeIdentity << std::endl;
  cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());
//  std::cout << "norm: " << std::endl << norm(I, shouldBeIdentity) << std::endl;
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

// not in use?..
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


double calcMedian(std::vector<float> values)
{
  size_t size = values.size();

  if (size == 0)
  {
    return 0;  // Undefined, really.
  }
  else
  {
    sort(values.begin(), values.end());
    if (size % 2 == 0)
    {
      return (values[size / 2 - 1] + values[size / 2]) / 2;
    }
    else
    {
      return values[size / 2];
    }
  }
}



#ifdef VIEW_ICP
void calcTransMats(wp3::Sensor &sensorA, wp3::Sensor &sensorB,
                   Eigen::Matrix4f transform_A, Eigen::Matrix4f transform_B,
                   Eigen::Matrix4f transform_reference_global, Eigen::Matrix4f & trans_AtoB, double & fitnessScore,
//                   pcl::visualization::PCLVisualizer viewerICP)
                   wp3::Visualization & viewer)
#else
void calcTransMats(wp3::Sensor &sensorA, wp3::Sensor &sensorB,
                   Eigen::Matrix4f transform_A, Eigen::Matrix4f transform_B,
                   Eigen::Matrix4f transform_reference_global, Eigen::Matrix4f & world_to_B, double & fitnessScore)
#endif
{
  Eigen::Matrix4f transform_ATOb;
  Eigen::Affine3f transform_ICP = Eigen::Affine3f::Identity();
  Eigen::Affine3f transform_ICP2 = Eigen::Affine3f::Identity();
  pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud2 (new pcl::PointCloud<pcl::PointXYZ>);

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Camera A to camera B (Aruco) ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  transform_ATOb = transform_B*transform_A.inverse();
  pcl::transformPointCloud (*sensorA.cloudCrPtr_, *sensorA.cloud1CrPtr_, transform_ATOb);
  pcl::transformPointCloud (*sensorA.cloudPtr_, *sensorA.cloud1Ptr_, transform_ATOb);
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Camera A to camera B (ROI ICP) ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  bool ICP1_converged;
  double fitnessScore2=1000, fitnessChange=1000;
  float maxCorrDist = ICP_MAX_CORR_DIST;
  Eigen::Matrix4f transform_AtoB_ICP = transform_ATOb; // initial ArUco transformation
  fitnessScore = 1000;


//  pcl::transformPointCloud (*sensorA.cloudCrPtr_, *tmpCloud, transform_AtoB_ICP);
  *tmpCloud = *sensorA.cloud1CrPtr_;

//  for(int i=0 ; i<10; i++)
  int round = 1;
  int iterations = ICP_ITERATIONS;
  while(fitnessChange>ICP_CONVERGE && round<=ICP_MAX_ROUNDS) // fitness change used as converge criteria
  {
//    wp3::ICP_allign(tmpCloud,sensorB.cloudCrPtr_,transform_ICP, ICP1_converged, 0.8-0.1*i, fitnessScore1);
//    wp3::ICP_allign(tmpCloud,sensorB.cloudCrPtr_,transform_ICP, ICP1_converged, 0.8/(2^round), fitnessScore1);
    wp3::ICP_allign(tmpCloud,sensorB.cloudCrPtr_,transform_ICP, ICP1_converged, maxCorrDist/round, iterations*round, fitnessScore);
    if (ICP1_converged)
    {
      ROS_INFO_STREAM("Iterative ICP cycle: " << round << ", MaxDistance: " << maxCorrDist/round  <<"\tICP converged with fitness score: " <<  fitnessScore);
  //    Eigen::Matrix4f transform_AtoB_ICP = transform_ICP.matrix()*transform_B*transform_A.inverse();
      transform_AtoB_ICP = transform_ICP.matrix()*transform_AtoB_ICP;
      pcl::transformPointCloud (*tmpCloud, *tmpCloud2, transform_ICP.matrix());
      *tmpCloud = *tmpCloud2;

//      fitnessChange = std::abs(fitnessScore1-fitnessScore2)/fitnessScore1;
      fitnessChange = std::abs(fitnessScore-fitnessScore2)/fitnessScore; // relative change
      fitnessScore2 = fitnessScore;
      round++;
//      iterations = iterations*2;
    }
    else
    {
      ROS_ERROR_STREAM("ICP did not converge!");
    }



#ifdef VIEW_ICP // ICP Viewer ----------------------
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vector_icp;
    std::map<std::string, Eigen::Matrix4f> transMap;
    cloud_vector_icp.clear();
    cloud_vector_icp.push_back(tmpCloud);
    cloud_vector_icp.push_back(sensorB.cloudCrPtr_);
    transMap.clear();
    transMap.insert (std::pair<std::string, Eigen::Matrix4f> ("", transform_ICP.matrix() ));
    viewer.runSingle(cloud_vector_icp, transMap);

//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_curr(tmpCloud,255,0,0);
//    viewerICP.removeAllPointClouds();
//    viewerICP.removeAllShapes();
//    viewerICP.spinOnce(); // spin to clear
//    viewerICP.addPointCloud<pcl::PointXYZ> (tmpCloud, "cloudA");
//    viewerICP.addPointCloud<pcl::PointXYZ> (sensorB.cloudCrPtr_, "cloudB");
//    viewerICP.setPointCloudRenderingProperties(color_curr,)
//    viewerICP.spinOnce(); // spin to view
#endif
  }
//  std::cout << "ICP complete" << std::endl;

  sensorA.cloud2CrPtr_ = tmpCloud;
//  pcl::transformPointCloud (*sensorA.cloudCrPtr_, *sensorA.cloud2CrPtr_, transform_AtoB_ICP);
  pcl::transformPointCloud (*sensorA.cloudPtr_, *sensorA.cloud2Ptr_, transform_AtoB_ICP);
//  trans_AtoB = transform_reference_global*transform_AtoB_ICP.inverse(); // value to be written
  trans_AtoB = transform_AtoB_ICP; // value to be written
  //		std::cout << "world_to_reference: "<< transform_reference_global << std::endl;
  //		std::cout << "transform_AtoB_ICP: "<< transform_AtoB_ICP << std::endl;
  //		std::cout << "world_to b: "<< world_to_B << std::endl;
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Camera A to Camera B (Final Fine ICP) ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ TEMP TULL
//  bool ICP2_converged;
//  double fitnessScore2;
//  wp3::ICP_allign(sensorA.cloud2Ptr_,sensorB.cloudPtr_,transform_ICP2, ICP2_converged,0.3, fitnessScore2);
//  fitnessScore_to_print = fitnessScore2;
//  Eigen::Matrix4f transform_AtoB_ICP2;
//  if (ICP2_converged)
//  {
//    transform_AtoB_ICP2 = transform_ICP2.matrix()*transform_ICP.matrix()*transform_B*transform_A.inverse();
//    std::cout << "Transformation ICP2: "<< transform_AtoB_ICP2 << std::endl;
//    pcl::transformPointCloud (*sensorA.cloudCrPtr_, *sensorA.cloud3CrPtr_, transform_AtoB_ICP2); // TULL
//    pcl::transformPointCloud (*sensorA.cloudPtr_, *sensorA.cloud3Ptr_, transform_AtoB_ICP2);

//  }
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}


//tf::Quaternion getAverageQuaternion(
Eigen::Quaternionf getAverageQuaternion(
    const std::vector<Eigen::Quaternionf>& quaternions,
    const std::vector<float>& weights)
{
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, quaternions.size());
  Eigen::Vector3d vec;
  for (size_t i = 0; i < quaternions.size(); ++i)
  {
    // Weigh the quaternions according to their associated weight
//    tf::Quaternion quat = quaternions[i] * weights[i];
    Eigen::Quaternionf quat = quaternions[i]; //weights[i];
    // Append the weighted Quaternion to a matrix Q.
//    Q(0,i) = quat.x();
//    Q(1,i) = quat.y();
//    Q(2,i) = quat.z();
//    Q(3,i) = quat.w();
    Q(0,i) = quat.x() * weights[i];
    Q(1,i) = quat.y() * weights[i];
    Q(2,i) = quat.z() * weights[i];
    Q(3,i) = quat.w() * weights[i];
  }

  // Creat a solver for finding the eigenvectors and eigenvalues
  Eigen::EigenSolver<Eigen::MatrixXd> es(Q * Q.transpose());

  // Find index of maximum (real) Eigenvalue.
  auto eigenvalues = es.eigenvalues();
  size_t max_idx = 0;
  double max_value = eigenvalues[max_idx].real();
  for (size_t i = 1; i < 4; ++i)
  {
    double real = eigenvalues[i].real();
    if (real > max_value)
    {
      max_value = real;
      max_idx = i;
    }
  }

  // Get corresponding Eigenvector, normalize it and return it as the average quat
  auto eigenvector = es.eigenvectors().col(max_idx).normalized();

//  tf::Quaternion mean_orientation(
//    eigenvector[0].real(),
//    eigenvector[1].real(),
//    eigenvector[2].real(),
//    eigenvector[3].real()
//  );
  Eigen::Quaternionf mean_orientation(
      eigenvector[3].real(), // w
      eigenvector[0].real(), // x
      eigenvector[1].real(), // y
      eigenvector[2].real()  // z
      );

  return mean_orientation;
}


} // end namespace wp3
