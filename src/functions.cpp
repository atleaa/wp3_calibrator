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

#include "wp3_calibrator/functions.h"
#include "wp3_calibrator/defines.h"
#include "wp3_calibrator/imageconverter.h"


namespace wp3 {

void init_reference(std::string kinect_number)
{
  std::string fs_filename = wp3::package_path + "kinect_ICP1_tMat/kinect" + kinect_number + "/results.yaml";
  std::cout << "Creating global reference in file " << fs_filename << "  ... " << std::flush;

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
//  icp.setUseReciprocalCorrespondences(true);
  icp.setRANSACOutlierRejectionThreshold(maxCorrDist);
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


void refineTransformation(wp3::Sensor & sensor, wp3::Sensor & reference)
{
  Eigen::Matrix4f transform_ATOb = Eigen::Matrix4f::Identity();
  Eigen::Affine3f transform_ICP = Eigen::Affine3f::Identity();
  Eigen::Affine3f transform_ICP2 = Eigen::Affine3f::Identity();
  pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud2 (new pcl::PointCloud<pcl::PointXYZ>);

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Camera A to camera B (ROI ICP) ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  bool ICP1_converged;
  double fitnessScore2=1000, fitnessChange=1000;
  float maxCorrDist = ICP_MAX_CORR_DIST;
  Eigen::Matrix4f transform_AtoB_ICP = Eigen::Matrix4f::Identity(); // initially transformed by ArUco transformation
  double fitnessScore = 1000;

  *tmpCloud = *sensor.cloud1CrPtr_;

//  for(int i=0 ; i<10; i++)
  int round = 0;
  int iterations = ICP_ITERATIONS;
  while(fitnessChange>ICP_CONVERGE && round<ICP_MAX_ROUNDS) // fitness change used as converge criteria
  {
    round++;
    wp3::ICP_allign(tmpCloud,reference.cloud1CrPtr_,transform_ICP, ICP1_converged, maxCorrDist/round, iterations*round, fitnessScore);
    if (ICP1_converged)
    {
      ROS_DEBUG_STREAM(sensor.name_ << "->ref:\tIterative ICP cycle: " << round << ", MaxDistance: " << maxCorrDist/round
                      <<"\tICP converged with fitness score: " <<  fitnessScore);
  //    Eigen::Matrix4f transform_AtoB_ICP = transform_ICP.matrix()*transform_B*transform_A.inverse();
      transform_AtoB_ICP = transform_ICP.matrix()*transform_AtoB_ICP;
      pcl::transformPointCloud (*tmpCloud, *tmpCloud2, transform_ICP.matrix());
      *tmpCloud = *tmpCloud2;

//      fitnessChange = std::abs(fitnessScore1-fitnessScore2)/fitnessScore1;
      fitnessChange = std::abs(fitnessScore-fitnessScore2)/fitnessScore; // relative change
      fitnessScore2 = fitnessScore;
    }
    else
    {
      ROS_ERROR_STREAM(sensor.name_ << "->ref\t ICP did not converge!");
    }
  }

  ROS_INFO_STREAM(sensor.name_ << "->ref:\tICP refinement complete." << std::endl
                  << sensor.name_ << "->ref:\tRefinement iterations:\t"<< round << std::endl
                  << sensor.name_ << "->ref:\tFinal fitness score:\t" << fitnessScore);

  pcl::transformPointCloud (*sensor.cloud1CrPtr_, *sensor.cloud2CrPtr_, transform_AtoB_ICP);
  pcl::transformPointCloud (*sensor.cloud1Ptr_, *sensor.cloud2Ptr_, transform_AtoB_ICP);
  *reference.cloud2Ptr_ = *reference.cloud1Ptr_;
  *reference.cloud2CrPtr_ = *reference.cloud1CrPtr_;
// save transformation in sensor class
  sensor.transArucoToICP_ = transform_AtoB_ICP;


} // end refineTransformation



// based on https://github.com/BobMcFry/averaging_weighted_quaternions
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
