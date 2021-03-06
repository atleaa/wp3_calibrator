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



#include "wp3_calibrator/arucoprocessor.h"
#include "wp3_calibrator/defines.h"

#include <thread>



namespace wp3 {

// Constructor
arucoProcessor::arucoProcessor() :
  croppedCloud_(new pcl::PointCloud<pcl::PointXYZ>)
{
  clearAll();
}

// Destrucor
arucoProcessor::~arucoProcessor()
{
  // boost::shared_ptr are autmatically removed when leaving scope, but can be cleared using ptr.reset();
}


void arucoProcessor::clearAll()
{
  acc_ = 0;
  croppedCloud_->clear();
  transformMap_.clear();
  transCamToArucoVec_.clear();
}

void arucoProcessor::pointcloudFromDepthImage (cv::Mat& depth_image,
                                               Eigen::Matrix3d& depth_intrinsics,
                                               pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud,
                                               bool crop = false,
                                               std::vector<int> c_ext = {},
                                               cv::Mat depth_aruco_dummy = {})
{
  // For point clouds XYZ
  double depth_focal_inverted_x = 1/depth_intrinsics(0,0);  // 1/fx
  double depth_focal_inverted_y = 1/depth_intrinsics(1,1);  // 1/fy

  pcl::PointXYZ new_point;


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_xyz_org(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<int> indices1;


  if (!crop)
  {
    cloud_source_xyz_org->points.resize(depth_image.cols*depth_image.rows, new_point);
    cloud_source_xyz_org->width = depth_image.cols;
    cloud_source_xyz_org->height = depth_image.rows;
    cloud_source_xyz_org->is_dense = false;

    for (int i=0;i<depth_image.rows;i++)
    {
      for (int j=0;j<depth_image.cols;j++)
      {
        float depth_value = depth_image.at<float>(i,j);

        if (depth_value > 0 && depth_value < 20)
        {
          // Find 3D position with respect to depth frame:
          new_point.z = depth_value;
          new_point.x = (j - depth_intrinsics(0,2)) * new_point.z * depth_focal_inverted_x;
          new_point.y = (i - depth_intrinsics(1,2)) * new_point.z * depth_focal_inverted_y;
          cloud_source_xyz_org->at(j,i) = new_point;
        }

        else
        {
          new_point.z = std::numeric_limits<float>::quiet_NaN();
          new_point.x = std::numeric_limits<float>::quiet_NaN();
          new_point.y = std::numeric_limits<float>::quiet_NaN();
          cloud_source_xyz_org->at(j,i) = new_point;
        }
      }
    }
  }
  else
  {
    //	  aruco_cornerpoints = {botx, boty, topx, topy};
    cloud_source_xyz_org->points.resize((depth_aruco_dummy.cols)*(depth_aruco_dummy.rows), new_point);
    cloud_source_xyz_org->width = depth_aruco_dummy.cols;
    cloud_source_xyz_org->height = depth_aruco_dummy.rows;
    cloud_source_xyz_org->is_dense = false;
    int bot_row = c_ext[0];
    int top_row = c_ext[2];
    int bot_col = c_ext[1];
    int top_col = c_ext[3];

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
          cloud_source_xyz_org->at(m-bot_row, k-bot_col) = new_point;
        }

        else
        {
          new_point.z = std::numeric_limits<float>::quiet_NaN();
          new_point.x = std::numeric_limits<float>::quiet_NaN();
          new_point.y = std::numeric_limits<float>::quiet_NaN();
          cloud_source_xyz_org->at(m-bot_row, k-bot_col) = new_point;
        }
      }
    }
  }
  //  std::cout << "nan      Cloud: " << cloud_source_xyz_org->size() << std::endl;
  //  pcl::removeNaNFromPointCloud(*cloud_source_xyz_org,*output_cloud, indices1);
  output_cloud = cloud_source_xyz_org;
  //  std::cout << "filtered Cloud: " << output_cloud->size() << std::endl;
}

std::vector<int> arucoProcessor::getMarkerIdsMean() const
{
  return markerIdsMean_;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr arucoProcessor::getCroppedCloud() const
{
  return croppedCloud_;
}


void arucoProcessor::viewImages(wp3::Sensor& node)
{
  std::string imageName, maskName, imageCroppedName, imageCropDepthName;
  // show full color image
  imageName = node.name_ + " Input";
  //cv::aruco::drawDetectedMarkers(inputImage,rejectedCandidatesAll, cv::noArray(), cv::Scalar(100, 0, 255) );
  //cv::aruco::drawDetectedMarkers(inputImage,markerCornersAll, cv::noArray(), cv::Scalar(200, 255, 200) );
  cv::aruco::drawDetectedMarkers(inputImage_,markerCornersMean_, markerIdsMean_);
  for(size_t i = 0; i < rotVecs_.size(); i++)
  {
    cv::aruco::drawAxis(inputImage_, intrinsicMat_, distortionMat_, rotVecs_[i], transVecs_[i], 0.3);
  }
  cv::imshow(imageName,inputImage_);
  cv::waitKey(30);

  cv::Mat depthMatCropped(inputDepth_.rows, inputDepth_.cols,CV_32F);
  for (size_t i = 0; i < markerIdsMean_.size(); i++)
  {
    // show cropped color image
    imageCroppedName = node.name_ + " Id:" + std::to_string(markerIdsMean_[i]) + " cropped";
    cv::Mat croppedImage(inputImage_.rows, inputImage_.cols,CV_8UC3);
    croppedImage.setTo(cv::Scalar(0,0,0)); // initialize color
    inputImage_.copyTo(croppedImage, maskVec_[i]); // Cut out ROI and store it in imageDest
    cv::imshow(imageCroppedName,croppedImage);
    cv::waitKey(30);

    // Show cropped depth image----------
#ifdef SHOWDEPTH
    imageCropDepthName = node.name_ + " Id:" + std::to_string(markerIdsMean_[i]) + " cropped depth";
    cv::Mat depthMatCroppedView;
    depthMatCropped.setTo(cv::Scalar(0) ); // initialize color
    inputDepth_.copyTo(depthMatCropped, maskVec_[i]);
    depthMatCropped.convertTo(depthMatCroppedView,CV_8UC1,33,0);
    cv::imshow(imageCropDepthName, depthMatCroppedView);
    cv::waitKey(30);
#endif
    // Show current mask
    //        std::string maskName = node.name_ + " Id:" + std::to_string(markerIdsMean_[i]) + " mask";
    //        cv::imshow(maskName,maskVec_[i]);
    //        cv::waitKey(30);

  }
}

void arucoProcessor::getMedianCornerPoints(VecVec2fPoint markerCorners,
                                           std::vector<cv::Mat> &inputDepthVec,
                                           Eigen::Matrix3d intrinsicMatrix,
                                           VecVec3fPoint &markerCornerPoints)
{
  cv::Point3f point_single;
  cv::Point3f point_med;

  double depth_focal_inverted_x = 1/intrinsicMatrix(0,0);  // 1/fx
  double depth_focal_inverted_y = 1/intrinsicMatrix(1,1);  // 1/fy

  //  markerCornerPoints.resize(markerCornersMean_.size());
  //  typedef std::vector<std::vector<cv::Point3f>> VecVec3f;
  markerCornerPoints.resize(markerCorners.size(), std::vector<cv::Point3f>(4));

  for(size_t i = 0; i < markerCorners.size(); i++) // i markers
  {
    for(int j = 0; j<4; j++)                        // j corners
    {
      int count = 0;
      // use nearest pixel. (could be replaced by a weighted average of 4 pixels)
      int x = (int)(markerCorners[i][j].x+0.5);
      int y = (int)(markerCorners[i][j].y+0.5);

      std::vector<float> x_vec, y_vec, z_vec;


      for(size_t k=0; k<inputDepthVec.size(); k++)        // k images
      {
        float depth_value = inputDepthVec[k].at<float>(y,x);

        if(depth_value > 0 && depth_value < 20)
        {
          point_single.x = (x - intrinsicMatrix(0,2)) * depth_value * depth_focal_inverted_x;
          point_single.y = (y - intrinsicMatrix(1,2)) * depth_value * depth_focal_inverted_y;
          point_single.z = depth_value;

          x_vec.push_back(point_single.x);
          y_vec.push_back(point_single.y);
          z_vec.push_back(point_single.z);
        }
      } // k images

      point_med.x = wp3::calcMedian(x_vec);
      point_med.y = wp3::calcMedian(y_vec);
      point_med.z = wp3::calcMedian(z_vec);

      markerCornerPoints[i][j] = point_med;

      //      std:: cout << node.name_ << ", corner " << j
      //                 << "\t marker: " << markerIdsMean_[i]
      //                 << "\t x: " << x <<", y: " << y
      //                 << "\t point: " << point_cv
      //                 << "\t point_median: " << points3D_med  << std::endl;

    } // j corners
  }
}

static double _dummy_double;
int arucoProcessor::findBestPose(std::vector<cv::Vec3d> &rotVecs,
                                 std::vector<cv::Vec3d> &transVecs,
                                 Vec3fPoint corners,
                                 double &score=_dummy_double)
{
  double arucoDim = ARUCODIMENSION; // in meters
  static tf::TransformBroadcaster tfPub_local;
  cv::Mat R_mat;
  Eigen::Matrix3d R;  // rotation
  Eigen::Vector3d T;  // translation
  Eigen::Matrix4d tf_OtoM; // Transformation Origin to Marker
  std::vector<Eigen::Matrix4d> tf_MtoC(4,Eigen::Matrix4d::Identity()); // Transformations Marker to Corners
  std::vector<Eigen::Matrix4d> tf_OtoC(4); // Transformations Origin to Corners

  // aruco corners are listed clockwise from top left corner
  tf_MtoC[0].block<3,1>(0,3) << -arucoDim/2,  arucoDim/2, 0.0;
  tf_MtoC[1].block<3,1>(0,3) <<  arucoDim/2,  arucoDim/2, 0.0;
  tf_MtoC[2].block<3,1>(0,3) <<  arucoDim/2, -arucoDim/2, 0.0;
  tf_MtoC[3].block<3,1>(0,3) << -arucoDim/2, -arucoDim/2, 0.0;

  std::vector<Eigen::Vector3d> Pd(4), Pt(4); // 3D points
  double distance;
  int index=0;
  score=9999;

  // get coordinates of measured depth
  std::vector<Eigen::Vector3f> Pd_float_c(4);
  for(int i=0; i<4; i++)
  {
    Pd_float_c[i] << corners[i].x, corners[i].y, corners[i].z;
    Pd[i] = Pd_float_c[i].cast<double>(); //convert
  }

  for(int i=0 ; i<rotVecs.size(); i++)
  {
    R = Eigen::Matrix3d::Zero();
    T = Eigen::Vector3d::Zero();
    distance=0;

    // convert cv to eigen
    cv::Rodrigues(rotVecs[i], R_mat);
    cv::cv2eigen(R_mat, R);
    cv::cv2eigen(transVecs[i],T);

    // Create Eigen Transformation Matrix Origin to Marker
    tf_OtoM.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
    tf_OtoM.block<3,3>(0,0) = R;
    tf_OtoM.block<3,1>(0,3) = T;

    for(int i=0; i<4; i++)
    {
      // Calculate coordinates of transformed points
      tf_OtoC[i] = tf_OtoM*tf_MtoC[i];
      Pt[i] = tf_OtoC[i].block(0,3,3,1);

      // calculate sum of squared euclidean distance
      distance += (Pt[i]-Pd[i]).squaredNorm();
    }

    // save the minimum sum of distance^2. as score
    if(distance<score)
    {
      score = distance;
      index = i;
    }

    // optionally publish tf to test
    //    Eigen::Affine3d transform_affine(tf_OtoM);
    //    tf::Transform transform_tf;
    //    tf::transformEigenToTF(transform_affine, transform_tf);
    //    tfPub_local.sendTransform(tf::StampedTransform(transform_tf, ros::Time::now(), "jetson6_ir_optical_frame", std::to_string(i) ) );

    //      Eigen::Affine3d transform_affine2(tf_cam_c1);
    //      tf::transformEigenToTF(transform_affine2, transform_tf);
    //      tfPub_local.sendTransform(tf::StampedTransform(transform_tf, ros::Time::now(), "jetson6_ir_optical_frame", "c1" ) );

  }
  return index;
}

void arucoProcessor::processImages(wp3::Sensor & node,
                                   MarkerMapType &transform4x4)
{
  // Clear memory
  clearAll();

  inputImage_ = node.getImageMat(); // get last image
  inputDepth_ = node.getDepthMat(); // get last image
  //  std::vector<cv::Mat> inputDepthVec = node.getDepthMatVec(); // get all recorded images (removed to save memory)


  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Cam. coeff. ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||
  std::vector<double> distortionVec = node.getDistCoeffs();
  distortionMat_ =   cv::Mat (distortionVec);

  Eigen::Matrix3d intrinsicMatrix;
  intrinsicMatrix = node.getIntrinsics_matrix();
  cv::eigen2cv(intrinsicMatrix, intrinsicMat_);

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ aruco param. ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||
  double arucoSquareDimension = ARUCODIMENSION; // in meters
  std::vector<int> markerIds, markerIdsAll; //, markerIdsMean; // markerIds.size() = number of Ids found
  markerIdsMean_.clear();

  VecVec2fPoint markerCorners, markerCornersAll, markerCornersPadded, rejectedCandidates, rejectedCandidatesAll;
  std::vector<std::vector<int>> markerIdsVec; // markerIds.size() = number of Ids found
  std::vector<VecVec2fPoint> rejectedCandidatesVec;

  cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
  //      params->doCornerRefinement = true;
  params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
  params->cornerRefinementWinSize = 5;
  params->cornerRefinementMaxIterations = 2000;
  params->adaptiveThreshConstant = true;
  params->cornerRefinementMinAccuracy = 0.001f;
  cv::Ptr<cv::aruco::Dictionary> markerDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||


  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Marker Detection ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||
  std::multimap<int, Vec2fPoint> markerCornersMap;
  //  markerCornersVec.clear();
  markerIdsVec.clear();
  for(size_t i=0; i<node.imageMatVec_.size(); i++)
  {
    // detect markers
    cv::aruco::detectMarkers(node.imageMatVec_[i],markerDictionary,markerCorners,markerIds, params, rejectedCandidates);

    // insert vectors
    markerIdsVec.push_back(markerIds);          // Vector of Id vectors
    rejectedCandidatesVec.push_back(rejectedCandidates);          // Vector of rejected candidates vectors

    // insert Maps
    std::stringstream markerStream;
    for (size_t i=0 ; i< markerIds.size(); i++)
    {
      markerStream << markerIds[i] << ' ';
      markerCornersMap.insert(std::pair<int, Vec2fPoint>(markerIds[i], markerCorners[i]) );
    }
    // show detected markers for debugging
    //    ROS_DEBUG_STREAM(node.name_ << " ID's detected:\t" << markerStream.str());

    // merge all Ids and Corners
    markerIdsAll.insert( markerIdsAll.end(), markerIds.begin(), markerIds.end() );
    markerCornersAll.insert( markerCornersAll.end(), markerCorners.begin(), markerCorners.end() );
    rejectedCandidatesAll.insert( rejectedCandidatesAll.end(), rejectedCandidates.begin(), rejectedCandidates.end() );
  }
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||


  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Average Corner Pixel Calc ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||
  typedef std::multimap<int, Vec2fPoint>::iterator MMAPIterator;
  // Returns a pair representing the range of elements with key equal to ('value')
  std::pair<MMAPIterator, MMAPIterator> result_1 = markerCornersMap.equal_range(1);
  std::pair<MMAPIterator, MMAPIterator> result_13 = markerCornersMap.equal_range(13);
  std::pair<MMAPIterator, MMAPIterator> result_40 = markerCornersMap.equal_range(40);

  // Iterate over the range
  markerCornersMean_.clear();
  Vec2fPoint cornersCurr(4), cornersMean(4);

  // Feedback on detection rate.
  int count1 = std::distance(result_1.first, result_1.second);
  ROS_DEBUG_STREAM(node.name_ << " detection rate Id 1:\t" << (float)count1*100/(float)IMAGES_C << "%");
  if(count1*100/IMAGES_C<10)
    ROS_ERROR_STREAM(node.name_ << " Id 1 detection rate is " << (float)count1*100/(float)IMAGES_C
                     << "% with only " << count1 << " detections in " << IMAGES_C << " images. "
                                                                                     "Consider moving marker or recalibrating the sensor intrinsics.");

  int count13 = std::distance(result_13.first, result_13.second);
  ROS_DEBUG_STREAM(node.name_ << " detection rate Id 13:\t" << (float)count13*100/(float)IMAGES_C << "%");
  if(count13*100/IMAGES_C<10)
    ROS_ERROR_STREAM(node.name_ << " Id 13 detection rate is " << (float)count13*100/(float)IMAGES_C
                     << "% with only " << count13 << " detections in " << IMAGES_C << " images. "
                                                                                      "Consider moving marker or recalibrating the sensor intrinsics.");

  int count40 = std::distance(result_40.first, result_40.second);
  ROS_DEBUG_STREAM(node.name_ << " detection rate Id 40:\t" << (float)count40*100/(float)IMAGES_C << "%");
  if(count40*100/IMAGES_C<10)
    ROS_ERROR_STREAM(node.name_ << " Id 40 detection rate is " << (float)count40*100/(float)IMAGES_C
                     << "% with only " << count40 << " detections in " << IMAGES_C << " images. "
                                                                                      "Consider moving marker or recalibrating the sensor intrinsics.");


  // Find average corners of marker 1
  //  cornersMean.clear();
  if(count1>0)
  {
    for (MMAPIterator it = result_1.first; it != result_1.second; it++)
    {
      cornersCurr = it->second;
      for (int j = 0; j<4; j++)
      {
        cornersMean[j].x += cornersCurr[j].x/count1;
        cornersMean[j].y += cornersCurr[j].y/count1;
      }
    }
    ROS_DEBUG_STREAM(node.name_ << " Id: 1 averaged corners:" << std::endl << cornersMean);
    markerIdsMean_.push_back(1);
    markerCornersMean_.push_back(cornersMean);
  }

  // Find average corners of marker 13
  if(count13>0)
  {
    cornersMean = Vec2fPoint(4);
    for (MMAPIterator it = result_13.first; it != result_13.second; it++)
    {
      cornersCurr = it->second;
      for (int j = 0; j<4; j++)
      {
        cornersMean[j].x += cornersCurr[j].x/count13;
        cornersMean[j].y += cornersCurr[j].y/count13;
      }
    }
    ROS_DEBUG_STREAM(node.name_ << " Id: 13 averaged corners:" << std::endl << cornersMean);
    markerIdsMean_.push_back(13);
    markerCornersMean_.push_back(cornersMean);
  }


  // Find average corners of marker 40
  if(count40>0)
  {
    cornersMean = Vec2fPoint(4);
    for (MMAPIterator it = result_40.first; it != result_40.second; it++)
    {
      cornersCurr = it->second;
      for (int j = 0; j<4; j++)
      {
        cornersMean[j].x += cornersCurr[j].x/count40;
        cornersMean[j].y += cornersCurr[j].y/count40;
      }
    }
    ROS_DEBUG_STREAM(node.name_ << " Id: 40 averaged corners:" << std::endl << cornersMean);
    markerIdsMean_.push_back(40);
    markerCornersMean_.push_back(cornersMean);
  }
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||


  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Create cropping mask~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||
  // init binary mask
  cv::Mat mask(inputDepth_.rows, inputDepth_.cols,CV_8UC1);
  maskVec_.resize(markerIdsMean_.size());

  for (size_t i = 0; i < markerIdsMean_.size(); i++)
  {
    // init color
    mask.setTo(cv::Scalar(0) );

    // Calculate corners of padded aruco
    markerCornersPadded = markerCornersMean_;

    // extrapolate the padded corner position using distance between the diagonal corners
    for(int j=0 ; j<4 ; j++)
    {
      cv::Point2f corner;
      corner.x = markerCornersMean_[i][j].x + ( markerCornersMean_[i][j].x - markerCornersMean_[i][(j+2)%4].x ) * PADDING*0.5;
      corner.y = markerCornersMean_[i][j].y + ( markerCornersMean_[i][j].y - markerCornersMean_[i][(j+2)%4].y ) * PADDING*0.5;
      markerCornersPadded[i][j] = corner;
    }

    // Create Polygon from vertices
    std::vector<cv::Point> ROI_Poly;
    cv::approxPolyDP(markerCornersMean_[i], ROI_Poly, 1.0, true);
    std::vector<cv::Point> ROI_PolyPadded;
    cv::approxPolyDP(markerCornersPadded[i], ROI_PolyPadded, 1.0, true);

    // Fill polygon white
    cv::fillConvexPoly(mask, &ROI_PolyPadded[0], ROI_PolyPadded.size(), 255, 8, 0);

    // Save mask of markerId i
    maskVec_[i] = mask.clone();
  }
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||


  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Crop depth images ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||
  cv::Mat depthMatCropped(inputDepth_.rows, inputDepth_.cols,CV_32F);
  pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_crop (new pcl::PointCloud<pcl::PointXYZ>);

  //  for(size_t j=0; j<node.depthMatVec_.size(); j++)
  for(size_t j=0; j<CLOUDS; j++)
  {
    for (size_t i = 0; i < markerIdsMean_.size(); i++)
    {
      depthMatCropped.setTo(cv::Scalar(0)); // initialize depth to 0
      node.depthMatVec_[j].copyTo(depthMatCropped, maskVec_[i]);

      pointcloudFromDepthImage(depthMatCropped, intrinsicMatrix, src_cloud_crop);
      //    pointcloudFromDepthImage(depthMatCropped, intrinsicMatrix, src_cloud_crop, true, rectCrop, depthMatCropped);

      // add cropped source to accumulated marker cloud
      //      if (markerIdsMean_[i] == 1)  *src_cloud_crop1_ += *src_cloud_crop;
      //      if (markerIdsMean_[i] == 13) *src_cloud_crop2_ += *src_cloud_crop;
      //      if (markerIdsMean_[i] == 40) *src_cloud_crop3_ += *src_cloud_crop;

      // add cropped source to combined cloud
      *node.cloudCrPtr_ += *src_cloud_crop; // save data directly in sensor class
      //      *croppedCloud_ += *src_cloud_crop;
    } // end for markerIdsMean.
  } //end for inputDepthVec
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||


  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Transform ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||
  Eigen::Matrix4f tmatTemp = Eigen::Matrix4f::Identity();
  VecVec3fPoint markerCornerPoints;
  transform4x4.clear(); // clear transformations
  static tf::TransformBroadcaster tfPub_local;

  VecVec2fPoint cornersCurrVec;
  int numMarkers = markerIdsMean_.size();
  //std::vector<std::vector<cv::Vec3d>> rotVecVec(numMarkers), transVecVec(numMarkers);
  std::vector<cv::Vec3d> rotVecsTmp, transVecsTmp;
  int index;
  double score=9999;

  // corner points from depth image
  getMedianCornerPoints(markerCornersMean_, node.depthMatVec_, intrinsicMatrix, markerCornerPoints);

  rotVecs_.clear();
  transVecs_.clear();

  for(int m=0 ; m<numMarkers ; m++)
  {
    cornersCurrVec.clear();

    // Get current marker corners.  (TODO, fix structure..)
    if(markerIdsMean_[m]==1)
    {
      for (MMAPIterator it = result_1.first; it != result_1.second; it++)
        cornersCurrVec.push_back(it->second);
    }
    if(markerIdsMean_[m]==13)
    {
      for (MMAPIterator it = result_13.first; it != result_13.second; it++)
        cornersCurrVec.push_back(it->second);
    }
    if(markerIdsMean_[m]==40)
    {
      for (MMAPIterator it = result_40.first; it != result_40.second; it++)
        cornersCurrVec.push_back(it->second);
    }

    // get all pose estimates of single marker
    cv::aruco::estimatePoseSingleMarkers(cornersCurrVec, arucoSquareDimension, intrinsicMat_,
                                         distortionMat_, rotVecsTmp, transVecsTmp);

    //  cv::Vec3f pointDepth_cv = markerCornerPoints[1][0]; // change vecvec?

    // Save the transformation closest to depth measurements
    index = findBestPose(rotVecsTmp, transVecsTmp, markerCornerPoints[m], score);
    ROS_DEBUG_STREAM(node.name_ << "." << markerIdsMean_[m] << "\tscore: " << score);
    if(score>1.0)
    {
      ROS_ERROR_STREAM(node.name_ << "." << markerIdsMean_[m] << "\tscore: " << score
                       << std::endl << "Score > 1 indicates poor marker transformation alignment."
                                       "Calibration refinement may not be possible. Recapture recommended.");
    }
    rotVecs_.push_back(rotVecsTmp[index]);
    transVecs_.push_back(transVecsTmp[index]);


    // convert
    Eigen::Matrix3d R;  // rotation
    Eigen::Vector3d T;  // translation
    cv::Mat R_mat;
    R = Eigen::Matrix3d::Zero();
    T = Eigen::Vector3d::Zero();
    // convert cv to eigen
    cv::Rodrigues(rotVecsTmp[index], R_mat);
    cv::cv2eigen(R_mat, R);
    cv::cv2eigen(transVecsTmp[index],T);
    Eigen::Matrix4d tf_OtoM; // Transformation
    // Create Eigen Transformation Matrix
    tf_OtoM.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
    tf_OtoM.block<3,3>(0,0) = R;
    tf_OtoM.block<3,1>(0,3) = T;

    //save
    transCamToArucoVec_.push_back(tf_OtoM);

    //    // publish
    //    Eigen::Affine3d transform_affine(tf_OtoM);
    //    tf::Transform transform_tf;
    //    tf::transformEigenToTF(transform_affine, transform_tf);
    //    std::string tfName = node.name_ + "." + std::to_string(markerIdsMean_[m]);
    //    tfPub_local.sendTransform(tf::StampedTransform(transform_tf, ros::Time::now(), node.getTfTopic(), tfName ) );
  }

  for(size_t i = 0; i < markerIdsMean_.size(); i++)
  {
    covertRodriguesToTransMatrix(rotVecs_[i],transVecs_[i], tmatTemp);
    transform4x4.insert (std::pair<int, Eigen::Matrix4f> (markerIdsMean_[i]+100*acc_,tmatTemp ));
    transformMap_.insert (std::pair<int, Eigen::Matrix4f> (markerIdsMean_[i]+100*acc_,tmatTemp ));
  }
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||

  acc_++;
}


void arucoProcessor::covertRodriguesToTransMatrix(cv::Vec3d rotationVectors, cv::Vec3d translationVectors, Eigen::Matrix4f& tMat)
{
  // Create the transformation matrix
  cv::Mat rotation3x3 = cv::Mat::eye(3, 3, CV_64F);
  cv::Rodrigues(rotationVectors, rotation3x3);

  if(not wp3::isRotationMatrix(rotation3x3))
    ROS_ERROR_STREAM("Invalid aruco pose detected" << std::endl
                     << "Rot:\t" << rotationVectors);


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

//Eigen::Matrix4f arucoProcessor::getAverageTransformation()
void arucoProcessor::getAverageTransformation(Eigen::Matrix4f& transMat_avg, MarkerMapType &transMapUsed)
{
  int validCount=0;
  //  Eigen::Vector3f weight = {1, 1, 1};
  std::vector<float> weight = {1, 1, 1};
  std::vector<float> weights;
  Eigen::Vector3f pos={0,0,0}, pos_avg;
  std::vector<Eigen::Quaternionf> quaternions;
  quaternions.clear();
  transMapUsed.clear();

  int i=0;
  // loop is obsolete. As only best aruco pose is used, there will not be multiple pose instances
  //  for(int i=0 ; i<acc_ ; i++) // loop all images
  //  {
  // if all markers are found
  if(transformMap_.count(1+100*i)>0 && transformMap_.count(13+100*i)>0 && transformMap_.count(40+100*i)>0)
  {
    //      std::cout << "found all IDs for acc= " << i << std:: endl;

    transMapUsed.insert (std::pair<int, Eigen::Matrix4f> (1+100*i, transformMap_.at(1+100*i) ));
    transMapUsed.insert (std::pair<int, Eigen::Matrix4f> (13+100*i, transformMap_.at(13+100*i) ));
    transMapUsed.insert (std::pair<int, Eigen::Matrix4f> (40+100*i, transformMap_.at(40+100*i) ));

    // que rotations
    Eigen::Quaternionf q1(transformMap_.at(1+100*i).block<3,3>(0,0));
    Eigen::Quaternionf q2(transformMap_.at(13+100*i).block<3,3>(0,0));
    Eigen::Quaternionf q3(transformMap_.at(40+100*i).block<3,3>(0,0));
    quaternions.push_back(q1);
    quaternions.push_back(q2);
    quaternions.push_back(q3);

    // que positions
    pos = pos + weight[0]*transformMap_.at(1+100*i).block<3, 1>(0, 3);
    pos = pos + weight[1]*transformMap_.at(13+100*i).block<3, 1>(0, 3);
    pos = pos + weight[2]*transformMap_.at(40+100*i).block<3, 1>(0, 3);

    validCount++;
    weights.insert(weights.end(), std::begin(weight), std::end(weight));
  }
  //  }

  // calculate average quaternion
  Eigen::Quaternionf q_avg(wp3::getAverageQuaternion(quaternions, weights));
  Eigen::Matrix3f R_avg = q_avg.toRotationMatrix();


  // calculate average position
  float weight_sum = 0.0;
  for (auto& n : weights)
    weight_sum += n;
  pos_avg = (1.0 / weight_sum) * pos;

  // insert transform
  transMat_avg = Eigen::Matrix4f::Identity();
  transMat_avg.block<3, 3>(0, 0) = R_avg;
  transMat_avg.block<3, 1>(0, 3) = pos_avg;
  //  return transMat_avgA;

}

/* Create an artificial marker in given position
 * Aruco id
 * R rotation euler Z, Y, X
 * T translation X,Y,Z */
void arucoProcessor::simulateMarker(int id, Eigen::Vector3d R, Eigen::Vector3d T)
{
  auto aruco_dict=cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  int pixelsPerMeter=200;
  cv::Mat markerImg(ARUCODIMENSION*pixelsPerMeter,ARUCODIMENSION*pixelsPerMeter,CV_8UC1);

  cv::aruco::drawMarker(aruco_dict,id,markerImg.rows,markerImg,1);

  // add padding
  int pad = (int) (PADDING*0.5*markerImg.rows);
  cv::copyMakeBorder( markerImg, markerImg, pad, pad, pad, pad, cv::BORDER_CONSTANT, cv::Scalar(255) );

  cv::imshow("Marker "+std::to_string(id),markerImg);
  cv::waitKey(30);

  pcl::PointCloud<pcl::PointXYZ>::Ptr marker_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ new_point;
  marker_cloud->points.resize(markerImg.cols*markerImg.rows, new_point);
  marker_cloud->width = markerImg.cols;
  marker_cloud->height = markerImg.rows;
  marker_cloud->is_dense = true;

  for (int i=0;i<markerImg.rows;i++)
  {
    for (int j=0;j<markerImg.cols;j++)
    {
      //          bool pixel_value = markerImg.at<float>(i,j);
      bool pixel_value = markerImg.at<bool>(i,j);
      if (pixel_value)
      {
        new_point.z = 0.0;
        new_point.x = (float)j/pixelsPerMeter;
        new_point.y = -(float)i/pixelsPerMeter;
        marker_cloud->at(j,i) = new_point;
      }
      else
      {
        new_point.z = std::numeric_limits<float>::quiet_NaN();
        new_point.x = std::numeric_limits<float>::quiet_NaN();
        new_point.y = std::numeric_limits<float>::quiet_NaN();
        marker_cloud->at(j,i) = new_point;
      }
    }
  }

  // center marker at origin
  pcl::transformPointCloud(*marker_cloud, *marker_cloud,
                           Eigen::Vector3f((-markerImg.cols/2+0.5)/pixelsPerMeter,
                                           ((markerImg.rows/2)-0.5)/pixelsPerMeter,0),
                           Eigen::Quaternionf(0.0,0.0,0.0,0.0));


  Eigen::Quaternion<double> q =
      Eigen::AngleAxisd(R[0], Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(R[1], Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(R[2], Eigen::Vector3d::UnitX());
  Eigen::Matrix3d Rmat = q.matrix();

  Eigen::Matrix4d tf_OtoM; // Transformation world origin to marker
  // Create Eigen Transformation Matrix
  tf_OtoM.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
  tf_OtoM.block<3,3>(0,0) = Rmat;
  tf_OtoM.block<3,1>(0,3) = T;

  // transform marker to input pose
  pcl::transformPointCloud(*marker_cloud, *marker_cloud, tf_OtoM.cast<float>());

  transformMap_.insert (std::pair<int, Eigen::Matrix4f> (id, tf_OtoM.cast<float>() ));
  *croppedCloud_ += *marker_cloud;

  // log inserted marker
  markerIdsMean_.push_back(id);
  transCamToArucoVec_.push_back(tf_OtoM);

} // end arucoProcessor::simulateMarker

} // end namespace wp3
