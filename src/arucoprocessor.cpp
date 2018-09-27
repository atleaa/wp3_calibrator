#include "wp3_calibrator/arucoprocessor.h"



namespace wp3 {

//std::string package_path = "/home/sfi/catkin_ws/src/wp3_calibrator/";

// Constructor
arucoProcessor::arucoProcessor() :
  src_cloud_crop1_(new pcl::PointCloud<pcl::PointXYZ>),
  src_cloud_crop2_(new pcl::PointCloud<pcl::PointXYZ>),
  src_cloud_crop3_(new pcl::PointCloud<pcl::PointXYZ>)
{

}


// Deconstrucor
arucoProcessor::~arucoProcessor()
{
 // boost::shared_ptr are autmatically removed when leaving scope, but can be cleared using ptr.reset();
}


void arucoProcessor::detectMarkers(cv::Mat &inputImage,cv::Mat &inputDepth, Eigen::Matrix4f & transform4x4, std::string kinect_number)
{

  std::string camera_name = "/kinect" + kinect_number;
  std::string filename_inputPicture = "Input Picture" + camera_name;
  std::string filename_crop1 = "cropped image 1" + camera_name;
  std::string filename_crop2 = "cropped image 2" + camera_name;
  std::string rgb_intrinsics_dir = package_path + "kinect_calibrations" + camera_name + "/calib_color.yaml";
  std::string filename_crop3 = "cropped image 3" + camera_name;

  std::cout << rgb_intrinsics_dir << std::endl;

  // TULL, use custom parameters!?
  // Default RGB parameters -> will be used in rectified depthMatrix since it is mapped onto RGB
  // Try to upscale IR intrinsics to RGB size--> use them to transform depth map to point cloud
  float const fx_default = 1081.37;
  float const fy_default = 1081.37;
  float const cx_default = 959.5;
  float const cy_default = 539.5;

  Eigen::Matrix3f ir_params;
  ir_params(0,0) = fx_default;
  ir_params(1,1) = fy_default;
  ir_params(0,2) = cx_default;
  ir_params(1,2) = cy_default;

  src_cloud_crop1_->clear();
  src_cloud_crop2_->clear();
  src_cloud_crop3_->clear();

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

  std::cout << "camera matrix aruco : " << cameraMatrix << std::endl << "distortion coeffs: " << dist_coeffs << std::endl;

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

        current_depthMat_A_crop1_ = inputDepth(myROI);

        std::vector<float> aruco_cornerpoints = {botx, boty, topx, topy};
        pointcloudFromDepthImage(inputDepth, ir_params, src_cloud_crop1_, true, aruco_cornerpoints, current_depthMat_A_crop1_);

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
        current_depthMat_A_crop2_ = inputDepth(myROI);
        std::vector<float> aruco_cornerpoints = {botx, boty, topx, topy};
        pointcloudFromDepthImage(inputDepth, ir_params, src_cloud_crop2_, true, aruco_cornerpoints, current_depthMat_A_crop2_);

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

        current_depthMat_A_crop3_ = inputDepth(myROI);

        std::vector<float> aruco_cornerpoints = {botx, boty, topx, topy};
        pointcloudFromDepthImage(inputDepth, ir_params, src_cloud_crop3_, true, aruco_cornerpoints, current_depthMat_A_crop3_);
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

void arucoProcessor::getCroppedCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  *cloud += *src_cloud_crop1_;
  *cloud += *src_cloud_crop2_;
  *cloud += *src_cloud_crop3_;
}

void arucoProcessor::createTransMatrix(cv::Vec3d rotationVectors, cv::Vec3d translationVectors, Eigen::Matrix4f& tMat)
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

} // end namespace wp3
