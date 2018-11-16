#include "wp3_calibrator/arucoprocessor.h"
#include "wp3_calibrator/defines.h"



namespace wp3 {

//std::string package_path = "/home/sfi/catkin_ws/src/wp3_calibrator/";

// Constructor
arucoProcessor::arucoProcessor() :
  src_cloud_crop1_(new pcl::PointCloud<pcl::PointXYZ>),
  src_cloud_crop2_(new pcl::PointCloud<pcl::PointXYZ>),
  src_cloud_crop3_(new pcl::PointCloud<pcl::PointXYZ>),
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
  src_cloud_crop1_->clear();
  src_cloud_crop2_->clear();
  src_cloud_crop3_->clear();
  croppedCloud_->clear();

  transformMap_.clear();
}

void arucoProcessor::max4points(std::vector<cv::Point2f> cornerPoints, float & topx, float & topy, float & botx, float & boty, bool &flag)
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


void arucoProcessor::pointcloudFromDepthImage (cv::Mat& depth_image,
                               Eigen::Matrix3d& depth_intrinsics,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud,
                               bool crop = false,
                               std::vector<int> c_ext = {},
                               cv::Mat depth_aruco_dummy = {})
{
  // For point clouds XYZ
//  float depth_focal_inverted_x = 1/depth_intrinsics(0,0);  // 1/fx
//  float depth_focal_inverted_y = 1/depth_intrinsics(1,1);  // 1/fy
  double depth_focal_inverted_x = 1/depth_intrinsics(0,0);  // 1/fx
  double depth_focal_inverted_y = 1/depth_intrinsics(1,1);  // 1/fy

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

  }
}

void arucoProcessor::detectMarkers(wp3::Sensor & node,
                                   MarkerMapType &transform4x4)
//void arucoProcessor::detectMarkers(boost::ref(wp3::Sensor node),
//                                   MarkerMapType &transform4x4)
{
//  cv::Mat inputImage = node.getImageMat(); // get last image
  inputImage_ = node.getImageMat(); // get last image
  std::vector<cv::Mat> inputImageVec = node.getImageMatVec(); // get all recorded images
//  cv::Mat inputDepth = node.getDepthMat(); // get last image
  inputDepth_ = node.getDepthMat(); // get last image
  std::vector<cv::Mat> inputDepthVec = node.getDepthMatVec(); // get all recorded images




  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Parameters ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||

  float topx, topy, botx, boty;
  bool invalid_points = true;

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Cam. coeff. ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||

  std::vector<double> distortionVec = node.getDistCoeffs();
//  std::cout << "distortionVec" << std::endl;
//  for(auto i : distortionVec)
//    std::cout << i << " " << std::endl;

  //  cv::Mat distortionMat(distortionVec);
  distortionMat_ =   cv::Mat (distortionVec);
//  std::cout << "distortionMat_" << std::endl << distortionMat_ << std::endl;

//  cv::Mat intrinsicMat;
  Eigen::Matrix3d intrinsicMatrix;
  intrinsicMatrix = node.getIntrinsics_matrix();
  cv::eigen2cv(intrinsicMatrix, intrinsicMat_);

  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ aruco param. ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||
  double arucoSquareDimension = ARUCODIMENSION; // in meters
  std::vector<int> markerIds, markerIdsAll; //, markerIdsMean; // markerIds.size() = number of Ids found
  markerIdsMean_.clear();

  VecVec2f markerCorners, markerCornersAll, markerCornersPadded, rejectedCandidates, rejectedCandidatesAll;
//  std::vector<cv::Vec3d> rotVecs, transVecs;

  std::vector<std::vector<int>> markerIdsVec; // markerIds.size() = number of Ids found
  std::vector<VecVec2f> rejectedCandidatesVec;
  std::vector<std::vector<cv::Vec3d>> rotVecsVec, transVecsVec;

  cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
  //      params->doCornerRefinement = true;
  params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
  params->cornerRefinementWinSize = 5;
  params->cornerRefinementMaxIterations = 2000;
  params->adaptiveThreshConstant = true;
  params->cornerRefinementMinAccuracy = 0.001f;

  //---testing parameters
  //params->cornerRefinementMinAccuracy = 0.1f;
  //  params->adaptiveThreshWinSizeStep = 5;
  //  params->adaptiveThreshConstant = 3;
  //  params->errorCorrectionRate = 0.9f;
  //  params->maxErroneousBitsInBorderRate = 0.9f;
  //---

  cv::Ptr<cv::aruco::Dictionary> markerDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_50);	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||



  //  std::vector<int> markerIds; //
  std::multimap<int, Vec2f> markerCornersMap;
  typedef std::multimap<int, Vec2f>::iterator MMAPIterator;
  //  std::vector<std::vector<cv::Point2f>> markerCorners;
  //  std::vector<cv::Vec3d> rotVecs, transVecs;


  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Marker Detection ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||
  //  markerCornersVec.clear();
  markerIdsVec.clear();
  for(size_t i=0; i<inputImageVec.size(); i++)
  {
    // detect markers
    cv::aruco::detectMarkers(inputImageVec[i],markerDictionary,markerCorners,markerIds, params, rejectedCandidates);

    // insert vectors
    //    markerCornersVec.push_back(markerCorners);  // Vector of corner vectors
    markerIdsVec.push_back(markerIds);          // Vector of Id vectors
    rejectedCandidatesVec.push_back(rejectedCandidates);          // Vector of rejected candidates vectors
    //    rotVecsVec.push_back(rotVecs);              // Vector of rotation vectors
    //    transVecsVec.push_back(transVecs);          // Vector of transformation vectors

    // insert Maps
    std::stringstream markerStream;
    //    for (auto i = markerIds.begin(); i != markerIds.end(); ++i)
    for (size_t i=0 ; i< markerIds.size(); i++)
    {
      //      markerStream << *i << ' ';
      markerStream << markerIds[i] << ' ';
      //      markerCornersMap.insert(std::pair<int, Vec2f>(*i, markerCorners[0]));
      markerCornersMap.insert(std::pair<int, Vec2f>(markerIds[i], markerCorners[i]) );
    }
    // show detected markers for debugging
    //    ROS_DEBUG_STREAM(node.name_ << " ID's detected:\t" << markerStream.str());



    // merge all Ids and Corners
    markerIdsAll.insert( markerIdsAll.end(), markerIds.begin(), markerIds.end() );
    markerCornersAll.insert( markerCornersAll.end(), markerCorners.begin(), markerCorners.end() );
    rejectedCandidatesAll.insert( rejectedCandidatesAll.end(), rejectedCandidates.begin(), rejectedCandidates.end() );
  }
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||


  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Average Marker Calc ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||
  // It returns a pair representing the range of elements with key equal to '1'
  std::pair<MMAPIterator, MMAPIterator> result_1 = markerCornersMap.equal_range(1);
  std::pair<MMAPIterator, MMAPIterator> result_13 = markerCornersMap.equal_range(13);
  std::pair<MMAPIterator, MMAPIterator> result_40 = markerCornersMap.equal_range(40);

  // Iterate over the range
  markerCornersMean_.clear();
  Vec2f cornersCurr(4), cornersMean(4);

  // Total Elements in the range
  int count1 = std::distance(result_1.first, result_1.second);
  ROS_DEBUG_STREAM(node.name_ << " detection rate Id 1:\t" << count1*100/ACCUMULATE << "%");
  if(count1*100/ACCUMULATE<30)
    ROS_ERROR_STREAM(node.name_ << " Id '1' detection rate is " << count1*100/ACCUMULATE
                     << "%. Consider moving marker or recalibrating sensor intrinsics.");

  int count13 = std::distance(result_13.first, result_13.second);
  ROS_DEBUG_STREAM(node.name_ << " detection rate Id 13:\t" << count13*100/ACCUMULATE << "%");
  if(count13*100/ACCUMULATE<30)
    ROS_ERROR_STREAM(node.name_ << " Id '13' detection rate is " << count13*100/ACCUMULATE
                     << "%. Consider moving marker or recalibrating sensor intrinsics.");

  int count40 = std::distance(result_40.first, result_40.second);
  ROS_DEBUG_STREAM(node.name_ << " detection rate Id 40:\t" << count40*100/ACCUMULATE << "%");
  if(count40*100/ACCUMULATE<30)
    ROS_ERROR_STREAM(node.name_ << " Id '40' detection rate is " << count40*100/ACCUMULATE
                     << "%. Consider moving marker or recalibrating sensor intrinsics.");

  // Find average corners of marker 1
  //  cornersMean.clear();
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
  if(count1>0)
  {
    markerIdsMean_.push_back(1);
    markerCornersMean_.push_back(cornersMean);
  }

  // Find average corners of marker 13
  cornersMean = Vec2f(4);
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
  if(count13>0)
  {
    markerCornersMean_.push_back(cornersMean);
    markerIdsMean_.push_back(13);
  }

  // Find average corners of marker 40
  cornersMean = Vec2f(4);
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
  if(count40>0)
  {
    markerCornersMean_.push_back(cornersMean);
    markerIdsMean_.push_back(40);
  }
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||



  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Create cropping mask~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||
  cv::Mat mask(inputImage_.rows, inputImage_.cols,CV_8UC1);
//  std::vector<cv::Mat> maskVec(markerIdsMean.size());
  maskVec_.resize(markerIdsMean_.size());


  for (size_t i = 0; i < markerIdsMean_.size(); i++)
  {
    // init color
    mask.setTo(cv::Scalar(0) );

    // Calculate corners of padded aruco
    markerCornersPadded = markerCornersMean_;
    float padScale=0.0;
    padScale = 7.0/6.0-1.0; // new size/old size

    // extrapolate the padded corner position using distance between the diagonal corners
    for(int j=0 ; j<4 ; j++)
    {
      cv::Point2f corner;
      corner.x = markerCornersMean_[i][j].x + ( markerCornersMean_[i][j].x - markerCornersMean_[i][(j+2)%4].x ) * padScale*0.5;
      corner.y = markerCornersMean_[i][j].y + ( markerCornersMean_[i][j].y - markerCornersMean_[i][(j+2)%4].y ) * padScale*0.5;
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

    // Show current mask
    //    maskName = node.name_ + " Id:" + std::to_string(markerIdsMean[i]) + " mask";
    //    cv::imshow(maskName,maskVec[i]);
    //    cv::waitKey(30);
  }
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||

  // DEBUG
  //  cv::Mat dst;
  //  cv::bitwise_xor(maskVec[0], maskVec[1], dst);
  //  if(cv::countNonZero(dst) > 0) //check non-0 pixels
  //     std::cout << "mask 0 is different from mask1" << std::endl;
  //  else
  //    std::cout << "mask 0 = mask1" << std::endl;


  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Crop depth images ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||
  cv::Mat depthMatCropped(inputDepthVec[0].rows, inputDepthVec[0].cols,CV_32F); // change type to visualize?
  pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud_crop (new pcl::PointCloud<pcl::PointXYZ>);

  for(size_t j=0; j<inputDepthVec.size(); j++)
  {
    for (size_t i = 0; i < markerIdsMean_.size(); i++)
    {
      depthMatCropped.setTo(cv::Scalar(0)); // initialize depth to 0
      inputDepthVec[j].copyTo(depthMatCropped, maskVec_[i]);

      pointcloudFromDepthImage(depthMatCropped, intrinsicMatrix, src_cloud_crop);
      //    pointcloudFromDepthImage(depthMatCropped, intrinsicMatrix, src_cloud_crop, true, rectCrop, depthMatCropped);

      // add cropped source to accumulated marker cloud
      if (markerIdsMean_[i] == 1)  *src_cloud_crop1_ += *src_cloud_crop;
      if (markerIdsMean_[i] == 13) *src_cloud_crop2_ += *src_cloud_crop;
      if (markerIdsMean_[i] == 40) *src_cloud_crop3_ += *src_cloud_crop;

      // add cropped source to combined cloud
      *croppedCloud_ += *src_cloud_crop;
    } // end for markerIdsMean.
  } //end for inputDepthVec
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||


  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Transform ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||
  Eigen::Matrix4f tmatTemp = Eigen::Matrix4f::Identity();
  transform4x4.clear(); // clear transformations

  // calculate pose based on average corners
  cv::aruco::estimatePoseSingleMarkers(markerCornersMean_, arucoSquareDimension, intrinsicMat_, distortionMat_, rotVecs_, transVecs_);

  for(size_t i = 0; i < markerIdsMean_.size(); i++)
  {
    createTransMatrix(rotVecs_[i],transVecs_[i], tmatTemp);
    transform4x4.insert (std::pair<int, Eigen::Matrix4f> (markerIdsMean_[i]+100*acc_,tmatTemp ));
    transformMap_.insert (std::pair<int, Eigen::Matrix4f> (markerIdsMean_[i]+100*acc_,tmatTemp ));
  }
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~||

  acc_++;
}

void arucoProcessor::makeCroppedCloud()
{
  *croppedCloud_ += *src_cloud_crop1_;
  *croppedCloud_ += *src_cloud_crop2_;
  *croppedCloud_ += *src_cloud_crop3_;
}

//pcl::PointCloud<pcl::PointXYZ>::Ptr arucoProcessor::getCroppedCloud()
//{
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
//  *cloud += *src_cloud_crop1_;
//  *cloud += *src_cloud_crop2_;
//  *cloud += *src_cloud_crop3_;
//  return cloud;
//}

void arucoProcessor::createTransMatrix(cv::Vec3d rotationVectors, cv::Vec3d translationVectors, Eigen::Matrix4f& tMat)
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
  /*std::map<int, Eigen::Matrix4f> transMapUsed*/;
  transMapUsed.clear();

  for(int i=0 ; i<acc_ ; i++) // loop all images
  {
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
  }

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

} // end namespace wp3
