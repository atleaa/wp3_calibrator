
#include "wp3_calibrator/defines.h"
#include "wp3_calibrator/classtest.h"
#include "wp3_calibrator/functions.h"
#include "wp3_calibrator/visualization.h"
#include "wp3_calibrator/arucoprocessor.h"
#include "wp3_calibrator/imageconverter.h"
#include "wp3_calibrator/sensor.h"

// Global variables:
//std::vector<cv::Vec3f> camera_colors;     // vector containing colors to use to identify cameras in the network
//std::map<std::string, int> color_map;     // map between camera frame_id and color

// begin main  --------------------------------------------------------------------------------------------
int main (int argc, char** argv)
{

  std::recursive_mutex r_mutex2;

  // init cloud vectors for visualizer
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vector_1, cloud_vector_2, cloud_vector_3, cloud_vector_4, cloud_vector_5, cloud_vector_6;

  // init transforms
  Eigen::Matrix4f transform_A = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f transform_B = Eigen::Matrix4f::Identity();
  //  std::map<float, Eigen::Matrix4f> transformMap_A;
  //  std::map<float, Eigen::Matrix4f> transformMap_B;
  MarkerMapType transformMap_A;
  MarkerMapType transformMap_B;


  Eigen::Matrix4f transform_ICP1_print = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f transform_reference_global = Eigen::Matrix4f::Identity();
  std::vector<Eigen::Matrix4f> transformVec;

  // init variables
  double ICP1_fitness_to_print;

  // user info
  std::cout << "n   - new image "<<  std::endl
            << "s   - save image "<<  std::endl
            << "+   - next sensor "<<  std::endl
            << "esc - quit "<<  std::endl;

  // init ROS =============================================================
  sleep(3); // ensure that main is initialized before initing ROS
  // ROS messaging init
  std::cout << "Starting: "<<  std::endl;
  ros::init(argc, argv, "aruco_tf_publisher");
  ros::NodeHandle nh;
  ros::spinOnce();

  // TODO: move creation of arucoprocessor?
  wp3::arucoProcessor aruco_A;
  wp3::arucoProcessor aruco_B;

  // init pcl viewer
  wp3::Visualization viewer;
  viewer.initialize();
  wp3::Visualization viewerAruco;
  viewerAruco.initializeSingle();
  std::map<std::string, Eigen::Matrix4f> transMap;

  std::string reference_node = "4";
  std::string calibration_order_initial[] = {"6", "2", "5", "3", "1"};

  //size_t numel_calib = sizeof(calibration_order_initial)/sizeof(calibration_order_initial[0]);
  size_t numel_calib = 5;

  std::cout << calibration_order_initial[0] << std::endl;

  // TODO: make node vector
  wp3::Sensor nodeA("jetson4");
  nodeA.setDepthTopic("/jetson4/hd/image_depth_rect");
  nodeA.setImageTopic("/jetson4/hd/image_color_rect");
  nodeA.setCloudTopic("/master/jetson4/points");

  wp3::Sensor nodeB("jetson6");
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
  while ((key != 27) && ros::ok())  // not ESC
  {
    key = cv::waitKey(30);

    // the process below is performed initially and updated every time the user presses the "n" key on the RGB image
    if (key == 110 || init == true) // n
    {
      init = false;
      aruco_A.clearAll();
      aruco_B.clearAll();

      for(int i=0;i<5;i++)
      {
        nodeA.readTopics(true);
        nodeB.readTopics(true);

        aruco_A.detectMarkers(nodeA.imageMat_, nodeA.depthMat_, transformMap_A, reference_node);
//        aruco_A.getCroppedCloud(nodeA.cloudCrPtr_);
        //      transform_A = transformMap_A.at(1); // Available id's: 1, 13, 40

        aruco_B.detectMarkers(nodeB.imageMat_, nodeB.depthMat_, transformMap_B, calibration_order_initial[calib_counter]);
//        aruco_B.getCroppedCloud(nodeB.cloudCrPtr_);
        //      transform_B = transformMap_B.at(1);
      }
      nodeA.cloudCrPtr_ = aruco_A.getCroppedCloud();
      nodeB.cloudCrPtr_ = aruco_B.getCroppedCloud();

      //TODO: map intersection,  https://stackoverflow.com/questions/3772664/intersection-of-two-stl-maps
      Eigen::Matrix4f transMat_avgA = Eigen::Matrix4f::Identity();
      std::map<int, Eigen::Matrix4f> transMapUsed_A;
      aruco_A.getAverageTransformation(transMat_avgA, transMapUsed_A);

      Eigen::Matrix4f transMat_avgB = Eigen::Matrix4f::Identity();
      std::map<int, Eigen::Matrix4f> transMapUsed_B;
      aruco_B.getAverageTransformation(transMat_avgB, transMapUsed_B);
      // TODO: make function/loop for average Aruco pose

//      Eigen::Matrix4f camA = transform_B*transform_A.inverse();
      Eigen::Matrix4f camA = transMat_avgB*transMat_avgA.inverse();
      Eigen::Matrix4f camB = Eigen::Matrix4f::Identity();

//      wp3::calcTransMats(nodeA, nodeB, transform_A, transform_B, transform_reference_global, transform_ICP1_print, ICP1_fitness_to_print);
      wp3::calcTransMats(nodeA, nodeB, transMat_avgA, transMat_avgB, transform_reference_global, transform_ICP1_print, ICP1_fitness_to_print);


      //make cloud vector --> TODO: use a loop to create vectors
      //      for (unsigned int i = 0; i < curr_cloud_vector.size(); i++)
      //      {
      //        pcl::PointCloud<pcl::PointXYZRGB>::Ptr empty_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);


      //      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vector_1,
      // cropped clouds
      cloud_vector_1.clear();
      cloud_vector_1.push_back(nodeA.cloud1CrPtr_); // step 1
      cloud_vector_1.push_back(nodeB.cloudCrPtr_);
      cloud_vector_2.clear();
      cloud_vector_2.push_back(nodeA.cloud2CrPtr_); // step 2
      cloud_vector_2.push_back(nodeB.cloudCrPtr_);
      cloud_vector_3.clear();
      cloud_vector_3.push_back(nodeA.cloud3CrPtr_); // step 3
      cloud_vector_3.push_back(nodeB.cloudCrPtr_);

      // full clouds
      cloud_vector_4.clear();
      cloud_vector_4.push_back(nodeA.cloud1Ptr_); // step 1
      cloud_vector_4.push_back(nodeB.cloudPtr_);
      cloud_vector_5.clear();
      cloud_vector_5.push_back(nodeA.cloud2Ptr_); // step 2
      cloud_vector_5.push_back(nodeB.cloudPtr_);
      cloud_vector_6.clear();
      cloud_vector_6.push_back(nodeA.cloud3Ptr_); // step 3
      cloud_vector_6.push_back(nodeB.cloudPtr_);

      transMap.clear();
      transMap.insert (std::pair<std::string, Eigen::Matrix4f> (nodeA.name_,camA ));
      transMap.insert (std::pair<std::string, Eigen::Matrix4f> (nodeB.name_,camB ));

      // Arucos
      std::map<int, Eigen::Matrix4f>::iterator it;
      for ( it = transMapUsed_A.begin(); it != transMapUsed_A.end(); it++ )
        transMap.insert (std::pair<std::string, Eigen::Matrix4f> (". "+nodeA.name_+"-"+std::to_string(it->first), camA*it->second ));
      for ( it = transMapUsed_B.begin(); it != transMapUsed_B.end(); it++ )
        transMap.insert (std::pair<std::string, Eigen::Matrix4f> (". "+nodeB.name_+"-"+std::to_string(it->first), camB*it->second ));
//      transMap.insert (std::pair<std::string, Eigen::Matrix4f> (". "+nodeB.name_+"-1", camB*transformMap_B.at(1) ));
//      transMap.insert (std::pair<std::string, Eigen::Matrix4f> (". "+nodeB.name_+"-13", camB*transformMap_B.at(13) ));
//      transMap.insert (std::pair<std::string, Eigen::Matrix4f> (". "+nodeB.name_+"-40", camB*transformMap_B.at(40) ));

//      transMap.insert (std::pair<std::string, Eigen::Matrix4f> (". "+nodeA.name_+"-1", camA*transformMap_A.at(1) ));
//      transMap.insert (std::pair<std::string, Eigen::Matrix4f> (". "+nodeA.name_+"-13", camA*transformMap_A.at(13) ));
//      transMap.insert (std::pair<std::string, Eigen::Matrix4f> (". "+nodeA.name_+"-40", camA*transformMap_A.at(40) ));

      transMap.insert (std::pair<std::string, Eigen::Matrix4f> (". "+nodeA.name_+"-avg",camA*transMat_avgA ));
      transMap.insert (std::pair<std::string, Eigen::Matrix4f> (". "+nodeB.name_+"-avg",camB*transMat_avgB ));



      // view all results
      viewer.run(cloud_vector_1, cloud_vector_2, cloud_vector_3, cloud_vector_4, cloud_vector_5, cloud_vector_6, transMap);

      // view Aruco only
      viewerAruco.runSingle(cloud_vector_4, transMap);

    }

    // if "s" is pressed on the RGB image the transformation from ICP1 and fitness result of ICP2 are saved
    if (key == 115) // s
    {
      wp3::saveResults(transform_ICP1_print, ICP1_fitness_to_print, calibration_order_initial[calib_counter]);
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

//    viewer.update();
    viewerAruco.update();
  }

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  r_mutex2.unlock();
} // end main
