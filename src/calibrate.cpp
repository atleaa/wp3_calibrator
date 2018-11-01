
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
  //Set logging level: (DEBUG, INFO, WARN, ERROR, FATAL)
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }

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
  ROS_INFO_STREAM( "Keyboard controls of calibrator:" << std::endl
                   << "n   - new image "<<  std::endl
                   << "s   - save image "<<  std::endl
                   << "+   - next sensor "<<  std::endl
                   << "esc - quit "<<  std::endl);

  // init ROS =============================================================
  sleep(3); // ensure that main is initialized before initing ROS
  // ROS messaging init
  ROS_INFO_STREAM("Starting: "<<  std::endl);

  // initialize ROS node
  ros::init(argc, argv, "wp3_calibrator");
  ros::NodeHandle nh;
  ros::spinOnce();

  // TODO: move creation of arucoprocessor?
  wp3::arucoProcessor aruco_A;
  wp3::arucoProcessor aruco_B;

  // init pcl viewer
  wp3::Visualization viewerAruco;
  viewerAruco.initializeSingle();
  wp3::Visualization viewerArucoCropped;
  viewerArucoCropped.initializeSingle();
  wp3::Visualization viewer;
  viewer.initialize();
#ifdef VIEW_ICP
  wp3::Visualization viewerICP;
  viewerICP.initializeSingle();
//  pcl::visualization::PCLVisualizer viewerICP ("ICP Visualizer");
//  viewerICP.setCameraPosition(-1.23666, -8.81802, -6.55671,
//                            -0.0567675, -2.09815, 5.04277,
//                            0.323175, -0.832741, 0.449555);
#endif

  std::map<std::string, Eigen::Matrix4f> transMap;

  std::string reference_node = "6";
  std::string calibration_order_initial[] = {"5", "2", "5", "3", "1"};

  //size_t numel_calib = sizeof(calibration_order_initial)/sizeof(calibration_order_initial[0]);
  size_t numel_calib = 5;

  // Read some parameters from launch file:
  //  nh.param(read, write, default);
//  std::string camera_info_topic;
//  nh.param("camera_info_topic", camera_info_topic, std::string("/jetson6/sd/camera_info"));

  // Subscribers:
//  ros::Subscriber camera_info_sub = nh.subscribe(camera_info_topic, 1, cameraInfoCallback);

  // Publishers:
  static tf::TransformBroadcaster transform_publisher;


  // TODO: make node vector and replace strings with config file or parameters (nh.param)
  wp3::Sensor nodeA("jetson6");
  nodeA.setCamera_info_topic("/jetson6/sd/camera_info");
  nodeA.setImageTopic("/jetson6/sd/image_color_rect");
  nodeA.setDepthTopic("/jetson6/sd/image_depth_rect");
//  nodeA.setCloudTopic("/master/jetson1/points");
  nodeA.setCloudTopic("/jetson6/sd/points");

  wp3::Sensor nodeB("jetson5");
  nodeB.setCamera_info_topic("/jetson5/sd/camera_info");
  nodeB.setImageTopic("/jetson5/sd/image_color_rect");
  nodeB.setDepthTopic("/jetson5/sd/image_depth_rect");
//  nodeB.setCloudTopic("/master/jetson2/points");
  nodeB.setCloudTopic("/jetson5/sd/points");


  wp3::init_reference(reference_node); // create first initial transformation

  r_mutex2.lock();
  int key = cv::waitKey(30);
//  bool init = false;  // don't autostart
  bool init = true;
  size_t calib_counter = 0;
  size_t viz_counter = 0;

  wp3::openGlobalReference(transform_reference_global, reference_node);

  // init ROS spinner
//  ros::Rate loopRate(ROS_LOOPRATE);
//  ros::AsyncSpinner spinner(8);
//  spinner.start();

  usleep(1000000); // give the spinner some time to start (1000 ms)

  //  begin main while ------------------------------------------------------------------------------------------
  while ((key != 27) && ros::ok())  // not ESC
  {
    key = cv::waitKey(30);

    // the process below is performed initially and updated every time the user presses the "n" key on the RGB image
    if (key == 110 || init == true) // n
    {
      ROS_INFO_STREAM("Starting calibration routine" << std::endl);
      init = false;
      aruco_A.clearAll();
      aruco_B.clearAll();
      nodeA.clear();
      nodeB.clear();

      ROS_INFO_STREAM("Reading topics and detecting markers to accumulate data.");
      for(int i=0;i<ACCUMULATE;i++) // accumulate point clouds
      {
        // reading topics
        nodeA.readTopics(true);
        nodeB.readTopics(true);

        // detect and accumulate cropped clouds
        aruco_A.detectMarkers(nodeA, transformMap_A, reference_node, Mask);
        aruco_B.detectMarkers(nodeB, transformMap_B, calibration_order_initial[calib_counter], Mask);

        // Accumulate full clouds
        nodeA.cloudCrPtr_ = aruco_A.getCroppedCloud();
        nodeB.cloudCrPtr_ = aruco_B.getCroppedCloud();
//        nodeA.appendClouds();
//        nodeB.appendClouds();
      }

//      transform_A = transformMap_A.at(101); // Available id's: 1, 13, 40
//      transform_B = transformMap_B.at(101);

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

      ROS_INFO_STREAM("Performing ICP between " << nodeA.name_ << " and " << nodeB.name_);
//      wp3::calcTransMats(nodeA, nodeB, transMat_avgA, transMat_avgB, transform_reference_global, transform_ICP1_print, ICP1_fitness_to_print);
#ifdef VIEW_ICP
      wp3::calcTransMats(nodeA, nodeB, transMat_avgA, transMat_avgB, transform_reference_global, transform_ICP1_print, ICP1_fitness_to_print, viewerICP);
#else
      wp3::calcTransMats(nodeA, nodeB, transMat_avgA, transMat_avgB, transform_reference_global, transform_ICP1_print, ICP1_fitness_to_print);
#endif
      ROS_INFO_STREAM("ICP complete with fitness score: " << ICP1_fitness_to_print);

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
//      cloud_vector_6.push_back(nodeA.cloud3Ptr_); // step 3
      cloud_vector_6.push_back(nodeB.cloudPtr_);
      // additional clouds TMP
//      cloud_vector_6.push_back(nodeA.cloud2Ptr_); // step 2
//      cloud_vector_6.push_back(nodeA.cloud2CrPtr_); // step 2
      cloud_vector_6.push_back(nodeB.cloudCrPtr_);



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



      // view Aruco only
      viewerAruco.runSingle(cloud_vector_4, transMap);
      viewerArucoCropped.runSingle(cloud_vector_1, transMap);
      // view all results
      viewer.run(cloud_vector_1, cloud_vector_2, cloud_vector_3, cloud_vector_4, cloud_vector_5, cloud_vector_6, transMap);
    }

    // if "s" is pressed on the RGB image the transformation from ICP1 and fitness result of ICP2 are saved
    if (key == 115) // s
    {
      wp3::saveResults(transform_ICP1_print, ICP1_fitness_to_print, calibration_order_initial[calib_counter]);
//      std::cout << numel_calib << std::endl;
    }


    if (key == 43) // +
    {
      calib_counter += 1;

      if (calib_counter >= numel_calib)
      {
        calib_counter = 0;
      }
//      std::cout << "evaluating node " << reference_node << "vs" << calibration_order_initial[calib_counter] << std::endl;
    }

//    viewer.update();
    viewer.update();
    viewerAruco.update();
    viewerArucoCropped.update();
#ifdef VIEW_ICP
//    viewerICP.spinOnce();
#endif

    // TF PUBLISHERS
    // TF - Manual world reference
    tf::Transform worldToReference_tf; //(rot,trans)
    tf::Quaternion q;
//    worldToReference_tf.setOrigin( tf::Vector3(7.6, 0.7, 4.2) ); // j1
    worldToReference_tf.setOrigin( tf::Vector3(0.1, 4.8, 4.2) ); // j4
//    q.setRPY(-2.44, 0, 0.7);  // j1
    q.setRPY(-2.50, 0, -1.571);  // j4
    worldToReference_tf.setRotation(q);
//    transform_publisher.sendTransform(tf::StampedTransform(worldToReference_tf, ros::Time::now(), "world", nodeA.name_+"_ir_optical_frame") );


    // TF - A to B
    Eigen::Matrix4f transform_m4f = transform_ICP1_print.inverse(); // Matrix to convert
    Eigen::Matrix4d transform_m4d(transform_m4f.cast <double>());
    Eigen::Affine3d transform_affine(transform_m4d);
    tf::Transform transform_tf;
    tf::transformEigenToTF(transform_affine, transform_tf);
    transform_publisher.sendTransform(tf::StampedTransform(transform_tf, ros::Time::now(), nodeA.name_+"_ir_optical_frame", nodeB.name_+"_ir_optical_frame") );
    ros::spinOnce();


////    sleep ROS spinner
//    loopRate.sleep();
  } // end while

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  r_mutex2.unlock();
} // end main
