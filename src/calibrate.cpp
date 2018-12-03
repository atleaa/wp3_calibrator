
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
  boost::mutex mtx;

  // init cloud vectors for visualizer
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vector_1, cloud_vector_2, cloud_vector_3, cloud_vector_4, cloud_vector_5, cloud_vector_6;

  // init transforms
  Eigen::Matrix4f transform_A = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f transform_B = Eigen::Matrix4f::Identity();
  //  std::map<float, Eigen::Matrix4f> transformMap_A;
  //  std::map<float, Eigen::Matrix4f> transformMap_B;
  MarkerMapType transformMap_A;
  MarkerMapType transformMap_B;

  // timers
  double t1, t2, t3, t4, t5, t6, t7;


  Eigen::Matrix4f transform_ICP1_print = Eigen::Matrix4f::Identity();  Eigen::Matrix4f transform_reference_global = Eigen::Matrix4f::Identity();
//  std::vector<Eigen::Matrix4f> transformVec;

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
//  ros::init(argc, argv, "wp3_calibrator");
  ros::init(argc, argv, "calibrate");
  ros::NodeHandle node_handle("~"); // private node handle
  ros::spinOnce();

  sleep(3); // wait for ROS debugger

#ifdef VIEWERS_ENABLED
  // init pcl viewer
  wp3::Visualization viewerAruco("Viewer Aruco");
  viewerAruco.initializeSingle();
  wp3::Visualization viewerArucoCropped("Viewer Aruco Cropped");
  viewerArucoCropped.initializeSingle();
  wp3::Visualization viewer("Viewer All");
  viewer.initialize();
#endif
#ifdef VIEW_ICP
  wp3::Visualization viewerICP("Viewer ICP steps");
  viewerICP.initializeSingle();
#endif

  std::map<std::string, Eigen::Matrix4f> transMap;

  std::string reference_node = "1";
  std::string calibration_order_initial[] = {"6", "2", "5", "3", "1"};

  //size_t numel_calib = sizeof(calibration_order_initial)/sizeof(calibration_order_initial[0]);
  size_t numel_calib = 5;

  // Subscribers:
//  ros::Subscriber camera_info_sub = nh.subscribe(camera_info_topic, 1, cameraInfoCallback);

  // Publishers:
  static tf::TransformBroadcaster transform_publisher;


  // Init sensors from launch file
  std::vector<wp3::Sensor::Ptr> sensorVec;
  int num_sensors;
  wp3::loadSensors(node_handle, sensorVec, num_sensors);


  // Init vector of transformations to identity
  std::vector<Eigen::Matrix4f> transformVec(num_sensors, Eigen::Matrix4f::Identity() );
  std::vector<MarkerMapType> tfMapVec(num_sensors);

  // Init aruco processor

  wp3::arucoProcessor aruco_A;
  wp3::arucoProcessor aruco_B;

  std::vector<wp3::arucoProcessor> apVec(num_sensors); // vector of arucoProcessor


  wp3::init_reference(reference_node); // create first initial transformation

//  r_mutex2.lock();
  int key_cv = cv::waitKey(30);
  char key;
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


  // Init threads
//  boost::thread threads[num_sensors];


  //  begin main while ------------------------------------------------------------------------------------------
  while ((key_cv != 27) && ros::ok())  // not ESC
  {
    key_cv = cv::waitKey(30);

//    ROS_INFO_STREAM("\nn to calibrate\nq to quit");
//    cin >> key;
//    if(key=='q') break;


    // Start new calibration routine --------------------------------------------------------------------------
    if (key_cv == 110 || key == 'n' || init == true) // n
    {
      t1 = ros::Time::now().toSec();
      init = false;
      ROS_INFO_STREAM("Starting calibration routine" << std::endl);

      // makes sure to clear all saved data!
//      sensorVec[0].reset();
//      sensorVec[1].reset();
//      sensorVec[2].reset();
//      sensorVec[3].reset();
//      sensorVec[4].reset();

//      delete &sensorVec;
//      std::vector<wp3::Sensor::Ptr> sensorVec;
//      wp3::loadSensors(node_handle, sensorVec, num_sensors);


      // reading topics -----------------------------------------------------------------------------------------
      ROS_INFO_STREAM("Recording topics...");
      // threads for each worker
      boost::thread_group threadGroup;
      for(int i=0 ; i < num_sensors ; i++)
      {
//        sensorVec[i]->readTopics(true);
        threadGroup.create_thread( boost::bind( &wp3::Sensor::readTopics, sensorVec[i], true ) );
      }
      // wait for threads to join
      threadGroup.join_all();
      ROS_INFO_STREAM("Recording complete" << std::endl);

      t2 = ros::Time::now().toSec();


      // detect and accumulate cropped clouds --------------------------------------------------------------------
      ROS_INFO_STREAM("Processing recorded data...");
//      boost::thread_group threadGroup2;
      for(int i=0 ; i < num_sensors ; i++)
      {
//        apVec[i].detectMarkers(*sensorVec[i], tfMapVec[i]);
        threadGroup.create_thread( boost::bind( &wp3::arucoProcessor::processImages, boost::ref(apVec[i]), *sensorVec[i], tfMapVec[i] ) );
      }
      threadGroup.join_all();
      ROS_INFO_STREAM("Processing complete" << std::endl);

      t3 = ros::Time::now().toSec();


      // View detected images all nodes --------------------------------------------------------------------------
      for(int i=0 ; i < num_sensors ; i++)
      {
#ifdef VIEW_IMAGES
        apVec[i].viewImages(*sensorVec[i]);
#endif
        // clear color images to save memory
        sensorVec[i]->clearImageMatVec();
      }
      // end View detected images all nodes --------------------------------------------------------------------------


      t4 = ros::Time::now().toSec();


      // get transformation based on aruco markers
      //TODO: map intersection,  https://stackoverflow.com/questions/3772664/intersection-of-two-stl-maps
      std::vector<Eigen::Matrix4f> transAvgVec(num_sensors, Eigen::Matrix4f::Identity());
      std::vector<MarkerMapType> transMapUsedVec(num_sensors);
      std::vector<Eigen::Matrix4f> camPoseVec(num_sensors, Eigen::Matrix4f::Identity() );
      for(int i=0 ; i<num_sensors ; i++)
      {
        apVec[i].getAverageTransformation(transAvgVec[i], transMapUsedVec[i]);
        camPoseVec[i] = transAvgVec[i].inverse();
        sensorVec[i]->transCamToAruco_ = transAvgVec[i];
      }

      // do aruco transformation
      for(int i=0 ; i < num_sensors ; i++)
      {
        pcl::transformPointCloud (*sensorVec[i]->cloudCrPtr_, *sensorVec[i]->cloud1CrPtr_, camPoseVec[i] );
        pcl::transformPointCloud (*sensorVec[i]->cloudPtr_, *sensorVec[i]->cloud1Ptr_, camPoseVec[i]);
      }
      // BREAK HERE IF RECORDING IS BAD?? (initial transformation wrong)

      t5 = ros::Time::now().toSec();

      // Refine calibration --------------------------------------------------------------------------------------------------
#ifdef VIEW_ICP
//      wp3::calcTransMats(*sensorVec[0], *sensorVec[1], transMat_avgA, transMat_avgB, transform_reference_global, transform_ICP1_print, ICP1_fitness_to_print, viewerICP);
#else
      ROS_INFO_STREAM("Performing ICP refinement between sensors and reference map"); // << sensorVec[i]->name_ << " and " << sensorVec[2]->name_);
      for(int i=0 ; i<num_sensors ; i++)
      {
        if(i==2) continue; // skip reference node
//        wp3::refineTransformation(*sensorVec[i], *sensorVec[2]);
        threadGroup.create_thread(boost::bind(wp3::refineTransformation, boost::ref(*sensorVec[i]), boost::ref(*sensorVec[2]) ));
      }
      threadGroup.join_all();
#endif
      ROS_INFO_STREAM("All ICP refinement complete");
      // end Refine calibration ----------------------------------------------------------------------------------------------

      t6 = ros::Time::now().toSec();


      // Visualization -------------------------------------------------------------------------------------------
      // create cloud vectors for visualization
      cloud_vector_1.clear();
      cloud_vector_2.clear();
      cloud_vector_3.clear();
      cloud_vector_4.clear();
      cloud_vector_5.clear();
      cloud_vector_6.clear();
      for(int i=0 ; i<num_sensors ; i++)
      {
        cloud_vector_1.push_back(sensorVec[i]->cloud1CrPtr_); // step 1 Aruco cropped
        cloud_vector_2.push_back(sensorVec[i]->cloud2CrPtr_); // step 2 ICP cropped
//        cloud_vector_3.push_back(sensorVec[i]->cloud3CrPtr_); // step 3 NA
        cloud_vector_4.push_back(sensorVec[i]->cloud1Ptr_); // step 1 Aruco
        cloud_vector_5.push_back(sensorVec[i]->cloud2Ptr_); // step 2 ICP
//        cloud_vector_6.push_back(sensorVec[i]->cloud3Ptr_); // step 3 NA
      }



      transMap.clear();
      for(int i=0 ; i<num_sensors ; i++)
      {
        // insert camera pose
        transMap.insert (std::pair<std::string, Eigen::Matrix4f> (sensorVec[i]->name_,camPoseVec[i]) );
//      transMap.insert (std::pair<std::string, Eigen::Matrix4f> (sensorVec[0]->name_,camA ));
//      transMap.insert (std::pair<std::string, Eigen::Matrix4f> (sensorVec[1]->name_,camB ));

        // insert pose of induvidual markers
        for(auto j : transMapUsedVec[i])
          transMap.insert (std::pair<std::string, Eigen::Matrix4f> (". "+sensorVec[i]->name_+"-"+std::to_string(j.first), camPoseVec[i]*j.second ));

        // insert averaged marker pose
        transMap.insert (std::pair<std::string, Eigen::Matrix4f> (". "+sensorVec[i]->name_+"-avg",camPoseVec[i]*transAvgVec[i] )); //camPoseVec[i]*transAvgVec[i]= identity

      }

//      // Arucos
//      std::map<int, Eigen::Matrix4f>::iterator it;
//      for ( it = transMapUsed_A.begin(); it != transMapUsed_A.end(); it++ )
//        transMap.insert (std::pair<std::string, Eigen::Matrix4f> (". "+sensorVec[0]->name_+"-"+std::to_string(it->first), camA*it->second ));

//      for ( it = transMapUsed_B.begin(); it != transMapUsed_B.end(); it++ )
//        transMap.insert (std::pair<std::string, Eigen::Matrix4f> (". "+sensorVec[1]->name_+"-"+std::to_string(it->first), camB*it->second ));

//      transMap.insert (std::pair<std::string, Eigen::Matrix4f> (". "+sensorVec[0]->name_+"-avg",camA*transMat_avgA ));
//      transMap.insert (std::pair<std::string, Eigen::Matrix4f> (". "+sensorVec[1]->name_+"-avg",camB*transMat_avgB ));


#ifdef VIEWERS_ENABLED
      // view Aruco only
      viewerAruco.runSingle(cloud_vector_4, transMap);
      viewerArucoCropped.runSingle(cloud_vector_1, transMap);
      // view all results
      viewer.run(cloud_vector_1, cloud_vector_2, cloud_vector_3, cloud_vector_4, cloud_vector_5, cloud_vector_6, transMap);
#endif

      t7 = ros::Time::now().toSec();
      // Visualization end----------------------------------------------------------------------------------------

//      double timestamp = ros::Time::now().toSec() - initial_time.toSec();
//      double sleep_timeA = ros::Time::now().toSec() - presleep_time.toSec();
//      last_time = current_time;
//      current_time = ros::Time::now();
//      scan_time1 = (current_time - last_time).toSec();

      // Calibration summary
      ROS_INFO_STREAM(std::endl << "\t\t\tCALIBRATION SUMMARY" << std::endl
                      << "=====================================================" << std::endl
                      << "Aruco size:\t" << ARUCODIMENSION << std::endl
                      << "Color images node:\t" << IMAGES_C << std::endl
                      << "Depth images per node:\t" << IMAGES_D << std::endl
                      << "Point clouds per node:\t" << CLOUDS << std::endl
                      << "ICP, starting search radius:\t" << ICP_MAX_CORR_DIST << std::endl
                      << "ICP, convergence limit:\t" << ICP_CONVERGE << std::endl
                      << "ICP, maximum rounds:\t" << ICP_MAX_ROUNDS << std::endl
                      << "ICP, maximum iterations in first round:\t" << ICP_ITERATIONS << std::endl
                      << "ICP, transformation step size:\t" << ICP_TRANS_EPSILON << std::endl
                      << "-----------------------------------------------------" << std::endl
                      << "Time to record data:\t" << t2-t1 << std::endl
                      << "Time to process images:\t" << t3-t2 << std::endl
                      << "Time to view images:\t" << t4-t3 << std::endl
                      << "Time to calculate aruco transformations:\t" << t5-t4 << std::endl
                      << "Time to perform ICP refinement:\t" << t6-t5 << std::endl
                      << "Time to do point cloud visualization:\t" << t7-t6 );


    } // End calibration routine --------------------------------------------------------------------------

    // if "s" is pressed on the RGB image the transformation from ICP1 and fitness result of ICP2 are saved
    if (key_cv == 115) // s
    {
      wp3::saveResults(transform_ICP1_print, ICP1_fitness_to_print, calibration_order_initial[calib_counter]);
//      std::cout << numel_calib << std::endl;
    }


    if (key_cv == 43) // +
    {
      calib_counter += 1;

      if (calib_counter >= numel_calib)
      {
        calib_counter = 0;
      }
//      std::cout << "evaluating node " << reference_node << "vs" << calibration_order_initial[calib_counter] << std::endl;
    }


    // TF PUBLISHERS
    // TF - Manual world reference
    tf::Transform worldToReference_tf; //(rot,trans)
    tf::Quaternion q;
    worldToReference_tf.setOrigin( tf::Vector3(7.6, 0.7, 4.2) ); // j1
//    worldToReference_tf.setOrigin( tf::Vector3(0.1, 4.8, 4.2) ); // j4
    q.setRPY(-2.44, 0, 0.7);  // j1
//    q.setRPY(-2.50, 0, -1.571);  // j4
    worldToReference_tf.setRotation(q);
//    transform_publisher.sendTransform(tf::StampedTransform(worldToReference_tf, ros::Time::now(), "world", sensorVec[0]->getTfTopic() ) );
//    transform_publisher.sendTransform(tf::StampedTransform(worldToReference_tf, ros::Time::now(), "world", "jetson1_ir_optical_frame" ) );


// publish transformations --------------------------
    Eigen::Matrix4f transform_m4f;
    Eigen::Matrix4d transform_m4d;
    Eigen::Affine3d transform_affine;
    tf::Transform transform_tf;

    // convert and publish ROS TF
    for(int i=0 ; i<num_sensors ; i++)
    {
      // cam to average between aruco markers
      transform_m4d = sensorVec[i]->transCamToAruco_.cast<double>();
      transform_affine = Eigen::Affine3d(transform_m4d);
      tf::transformEigenToTF(transform_affine, transform_tf);
      std::string fromString = sensorVec[i]->getTfTopic();
      std::string toString = sensorVec[i]->name_ + ".avg";
      transform_publisher.sendTransform(tf::StampedTransform(transform_tf, ros::Time::now(), fromString, toString ) );

      // Cam to aruco markers
      for(int j=0 ; j < apVec[i].transCamToArucoVec_.size() ; j++ )
      {
        // publish
        transform_m4d = apVec[i].transCamToArucoVec_[j];
        transform_affine = Eigen::Affine3d(transform_m4d);
        tf::transformEigenToTF(transform_affine, transform_tf);
        std::string fromString = sensorVec[i]->getTfTopic();
        std::string toString = sensorVec[i]->name_ + std::to_string(j);
        transform_publisher.sendTransform(tf::StampedTransform(transform_tf, ros::Time::now(), fromString, toString ) );
      }

      // aruco to ICP
      transform_m4f = sensorVec[i]->transArucoToICP_;
      transform_m4d = transform_m4f.cast <double>();
      transform_affine = Eigen::Affine3d(transform_m4d);
      tf::transformEigenToTF(transform_affine, transform_tf);
      fromString = sensorVec[i]->name_ + ".avg";
      toString = sensorVec[i]->name_ + ".ICP";
      transform_publisher.sendTransform(tf::StampedTransform(transform_tf, ros::Time::now(), fromString, toString ) );

      if(i==2) continue; // skip rest if reference node
      // publish camera pose
      transform_m4f = sensorVec[2]->transCamToAruco_ // pose avg aruco [ref]
                    * sensorVec[i]->transArucoToICP_ // pose ICP refined [i]
                    * sensorVec[i]->transCamToAruco_.inverse(); // pose Cam [i]
      transform_m4d = transform_m4f.cast <double>();
      transform_affine = Eigen::Affine3d(transform_m4d);
      tf::transformEigenToTF(transform_affine, transform_tf);
      fromString = sensorVec[2]->getTfTopic();
      toString = sensorVec[i]->getTfTopic();
      transform_publisher.sendTransform(tf::StampedTransform(transform_tf, ros::Time::now(), fromString, toString ) );
    }
    ros::spinOnce();

    // TF - A to B
//    Eigen::Matrix4f transform_m4f = transform_ICP1_print.inverse(); // Matrix to convert
//    Eigen::Matrix4d transform_m4d(transform_m4f.cast <double>());
//    Eigen::Affine3d transform_affine(transform_m4d);
//    tf::Transform transform_tf;
//    tf::transformEigenToTF(transform_affine, transform_tf);
////    transform_publisher.sendTransform(tf::StampedTransform(transform_tf, ros::Time::now(), sensorVec[0]->getTfTopic(), sensorVec[1]->getTfTopic() ) );
////    transform_publisher.sendTransform(tf::StampedTransform(transform_tf, ros::Time::now(), "jetson1_ir_optical_frame", "jetson6_ir_optical_frame" ) );
//    ros::spinOnce();

#ifdef VIEWERS_ENABLED
    // refresh viewers
    viewer.update();
    viewerAruco.update();
    viewerArucoCropped.update();
#endif
#ifdef VIEW_ICP
    //    viewerICP.spinOnce();
#endif

////    sleep ROS spinner
//    loopRate.sleep();
  } // end while

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  r_mutex2.unlock();
} // end main
