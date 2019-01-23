
#include "wp3_calibrator/defines.h"
#include "wp3_calibrator/functions.h"
#include "wp3_calibrator/visualization.h"
#include "wp3_calibrator/arucoprocessor.h"
#include "wp3_calibrator/imageconverter.h"
#include "wp3_calibrator/sensor.h"
#include "wp3_calibrator/validation.h"




// begin main  --------------------------------------------------------------------------------------------
int main (int argc, char** argv)
{
  //Set logging level: (DEBUG, INFO, WARN, ERROR, FATAL)
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }

  // init cloud vectors for visualizer
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vector_1, cloud_vector_2, cloud_vector_3, cloud_vector_4, cloud_vector_5, cloud_vector_6, worldClouds;

  // init transforms
  Eigen::Matrix4f transform_m4f;
  Eigen::Matrix4d transform_m4d;
  Eigen::Affine3d transform_affine;
  tf::Transform transform_tf;
  std::vector<int> markerIds;
  std::string fromString, toString;

  // timers
  double t1, t2, t3, t4, t5, t6, t7;

  // user info
  ROS_INFO_STREAM( "Keyboard controls of calibrator:" << std::endl
                   << "n   - new calibration "<<  std::endl
//                   << "s   - save image "<<  std::endl
//                   << "+   - next sensor "<<  std::endl
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
//  wp3::Visualization viewerAruco("Viewer Aruco");
//  viewerAruco.initializeSingle();
//  wp3::Visualization viewerArucoCropped("Viewer Aruco Cropped");
//  viewerArucoCropped.initializeSingle();
//  wp3::Visualization viewerWorld("Viewer World Map");
//  viewerWorld.initializeSingle();
  wp3::Visualization viewerIcpCropped("Viewer ICP Cropped");
  viewerIcpCropped.initializeSingle();
  wp3::Visualization viewerIcp("Viewer ICP");
  viewerIcp.initializeSingle();
  wp3::Visualization viewerMulti("Viewer All");
  viewerMulti.initialize();
#endif

  std::map<std::string, Eigen::Matrix4f> transMap, worldTransMap;

  // Subscribers:
  //  ros::Subscriber camera_info_sub = nh.subscribe(camera_info_topic, 1, cameraInfoCallback);

  // Publishers:
  static tf::TransformBroadcaster transform_publisher;

  // Init sensors from launch file
  std::vector<wp3::Sensor::Ptr> sensorVec;
  int num_sensors;
  wp3::loadSensors(node_handle, sensorVec, num_sensors);



  // init world ---------------------
  wp3::Sensor::Ptr worldSensor = boost::make_shared<wp3::Sensor>("world", node_handle); // create world "sensor"
  worldSensor->name_ = "world";
  wp3::arucoProcessor worldAruco; // world aruco processor
  Eigen::Matrix4f tf_worldToAvg;  // world transformation
  MarkerMapType worldMarkerMap;   // map of markers included in world
  pcl::PointCloud<pcl::PointXYZ>::Ptr worldCloud(new pcl::PointCloud<pcl::PointXYZ>); // world point cloud
  // ---------------------------------

  std::vector<MarkerMapType> tfMapVec(num_sensors);

  // Init aruco processor
  std::vector<wp3::arucoProcessor> apVec(num_sensors); // vector of arucoProcessor

  int key_cv = cv::waitKey(30);
  char key;
  //  bool init = false;  // don't autostart
  bool init = true;

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
    key_cv = cv::waitKey(100);

    if(init) // Create world map based on known marker poses
    {
      Eigen::Vector3d T(6.485, 5.9453, 0.310); // translation X,Y,Z
      Eigen::Vector3d R(0.0, 0.0, 0.0); // rotation euler Z, Y, X
      worldAruco.simulateMarker(1,R,T);

      T = Eigen::Vector3d(6.485, 3.959, 0.310); // translation X,Y,Z
      R = Eigen::Vector3d(0.0, 0.0, 0.0); // rotation euler Z, Y, X
      worldAruco.simulateMarker(13,R,T);

      T = Eigen::Vector3d(3.283, 3.958, 0.310); // translation X,Y,Z
      R = Eigen::Vector3d(0.0, 0.0, 0.0); // rotation euler Z, Y, X
      worldAruco.simulateMarker(40,R,T);

      worldAruco.getAverageTransformation(tf_worldToAvg, worldMarkerMap);
      worldCloud = worldAruco.getCroppedCloud();
      worldSensor->cloud1CrPtr_ = worldCloud;
      worldSensor->cloud1Ptr_ = worldCloud;
      worldSensor->transCamToAruco_ = tf_worldToAvg;
      worldSensor->setTfTopic("world");
    }


    // Start new calibration routine --------------------------------------------------------------------------
    if (key_cv == 110 || key == 'n' || init == true) // n
    {
      t1 = ros::Time::now().toSec();
      init = false;
      ROS_INFO_STREAM("Starting calibration routine" << std::endl);


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
        camPoseVec[i] = worldSensor->transCamToAruco_ * transAvgVec[i].inverse();
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
      ROS_INFO_STREAM("Performing ICP refinement between sensors and reference map"); // << sensorVec[i]->name_ << " and " << sensorVec[2]->name_);
      for(int i=0 ; i<num_sensors ; i++)
      {
        threadGroup.create_thread(boost::bind(wp3::refineTransformation, boost::ref(*sensorVec[i]), boost::ref(*worldSensor) ));
      }
      threadGroup.join_all();

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
      worldClouds.clear();


      // sensors
      for(int i=0 ; i<num_sensors ; i++)
      {
        cloud_vector_1.push_back(sensorVec[i]->cloud1CrPtr_); // step 1 Aruco cropped
        cloud_vector_2.push_back(sensorVec[i]->cloud2CrPtr_); // step 2 ICP cropped
        //        cloud_vector_3.push_back(sensorVec[i]->cloud3CrPtr_); // step 3 NA
        cloud_vector_4.push_back(sensorVec[i]->cloud1Ptr_); // step 1 Aruco
        cloud_vector_5.push_back(sensorVec[i]->cloud2Ptr_); // step 2 ICP
        //        cloud_vector_6.push_back(sensorVec[i]->cloud3Ptr_); // step 3 NA
      }

      // world map
      cloud_vector_1.push_back(worldSensor->cloud1CrPtr_); // step 1 Aruco cropped
      cloud_vector_2.push_back(worldSensor->cloud1CrPtr_); // step 2 ICP cropped
      worldClouds.push_back(worldSensor->cloud1CrPtr_); // step 2 ICP cropped

      transMap.clear();
      worldTransMap.clear();
      // origin
      transMap.insert (std::pair<std::string, Eigen::Matrix4f> (". World-Origin", Eigen::Matrix4f::Identity() ));
      worldTransMap.insert (std::pair<std::string, Eigen::Matrix4f> (". World-Origin", Eigen::Matrix4f::Identity() ));
      // world map markers
      for(auto j : worldMarkerMap)
      {
        transMap.insert (std::pair<std::string, Eigen::Matrix4f> (". World-"+std::to_string(j.first), j.second ));
        worldTransMap.insert (std::pair<std::string, Eigen::Matrix4f> (". World-"+std::to_string(j.first), j.second ));
      }
      // insert averaged world marker pose
      transMap.insert (std::pair<std::string, Eigen::Matrix4f> (". World-avg",worldSensor->transCamToAruco_ ));
      worldTransMap.insert (std::pair<std::string, Eigen::Matrix4f> (". World-avg",worldSensor->transCamToAruco_ ));

      for(int i=0 ; i<num_sensors ; i++)
      {
        // insert camera pose
        transMap.insert (std::pair<std::string, Eigen::Matrix4f> (sensorVec[i]->name_,camPoseVec[i]) );
        // insert pose of induvidual markers
        for(auto j : transMapUsedVec[i])
          transMap.insert (std::pair<std::string, Eigen::Matrix4f> (". "+sensorVec[i]->name_+"-"+std::to_string(j.first), camPoseVec[i]*j.second ));
        // insert averaged marker pose
        transMap.insert (std::pair<std::string, Eigen::Matrix4f> (". "+sensorVec[i]->name_+"-avg",camPoseVec[i]*transAvgVec[i] )); //camPoseVec[i]*transAvgVec[i]= identity

      }

#ifdef VIEWERS_ENABLED
      // view Aruco only
//      viewerAruco.runSingle(cloud_vector_4, transMap);
//      viewerArucoCropped.runSingle(cloud_vector_1, transMap);
      viewerIcp.runSingle(cloud_vector_5, transMap);
      viewerIcpCropped.runSingle(cloud_vector_2, transMap);
//      viewerWorld.runSingle(worldClouds, worldTransMap);
      // view all results
      viewerMulti.run(cloud_vector_1, cloud_vector_2, cloud_vector_3, cloud_vector_4, cloud_vector_5, cloud_vector_6, transMap);
#endif

      t7 = ros::Time::now().toSec();
      // Visualization end----------------------------------------------------------------------------------------

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

      // check results of known markers
      wp3::doValidation(sensorVec, worldSensor);
    } // End calibration routine --------------------------------------------------------------------------

    // publish transformations --------------------------

    // WORLD MAP
    markerIds = worldAruco.getMarkerIdsMean();
    // world to marker.avg
    transform_m4d = worldSensor->transCamToAruco_.cast<double>();
    transform_affine = Eigen::Affine3d(transform_m4d);
    tf::transformEigenToTF(transform_affine, transform_tf);
    fromString = worldSensor->getTfTopic();
    toString = worldSensor->name_ + ".avg";
    transform_publisher.sendTransform(tf::StampedTransform(transform_tf, ros::Time::now(), fromString, toString ) );

    // world to marker 1, 13, 40
    for(int j=0 ; j < worldAruco.transCamToArucoVec_.size() ; j++ )
    {
      transform_m4d = worldAruco.transCamToArucoVec_[j];
      transform_affine = Eigen::Affine3d(transform_m4d);
      tf::transformEigenToTF(transform_affine, transform_tf);
      fromString = worldSensor->getTfTopic();
      toString = worldSensor->name_ +"."+ std::to_string(markerIds[j]);
      transform_publisher.sendTransform(tf::StampedTransform(transform_tf, ros::Time::now(), fromString, toString ) );
    }

    // SENSORS
    for(int i=0 ; i<num_sensors ; i++)
    {
      markerIds = apVec[i].getMarkerIdsMean();

      // cam origin to single aruco markers
      for(int j=0 ; j < apVec[i].transCamToArucoVec_.size() ; j++ )
      {
        transform_m4d = apVec[i].transCamToArucoVec_[j];
        transform_affine = Eigen::Affine3d(transform_m4d);
        tf::transformEigenToTF(transform_affine, transform_tf);
        fromString = sensorVec[i]->getTfTopic();
        toString = sensorVec[i]->name_ +"."+ std::to_string(markerIds[j]);
        transform_publisher.sendTransform(tf::StampedTransform(transform_tf, ros::Time::now(), fromString, toString ) );
      }

      //      std::cout << "transform_AtoB_ICP - " << sensorVec[i]->name_ << std::endl << sensorVec[i]->transArucoToICP_;

      // world.marker.avg (ICP refinement)
      transform_m4f = sensorVec[i]->transArucoToICP_;
      transform_m4d = transform_m4f.cast <double>();
      transform_affine = Eigen::Affine3d(transform_m4d);
      tf::transformEigenToTF(transform_affine, transform_tf);
      //      fromString = sensorVec[i]->name_ + ".avg";
      fromString = worldSensor->name_ + ".avg";
      toString = sensorVec[i]->name_ + ".icp";
      transform_publisher.sendTransform(tf::StampedTransform(transform_tf, ros::Time::now(), fromString, toString ) );

      // publish camera pose
      transform_m4f =
          sensorVec[i]->transArucoToICP_ // pose ICP refined [i]
          * worldSensor->transCamToAruco_ // pose avg aruco [ref]
          * sensorVec[i]->transCamToAruco_.inverse(); // pose Cam [i]
      transform_m4d = transform_m4f.cast <double>();
      transform_affine = Eigen::Affine3d(transform_m4d);
      tf::transformEigenToTF(transform_affine, transform_tf);
      //      fromString = sensorVec[2]->getTfTopic();
      fromString = "world";
      toString = sensorVec[i]->getTfTopic();
      transform_publisher.sendTransform(tf::StampedTransform(transform_tf, ros::Time::now(), fromString, toString ) );
    }
    ros::spinOnce();

#ifdef VIEWERS_ENABLED
    // refresh viewers
//    viewerWorld.update();
    viewerMulti.update();
//    viewerAruco.update();
//    viewerArucoCropped.update();
    viewerIcp.update();
    viewerIcpCropped.update();
#endif


    ////    sleep ROS spinner
    //    loopRate.sleep();
  } // end while

} // end main
