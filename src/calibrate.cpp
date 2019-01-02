
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


void calcPointfromPixel(wp3::Sensor::Ptr &world,
                        wp3::Sensor::Ptr &node,
                        std::string pointId,
                        std::vector<int> xVec, std::vector<int> yVec,
                        float &median,
                        float &mean,
                        float &standardDeviation,
                        float &standardError)
{
  float x, y, hres, vres, distance, error;
  std::vector<float> medianVec, meanVec, standardDeviationVec, standardErrorVec;
  Eigen::Matrix3d intrinsicMatrix = node->getIntrinsics_matrix();
  Eigen::Matrix4f transform_m4f;
  Eigen::Vector4f point;
  Eigen::Vector4f errorVec;
  point << 0,0,0,1;
  errorVec << 0,0,0,0;
  Eigen::Vector4f pointTrans;
  std::vector<cv::Mat> imageVec = node->getDepthMatVec();
  double depth_focal_inverted_x = 1/intrinsicMatrix(0,0);  // 1/fx
  double depth_focal_inverted_y = 1/intrinsicMatrix(1,1);  // 1/fy

  // get marker id from string
  std::string str = pointId.substr(0,2);
  std::stringstream strStream(str);
  int id=0;
  strStream >> id;

  std::map<int, Eigen::Vector4f> refPointMap;
  refPointMap[11] = 	(Eigen::Vector4f() << 2.0001506014,	 2.0005232588,  0.0017341049, 0.0 ).finished();
  refPointMap[12] = 	(Eigen::Vector4f() << 2.0003996957,	 5.2625767383, -0.0005132107, 0.0 ).finished();
  refPointMap[13] = 	(Eigen::Vector4f() << 1.9990031637,	 8.6987815688,  0.0002499338, 0.0 ).finished();
  refPointMap[21] = 	(Eigen::Vector4f() << 4.7121465682,	 1.9984242284, -0.0022354344, 0.0 ).finished();
  refPointMap[22] = 	(Eigen::Vector4f() << 4.6938289136,	 5.2993518722,  0.0030167305, 0.0 ).finished();
  refPointMap[23] = 	(Eigen::Vector4f() << 4.2808275916,	 9.2044133825,  0.0038446565, 0.0 ).finished();
  refPointMap[31] = 	(Eigen::Vector4f() << 5.2525662086,	 2.0006498535, -0.0012827744, 0.0 ).finished();
  refPointMap[32] = 	(Eigen::Vector4f() << 5.2751022105,	 5.2885478721,  0.0022119397, 0.0 ).finished();
  refPointMap[33] = 	(Eigen::Vector4f() << 5.4173080345,	 9.2354240626,  0.0067583546, 0.0 ).finished();
  refPointMap[41] = 	(Eigen::Vector4f() << 6.9990322663,	 1.9993571881,  0.0020473808, 0.0 ).finished();
  refPointMap[42] = 	(Eigen::Vector4f() << 8.1508440177,	 5.9499631486,  0.3653026523, 0.0 ).finished();
  refPointMap[43] = 	(Eigen::Vector4f() << 8.1454383538,	 7.9510350749,  0.3649493502, 0.0 ).finished();
  refPointMap[44] = 	(Eigen::Vector4f() << 6.9962809594,	10.3806603856,  0.0034262368, 0.0 ).finished();

//  std::cout << id << ":\n" << refPoints.find(id)->second << '\n';
  Eigen::Vector4f refPoint = refPointMap.find(id)->second;

  for(int i=0 ; i< xVec.size() ; i++)
  {
    x = static_cast<float>(xVec[i]);
    y = static_cast<float>(yVec[i]);
    median=0;
    mean=0;

    float sum=0;
    int count=0;
    std::vector<float> depthValueVec, depthValueVecFiltered;
    standardDeviation=0;

    for(size_t k=0; k<imageVec.size(); k++)        // k images
    {
      depthValueVec.push_back(imageVec[k].at<float>(y,x));
    }

    // calc mean and stdDev
    median = wp3::calcMedian(depthValueVec);
    medianVec.push_back(median);

    for(size_t k=0; k<imageVec.size(); k++)        // k images
    {
      //        float depthValue = imageVec[k].at<float>(y,x);
      if(depthValueVec[k] > median-0.2 && depthValueVec[k] < median+0.2)
      {
        depthValueVecFiltered.push_back( depthValueVec[k] );
        sum += depthValueVec[k];
        count++;
      }
    } // k images
    mean = sum / count;
    meanVec.push_back(mean);
    for(int i = 0; i < depthValueVecFiltered.size(); i++)
      standardDeviation += pow(depthValueVecFiltered[i] - mean, 2);
    standardDeviation = sqrt(standardDeviation / (depthValueVecFiltered.size()-1)); // s^2/N-1 for unbiased standard dev
    standardDeviationVec.push_back(standardDeviation);

    // https://en.wikipedia.org/wiki/Standard_error
    standardError = standardDeviation / sqrt(depthValueVecFiltered.size());
    standardErrorVec.push_back(standardError);

  }

  median = std::accumulate( medianVec.begin(), medianVec.end(), 0.0)/medianVec.size();
  mean = std::accumulate( meanVec.begin(), meanVec.end(), 0.0)/meanVec.size();
  standardDeviation = std::accumulate( standardDeviationVec.begin(), standardDeviationVec.end(), 0.0)/standardDeviationVec.size();
  standardError = std::accumulate( standardErrorVec.begin(), standardErrorVec.end(), 0.0)/standardErrorVec.size();

  x = std::accumulate( xVec.begin(), xVec.end(), 0.0)/xVec.size();
  y = std::accumulate( yVec.begin(), yVec.end(), 0.0)/yVec.size();


  // calc point
  point(0) = (x - intrinsicMatrix(0,2)) * median * depth_focal_inverted_x;
  point(1) = (y - intrinsicMatrix(1,2)) * median * depth_focal_inverted_y;
  point(2) = median;


  // euclidean distance from sensor
  distance = sqrt( point(0)*point(0)+point(1)*point(1)+point(2)*point(2) ); // exclude point(3)

  //transform to world coordinates
  transform_m4f =
      node->transArucoToICP_ // pose ICP refined [i]
      * world->transCamToAruco_ // pose avg aruco [ref]
      * node->transCamToAruco_.inverse(); // pose Cam [i]

  pointTrans = transform_m4f*point;
  //      std::cout << "pointWorld:\n" << pointWorld << std::endl;

  // errors
  errorVec = pointTrans-refPoint;
  errorVec(3) = 0.0;
  error = errorVec.norm();

  const int FRAME_WIDTH = 512;
  const int FRAME_HEIGHT = 424;
  const float FOV_HORIZONTAL = 70 * PI / 180.0; // convert to radians //70.6 deg?
  const float FOV_VERTICAL = 60.0 * PI / 180.0;   // convert to radians
  const float HORIZONTAL_SCALING = 2 * std::tan(FOV_HORIZONTAL / 2.0) / (float)FRAME_WIDTH;
  const float VERTICAL_SCALING = 2 * std::tan(FOV_VERTICAL / 2.0) / (float)FRAME_HEIGHT;

  hres = HORIZONTAL_SCALING * median;
  vres = VERTICAL_SCALING * median;

  //      std::cout << "\nNode\tId\tx\ty\tmedian\tmean\tstdDev\tWx\tWy\tWz\n"
  //                   "=======================================================================================================\n";
  ROS_INFO_STREAM(node->name_ << "\t" << pointId<< "\t" << x << "\t" << y << "\t" << median << "\t" << mean << "\t" << standardDeviation << "\t" << standardError
                  << "\t" << point(0) << "\t" << point(1) << "\t" << point(2) << "\t" << distance
                  << "\t" << pointTrans(0) << "\t" << pointTrans(1) << "\t" << pointTrans(2) << "\t"
                  << "\t" << errorVec(0) << "\t" << errorVec(1) << "\t" << errorVec(2) << "\t" << error << "\t"
                  << hres << "\t" << vres );
}

void doVerification(std::vector<wp3::Sensor::Ptr> &sensorVec, wp3::Sensor::Ptr &worldSensor)
{
  float median, mean, standardDeviation, standardError;
  ROS_INFO_STREAM("Node\tId\tx\ty\tmedian\tmean\tstdDev\t\tSEM\t\tCx\tCy\tCz\tCdistance\tWx\tWy\tWz\tEx\tEy\tEz\tEd\tHres\tVres\n"
               "=======================================================================================================");
  // Jetson1
  calcPointfromPixel(worldSensor, sensorVec[0], "11",  {61}, {175}, median, mean, standardDeviation, standardError);
  calcPointfromPixel(worldSensor, sensorVec[0], "21",  {160},{255}, median, mean, standardDeviation, standardError);
  calcPointfromPixel(worldSensor, sensorVec[0], "22*", {295,296}, {146,147}, median, mean, standardDeviation, standardError);
  calcPointfromPixel(worldSensor, sensorVec[0], "31*", {186,185}, {275,276}, median, mean, standardDeviation, standardError);
  calcPointfromPixel(worldSensor, sensorVec[0], "32",  {321}, {159}, median, mean, standardDeviation, standardError);
  calcPointfromPixel(worldSensor, sensorVec[0], "41",  {288}, {359}, median, mean, standardDeviation, standardError);
  // Jetson2
  calcPointfromPixel(worldSensor, sensorVec[1], "11",  {191}, {349}, median, mean, standardDeviation, standardError);
  calcPointfromPixel(worldSensor, sensorVec[1], "12",  { 57}, {195}, median, mean, standardDeviation, standardError);
  calcPointfromPixel(worldSensor, sensorVec[1], "21",  {338}, {210}, median, mean, standardDeviation, standardError);
  calcPointfromPixel(worldSensor, sensorVec[1], "22",  {192}, {113}, median, mean, standardDeviation, standardError);
  calcPointfromPixel(worldSensor, sensorVec[1], "31",  {360}, {189}, median, mean, standardDeviation, standardError);
  calcPointfromPixel(worldSensor, sensorVec[1], "32",  {216}, { 99}, median, mean, standardDeviation, standardError);
  calcPointfromPixel(worldSensor, sensorVec[1], "41",  {421}, {132}, median, mean, standardDeviation, standardError);
  // Jetson3                                               }  {   }
  calcPointfromPixel(worldSensor, sensorVec[2], "11",  { 98}, { 51}, median, mean, standardDeviation, standardError);
//  calcPointfromPixel(worldSensor, sensorVec[2], "13",  {406}, { 48}, median, mean, standardDeviation, standardError); // to far?
  calcPointfromPixel(worldSensor, sensorVec[2], "21",  { 60}, {137}, median, mean, standardDeviation, standardError);
  calcPointfromPixel(worldSensor, sensorVec[2], "22*", {253,252}, {134,135}, median, mean, standardDeviation, standardError);
  calcPointfromPixel(worldSensor, sensorVec[2], "23",  {468}, {116}, median, mean, standardDeviation, standardError);
  calcPointfromPixel(worldSensor, sensorVec[2], "31",  { 51}, {159}, median, mean, standardDeviation, standardError);
  calcPointfromPixel(worldSensor, sensorVec[2], "32*", {253,252}, {158,159}, median, mean, standardDeviation, standardError);
  calcPointfromPixel(worldSensor, sensorVec[2], "41",  { 10},  {253}, median, mean, standardDeviation, standardError);
//  calcPointfromPixel(worldSensor, sensorVec[2], "42",  {318}, {325}, median, mean, standardDeviation, standardError); // too reflective
  calcPointfromPixel(worldSensor, sensorVec[2], "42**",  {314,322}, {325,325}, median, mean, standardDeviation, standardError); //use white edges (not center)
  calcPointfromPixel(worldSensor, sensorVec[2], "43",  {500}, {322}, median, mean, standardDeviation, standardError);
  // Jetson4                                               }  {   }
  calcPointfromPixel(worldSensor, sensorVec[3], "12*", {232,232}, {312,313}, median, mean, standardDeviation, standardError);
  calcPointfromPixel(worldSensor, sensorVec[3], "21*", {440,441}, {152,152}, median, mean, standardDeviation, standardError);
  calcPointfromPixel(worldSensor, sensorVec[3], "22*", {238,238}, {149,150}, median, mean, standardDeviation, standardError);
  calcPointfromPixel(worldSensor, sensorVec[3], "31*", {432,432}, {128,129}, median, mean, standardDeviation, standardError);
  calcPointfromPixel(worldSensor, sensorVec[3], "32*", {240,239}, {124,125}, median, mean, standardDeviation, standardError);
  calcPointfromPixel(worldSensor, sensorVec[3], "33",  { 14}, {115}, median, mean, standardDeviation, standardError);
  calcPointfromPixel(worldSensor, sensorVec[3], "41",  {409}, { 66}, median, mean, standardDeviation, standardError);
  // Jetson5                                               }  {   }
  calcPointfromPixel(worldSensor, sensorVec[4], "13*", {447}, {120}, median, mean, standardDeviation, standardError);
//  calcPointfromPixel(worldSensor, sensorVec[4], "21",  {106}, { 66}, median, mean, standardDeviation, standardError); // poor measurement
  calcPointfromPixel(worldSensor, sensorVec[4], "22",  {220}, {122}, median, mean, standardDeviation, standardError);
//  calcPointfromPixel(worldSensor, sensorVec[4], "23",  {431}, {204}, median, mean, standardDeviation, standardError); // misaligned color
  calcPointfromPixel(worldSensor, sensorVec[4], "23",  {433}, {202}, median, mean, standardDeviation, standardError); // actual center of marker
//  calcPointfromPixel(worldSensor, sensorVec[4], "31",  { 86}, { 76}, median, mean, standardDeviation, standardError); // poor measurement
  calcPointfromPixel(worldSensor, sensorVec[4], "32",  {198}, {137}, median, mean, standardDeviation, standardError);
  calcPointfromPixel(worldSensor, sensorVec[4], "42*", { 76,76}, {250,251}, median, mean, standardDeviation, standardError);
//  calcPointfromPixel(worldSensor, sensorVec[4], "43",  {196}, {350}, median, mean, standardDeviation, standardError); // too reflective
  calcPointfromPixel(worldSensor, sensorVec[4], "43**", {192,200}, {348,350}, median, mean, standardDeviation, standardError); // use white corners
  // Jetson6                                               }  {   }
  calcPointfromPixel(worldSensor, sensorVec[5], "12",  {432}, {235}, median, mean, standardDeviation, standardError);
  calcPointfromPixel(worldSensor, sensorVec[5], "13",  {221}, {361}, median, mean, standardDeviation, standardError);
//  calcPointfromPixel(worldSensor, sensorVec[5], "21",  {436}, { 89}, median, mean, standardDeviation, standardError); // too far
  calcPointfromPixel(worldSensor, sensorVec[5], "22",  {315}, {138}, median, mean, standardDeviation, standardError);
  calcPointfromPixel(worldSensor, sensorVec[5], "23",  {115}, {239}, median, mean, standardDeviation, standardError);
  calcPointfromPixel(worldSensor, sensorVec[5], "31",  {417}, { 79}, median, mean, standardDeviation, standardError); // rough
  calcPointfromPixel(worldSensor, sensorVec[5], "33",  { 92}, {191}, median, mean, standardDeviation, standardError);

  // Refpoints
  calcPointfromPixel(worldSensor, sensorVec[3], "Refl.", {308}, {220}, median, mean, standardDeviation, standardError);
  calcPointfromPixel(worldSensor, sensorVec[3], "Paper", {290}, {220}, median, mean, standardDeviation, standardError);
  calcPointfromPixel(worldSensor, sensorVec[3], "Robot", {300}, {220}, median, mean, standardDeviation, standardError);
  calcPointfromPixel(worldSensor, sensorVec[3], "Refl.2", {225}, {65}, median, mean, standardDeviation, standardError);
  calcPointfromPixel(worldSensor, sensorVec[3], "Paper2", {230}, {65}, median, mean, standardDeviation, standardError);
  calcPointfromPixel(worldSensor, sensorVec[3], "Robot2", {225}, {65}, median, mean, standardDeviation, standardError);
}

int main (int argc, char** argv)
{
  //Set logging level: (DEBUG, INFO, WARN, ERROR, FATAL)
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }

  std::recursive_mutex r_mutex2;
  boost::mutex mtx;

  // init cloud vectors for visualizer
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vector_1, cloud_vector_2, cloud_vector_3, cloud_vector_4, cloud_vector_5, cloud_vector_6, worldClouds;

  // init transforms
  Eigen::Matrix4f transform_A = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f transform_B = Eigen::Matrix4f::Identity();
  //  std::map<float, Eigen::Matrix4f> transformMap_A;
  //  std::map<float, Eigen::Matrix4f> transformMap_B;
  MarkerMapType transformMap_A;
  MarkerMapType transformMap_B;

  Eigen::Matrix4f transform_m4f;
  Eigen::Matrix4d transform_m4d;
  Eigen::Affine3d transform_affine;
  tf::Transform transform_tf;
  std::vector<int> markerIds;
  std::string fromString, toString;

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
  wp3::Visualization viewerWorld("Viewer World Map");
  viewerWorld.initializeSingle();
  wp3::Visualization viewer("Viewer All");
  viewer.initialize();
#endif
#ifdef VIEW_ICP
  wp3::Visualization viewerICP("Viewer ICP steps");
  viewerICP.initializeSingle();
#endif

  std::map<std::string, Eigen::Matrix4f> transMap, worldTransMap;

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

  //init world as a sensor
  wp3::Sensor::Ptr worldSensor = boost::make_shared<wp3::Sensor>("world", node_handle);
  worldSensor->name_ = "world";

  //world
  wp3::arucoProcessor worldAruco;
  Eigen::Matrix4f tf_worldToAvg;
  MarkerMapType worldMarkerMap;
  pcl::PointCloud<pcl::PointXYZ>::Ptr worldCloud(new pcl::PointCloud<pcl::PointXYZ>);

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
    key_cv = cv::waitKey(100);

//    ROS_INFO_STREAM("\nn to calibrate\nq to quit");
//    cin >> key;
//    if(key=='q') break;


    if(init)
    {
      // TULL TEST CREATE ARUCO MARKER IMAGE ----------------------------------
//      init = false;

//      Eigen::Vector3d T(6.5136, 5.9499, 0.308); // translation X,Y,Z
      Eigen::Vector3d T(6.485, 5.9453, 0.310); // translation X,Y,Z
      Eigen::Vector3d R(0.0, 0.0, 0.0); // rotation euler Z, Y, X
      worldAruco.simulateMarker(1,R,T);

//      T = Eigen::Vector3d(6.5171, 3.9527, 0.310); // translation X,Y,Z
      T = Eigen::Vector3d(6.485, 3.959, 0.310); // translation X,Y,Z
      R = Eigen::Vector3d(0.0, 0.0, 0.0); // rotation euler Z, Y, X
      worldAruco.simulateMarker(13,R,T);

//      T = Eigen::Vector3d(3.3322, 3.9618, 0.310); // translation X,Y,Z
      T = Eigen::Vector3d(3.283, 3.958, 0.310); // translation X,Y,Z
      R = Eigen::Vector3d(0.0, 0.0, 0.0); // rotation euler Z, Y, X
      worldAruco.simulateMarker(40,R,T);

      worldAruco.getAverageTransformation(tf_worldToAvg, worldMarkerMap);
      worldCloud = worldAruco.getCroppedCloud();
      worldSensor->cloud1CrPtr_ = worldCloud;
      worldSensor->cloud1Ptr_ = worldCloud;
      worldSensor->transCamToAruco_ = tf_worldToAvg;
      worldSensor->setTfTopic("world");

      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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
#ifdef VIEW_ICP
//      wp3::calcTransMats(*sensorVec[0], *sensorVec[1], transMat_avgA, transMat_avgB, transform_reference_global, transform_ICP1_print, ICP1_fitness_to_print, viewerICP);
#else
      ROS_INFO_STREAM("Performing ICP refinement between sensors and reference map"); // << sensorVec[i]->name_ << " and " << sensorVec[2]->name_);
      for(int i=0 ; i<num_sensors ; i++)
      {
//        if(i==2) continue; // skip reference node
//        wp3::refineTransformation(*sensorVec[i], *sensorVec[2]);
//        threadGroup.create_thread(boost::bind(wp3::refineTransformation, boost::ref(*sensorVec[i]), boost::ref(*sensorVec[2]) ));
        threadGroup.create_thread(boost::bind(wp3::refineTransformation, boost::ref(*sensorVec[i]), boost::ref(*worldSensor) ));
      }
      threadGroup.join_all();
#endif
      ROS_INFO_STREAM("All ICP refinement complete");

      // TULL
//      for(int i=0 ; i<num_sensors ; i++)
//      {
////        transform_m4f =
////            worldSensor->transCamToAruco_ // pose avg aruco [ref]
////            * sensorVec[i]->transCamToAruco_.inverse() // pose Cam [i]
////            * sensorVec[i]->transArucoToICP_; // pose ICP refined [i]
//        transform_m4f = sensorVec[i]->transArucoToICP_ * camPoseVec[i]; // pose ICP refined [i]
////        transform_m4f = sensorVec[i]->transArucoToICP_; // pose ICP refined [i]
//        pcl::transformPointCloud (*sensorVec[i]->cloudCrPtr_, *sensorVec[i]->cloud2CrPtr_, transform_m4f);
//        pcl::transformPointCloud (*sensorVec[i]->cloudPtr_, *sensorVec[i]->cloud2Ptr_, transform_m4f);
//      }
      // TULL END




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
      viewerWorld.runSingle(worldClouds, worldTransMap);
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

      // check results of known markers
      doVerification(sensorVec, worldSensor);


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
//    tf::Transform worldToReference_tf; //(rot,trans)
//    tf::Quaternion q;
//    worldToReference_tf.setOrigin( tf::Vector3(7.6, 0.7, 4.2) ); // j1
//    worldToReference_tf.setOrigin( tf::Vector3(0.1, 4.8, 4.2) ); // j4
//    q.setRPY(-2.44, 0, 0.7);  // j1
//    q.setRPY(-2.50, 0, -1.571);  // j4
//    worldToReference_tf.setRotation(q);
//    transform_publisher.sendTransform(tf::StampedTransform(worldToReference_tf, ros::Time::now(), "world", sensorVec[0]->getTfTopic() ) );
//    transform_publisher.sendTransform(tf::StampedTransform(worldToReference_tf, ros::Time::now(), "world", "jetson1_ir_optical_frame" ) );


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


      // sensor.marker.avg to sensor.origin
//      transform_m4f = sensorVec[i]->transCamToAruco_.inverse(); // inverse to get aruco to cam
//      transform_m4d = transform_m4f.cast<double>();
//      transform_affine = Eigen::Affine3d(transform_m4d);
//      tf::transformEigenToTF(transform_affine, transform_tf);
////      fromString = worldSensor->name_ + ".avg";
//      fromString = sensorVec[i]->name_ + ".icp";
//      toString = sensorVec[i]->getTfTopic();
//      transform_publisher.sendTransform(tf::StampedTransform(transform_tf, ros::Time::now(), fromString, toString ) );


//      if(i==2) continue; // skip rest if reference node
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

//      transform_m4f = sensorVec[i]->transCamToAruco_.inverse(); // pose Cam [i]
//      transform_m4d = transform_m4f.cast <double>();
//      transform_affine = Eigen::Affine3d(transform_m4d);
//      tf::transformEigenToTF(transform_affine, transform_tf);
//      fromString = sensorVec[i]->name_ + ".ICP";
////      fromString = worldSensor->name_ + ".avg";
//      toString = sensorVec[i]->getTfTopic();
//      transform_publisher.sendTransform(tf::StampedTransform(transform_tf, ros::Time::now(), fromString, toString ) );

      //    apVec[i].getAverageTransformation(transAvgVec[i], transMapUsedVec[i]);
      //    camPoseVec[i] = tf_worldToAvg * transAvgVec[i].inverse();
      //    sensorVec[i]->transCamToAruco_ = transAvgVec[i];

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
    viewerWorld.update();
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
