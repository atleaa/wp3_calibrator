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


#include "wp3_calibrator/validation.h"

namespace wp3 {

//void calcPointfromPixel(wp3::Sensor::Ptr &world,
//wp3::Sensor::Ptr &node,
void calcPointfromPixel(boost::shared_ptr<Sensor> &world,
                        boost::shared_ptr<Sensor> &node,
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

void doValidation(std::vector<wp3::Sensor::Ptr> &sensorVec, wp3::Sensor::Ptr &worldSensor)
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

} // end namespace wp3
