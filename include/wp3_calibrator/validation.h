#ifndef VALIDATION_H
#define VALIDATION_H

#include "wp3_calibrator/defines.h"
#include "wp3_calibrator/sensor.h"
#include "wp3_calibrator/functions.h"

namespace wp3
{
void calcPointfromPixel(boost::shared_ptr<Sensor> &world,
                        boost::shared_ptr<Sensor> &node,
                        std::string pointId,
                        std::vector<int> xVec, std::vector<int> yVec,
                        float &median,
                        float &mean,
                        float &standardDeviation,
                        float &standardError);
void doValidation(std::vector<wp3::Sensor::Ptr> &sensorVec, wp3::Sensor::Ptr &worldSensor);
} // end namespace wp3

#endif // VALIDATION_H
