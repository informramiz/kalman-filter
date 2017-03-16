//
//  MeasurementPackage.h
//  kalmanFilter
//
//  Created by Ramiz Raja on 3/17/17.
//  Copyright © 2017 Ramiz Raja. All rights reserved.
//

#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

class MeasurementPackage {
public:
    long timestamp_;
    Eigen::VectorXd raw_measurements_;
    enum SensorType {
        LASER, RADAR
    } sensor_type_;
};

#endif /* MeasurementPackage_h */
