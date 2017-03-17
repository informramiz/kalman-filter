//
//  Tracking.hpp
//  kalmanFilter
//
//  Created by Ramiz Raja on 3/17/17.
//  Copyright © 2017 Ramiz Raja. All rights reserved.
//

#ifndef TRACKING_HPP_
#define TRACKING_HPP_

#include <vector>
#include <string>
#include <fstream>

#include "MeasurementPackage.h"
#include "KalmanFilter.h"

class Tracking {
public:
    Tracking();
    virtual ~Tracking();
    void processMeasurement(const MeasurementPackage & measurement_pack);
    KalmanFilter kalmanFilter_;
    
private:
    bool isInitialized_;
    long previousTimestamp_;
    
    // acceleration noise components
    float noiseAx_;
    float noiseAy_;
};

#endif /* Tracking_hpp */
