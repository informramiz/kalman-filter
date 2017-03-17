//
//  Tracking.cpp
//  kalmanFilter
//
//  Created by Ramiz Raja on 3/17/17.
//  Copyright © 2017 Ramiz Raja. All rights reserved.
//

#include <iostream>
#include <cmath>

#include "Eigen/Dense"
#include "Tracking.hpp"

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tracking::Tracking() {
    isInitialized_ = false;
    previousTimestamp_ = 0;
    
    //create a 4D state vector, we don't know yet the values of the x state
    kalmanFilter_.x_ = Eigen::VectorXd(4);
    
    //state covariance matrix P
    kalmanFilter_.P_ = Eigen::MatrixXd(4, 4);
    kalmanFilter_.P_ << 1, 0, 0, 0,
                        0, 1, 0, 0,
                        0, 0, 1000, 0,
                        0, 0, 0, 1000;
    
    //measurement covariance/noise/uncertainty matrix R
    kalmanFilter_.R_ = Eigen::MatrixXd(2, 2);
    kalmanFilter_.R_ << 0.0255, 0,
                        0, 0.0255;
    
    //measurement matrix
    kalmanFilter_.H_ = Eigen::MatrixXd(2, 4);
    kalmanFilter_.H_ << 1, 0, 0, 0,
                        0, 1, 0, 0;
    
    //the state transition vector F in initial form
    kalmanFilter_.F_ = Eigen::MatrixXd(4, 4);
    kalmanFilter_.F_ << 1, 0, 1, 0,
                        0, 1, 0, 1,
                        0, 0, 1 , 0,
                        0, 0, 0, 1;
    
    //process covariance matrix Q, we don't know values of deltaT yet
    kalmanFilter_.Q_ = Eigen::MatrixXd(4, 4);
    kalmanFilter_.Q_ << 1, 0, 1, 0,
                        0, 1, 0, 1,
                        1, 0, 1, 0,
                        0, 1, 0, 1;
    noiseAx_ = 5;
    noiseAy_ = 5;
}

Tracking::~Tracking() {
    
}

//Process a single measurement
void Tracking::processMeasurement(const MeasurementPackage &measurement_pack) {
    if(!isInitialized_) {
        //set the location with initial location and zero velocity
        kalmanFilter_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
        
        previousTimestamp_ = measurement_pack.timestamp_;
        isInitialized_ = true;
        
        return;
    }
    
    //compute the time elapsed between the current and previous measurements in seconds
    float dt = (measurement_pack.timestamp_ - previousTimestamp_) / 1000000.0;
    previousTimestamp_ = measurement_pack.timestamp_;
    
    //1. Modify the F matrix so that the time is integrated
    kalmanFilter_.F_(0, 2) = dt;
    kalmanFilter_.F_(1, 3) = dt;
    
    //2. Set the process covariance matrix Q
    kalmanFilter_.Q_(0, 0) = (pow(dt, 4) / 4) * noiseAx_;
    kalmanFilter_.Q_(0, 2) = (pow(dt, 3) / 2) * noiseAx_;
    kalmanFilter_.Q_(1, 1) = (pow(dt, 4) / 4) * noiseAy_;
    kalmanFilter_.Q_(1, 3) = (pow(dt, 3) / 2) * noiseAy_;
    kalmanFilter_.Q_(2, 0) = (pow(dt, 3) / 2) * noiseAx_;
    kalmanFilter_.Q_(2, 2) = (pow(dt, 2) * noiseAx_);
    kalmanFilter_.Q_(3, 1) = (pow(dt, 3) / 2) * noiseAy_;
    kalmanFilter_.Q_(3, 3) = (pow(dt, 2) * noiseAy_);
    
    //3. Call the Kalman Filter predict() function
    kalmanFilter_.predict();
    //4. Call the Kalman Filter update() function
    // with the most recent raw measurements_
    kalmanFilter_.update(measurement_pack.raw_measurements_);
    
    std::cout << "x_= " << kalmanFilter_.x_ << std::endl;
    std::cout << "P_= " << kalmanFilter_.P_ << std::endl;
}


