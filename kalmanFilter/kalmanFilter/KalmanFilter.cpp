//
//  KalmanFilter.cpp
//  kalmanFilter
//
//  Created by Ramiz Raja on 3/17/17.
//  Copyright © 2017 Ramiz Raja. All rights reserved.
//

#include "KalmanFilter.h"

KalmanFilter::KalmanFilter() {
    
}

KalmanFilter::~KalmanFilter() {
    
}

void KalmanFilter::predict() {
    // state prediction
    x_ = F_ * x_;
    // covariance/uncertainty prediction
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::update(const VectorXd &z) {
    //predict what next measurement z should be
    VectorXd z_prediction = H_ * x_;
    //calculate the difference between
    // predicted and actual measurement
    VectorXd y = z - z_prediction;
    
    // Calculate Kalman Game
    MatrixXd S = H_ * P_ * H_.transpose() + R_;
    MatrixXd K = P_ * H_.transpose() * S.inverse();
    
    // update the state based on measurement
    x_ = x_ + K * y;
    // update the state covariance/uncertainty based on measurement
    MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
    P_ = (I - K * H_) * P_;
}
