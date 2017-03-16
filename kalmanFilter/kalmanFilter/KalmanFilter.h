//
//  KalmanFilter.h
//  kalmanFilter
//
//  Created by Ramiz Raja on 3/17/17.
//  Copyright © 2017 Ramiz Raja. All rights reserved.
//

#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter {
public:
    ///* state vector
    VectorXd x_;
    
    ///* state covariance matrix
    MatrixXd P_;
    
    ///* state transition matrix
    MatrixXd F_;
    
    ///* Process covariance matrix
    MatrixXd Q_;
    
    ///* measurement matrix
    MatrixXd H_;
    
    ///* measurement covariance matrix
    MatrixXd R_;
    
    /**
     * Constructor
     */
    KalmanFilter();
    
    /**
     * Destructor
     */
    virtual ~KalmanFilter();
    
    /**
     * Predicts the state and the state covariance matrix 
     * using the process model
     */
    void predict();
    
    /**
     * Update the state and state covariance based on 
     * @param z, the measurement at k+1
     */
    void update(const VectorXd &z);
};


#endif /* KALMAN_FILTER_H_ */
