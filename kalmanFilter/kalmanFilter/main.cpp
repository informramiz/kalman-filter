//
//  main.cpp
//  kalmanFilter
//
//  Created by Ramiz Raja on 3/15/17.
//  Copyright © 2017 Ramiz Raja. All rights reserved.
//

#include <iostream>
#include <vector>

#include "Eigen/Dense"

using namespace std;
using namespace Eigen;

//Kalman Filter Variables
VectorXd x;	// object state
MatrixXd P;	// object covariance matrix
VectorXd u;	// external motion
MatrixXd F; // state transition matrix
MatrixXd H;	// measurement matrix
MatrixXd R;	// measurement covariance matrix
MatrixXd I; // Identity matrix
MatrixXd Q;	// process covariance matrix

//measurements
vector<VectorXd> measurements;

void predict(VectorXd &x, MatrixXd &P);
void update(VectorXd &x, MatrixXd &P, VectorXd z);
void filter(VectorXd &x, MatrixXd &P);
void initMatrices();
void initMeasurements();

int main(int argc, const char * argv[]) {
    // insert code here...
    std::cout << "Hello, World!\n";
    
    initMatrices();
    initMeasurements();
    
    filter(x, P);
    cout << "Final x: " << endl << x << endl;
    cout << "Final P: " << endl << P << endl;
    
    return 0;
}

void initMeasurements() {
    VectorXd single_measurement(1);
    
    single_measurement << 1;
    measurements.push_back(single_measurement);
    
    single_measurement << 2;
    measurements.push_back(single_measurement);
    
    single_measurement << 3;
    measurements.push_back(single_measurement);
}

void initMatrices() {
    x = VectorXd(2);
    x << 0, 0;
    
    P = MatrixXd(2, 2);
    P << 1000, 0, 0, 100;
    
    u = VectorXd(2);
    u << 0, 0;
    
    F = MatrixXd(2, 2);
    F << 1, 1, 0, 1;
    
    H = MatrixXd(1, 2);
    H << 1, 0;
    
    R = MatrixXd(1, 1);
    R << 1;
    
    I = MatrixXd::Identity(2, 2);
    
    Q = MatrixXd(2, 2);
    Q << 0, 0, 0, 0;
}

void predict(VectorXd &x, MatrixXd &P) {
    x = F * x + u;
    P = F * P * F.transpose() +  Q;
}

void update(VectorXd &x, MatrixXd &P, VectorXd z) {
    VectorXd y = z - H * x;
    MatrixXd S = H * P * H.transpose() + R;
    MatrixXd K = P * H.transpose() * S.inverse();
    x = x + (K * y);
    P = (I - K * H) * P;
}

void filter(VectorXd &x, MatrixXd &P) {
    for (unsigned int i = 0; i < measurements.size(); ++i) {
        VectorXd z = measurements[i];
        // measurement update step
        update(x, P, z);
        // prediction step
        predict(x, P);
    }
}
