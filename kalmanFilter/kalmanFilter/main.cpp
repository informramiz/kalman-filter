//
//  main.cpp
//  kalmanFilter
//
//  Created by Ramiz Raja on 3/15/17.
//  Copyright © 2017 Ramiz Raja. All rights reserved.
//

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include "Eigen/Dense"
#include "MeasurementPackage.h"
#include "Tracking.hpp"

using namespace std;
using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;


//measurements
vector<MeasurementPackage> readMeasurements();
void runKalmanFilter(const vector<MeasurementPackage> & measurement_pack_list);

int main(int argc, const char * argv[]) {
    vector<MeasurementPackage> measurement_pack_list = readMeasurements();
    runKalmanFilter(measurement_pack_list);
    return 0;
}

void runKalmanFilter(const vector<MeasurementPackage> & measurement_pack_list) {
    //Create a Tracking instance
    Tracking tracking;
    
    size_t size = measurement_pack_list.size();
    //call processMeasurement for each measurement
    for(size_t i = 0; i < size; ++i) {
        tracking.processMeasurement(measurement_pack_list[i]);
    }
}

vector<MeasurementPackage> readMeasurements() {
    
    vector<MeasurementPackage> measurement_pack_list;
    
    // hardcoded input file with laser and radar measurements
    string in_file_name = "obj_pose-laser-radar-synthetic-input.txt";
    ifstream inFile(in_file_name.c_str(), std::ifstream::in);
    
    if(!inFile.is_open()) {
        std::cout << "Can not open input file: " << in_file_name << std::endl;
        return measurement_pack_list;
    }
    
    // set i to get only first 3 measurments
    int i = 0;
    string line;
    while(getline(inFile, line) && (i <= 3)) {
        MeasurementPackage measurement_pack;
        string sensor_type;
        long timestamp;
        float x;
        float y;
        
        istringstream iss(line); //reads first element from the current line
        iss >> sensor_type;
        if (sensor_type.compare("L") == 0) { //laser measurement
            //read measurements
            measurement_pack.sensor_type_ = MeasurementPackage::LASER;
            measurement_pack.raw_measurements_ = Eigen::VectorXd(2);
            
            iss >> x;
            iss >> y;
            iss >> timestamp;
            
            measurement_pack.raw_measurements_ << x, y;
            measurement_pack.timestamp_ = timestamp;
            
            measurement_pack_list.push_back(measurement_pack);
        } else if(sensor_type.compare("R") == 0) { //Radar measurement
            //skip radar measurement
            continue;
        }
        
        i++;
    }
    
    if(inFile.is_open()) {
        inFile.close();
    }
    
    return measurement_pack_list;
}
