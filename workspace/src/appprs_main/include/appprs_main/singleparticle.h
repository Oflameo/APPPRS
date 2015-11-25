/*
 * singleparticle.h
 *
 *  Created on: Nov 16, 2015
 *      Author: jamie
 */

#ifndef SINGLEPARTICLE_H_
#define SINGLEPARTICLE_H_

#include <vector>
#include "Eigen/Dense"
#include <iostream>
#include <opencv2/opencv.hpp>
#include "tunable_parameters.h"
#include <random>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <thread>

class single_particle {
public:
    //Default Constructor
    single_particle();
    //Default Destructor
    virtual ~single_particle();

    //Overload Constructor
    single_particle(std::vector<float> initialState);

    //Accessor Functions
    float getX() const;
    float getY() const;
    float getTh() const;
    float getWeight() const;
    std::vector<float> getState();

    //Mutator Functions
    void setX(float x);
    void setY(float y);
    void setTh(float th);
    void setWeight(float weight);
    void setMapImage(cv::Mat &map_image_in);
    //void setLaserRays(std::vector<Eigen::MatrixXf> &laserFrameRaysInput);

    //Output Functions
    void laserMeasurement(std::vector<float> laserRanges, std::vector<float> laserWRTMap);
    void move(std::vector<float> odometry);



private:
    uchar queryMapImage(float x, float y);
    void weightCrush();
    float weight;
    std::vector<float> state;
    cv::Mat map_image;    
};




#endif /* SINGLEPARTICLE_H_ */
