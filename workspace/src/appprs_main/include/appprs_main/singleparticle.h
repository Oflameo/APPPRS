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
    void setMapImage(cv::Mat &map_image_in);

    //Output Functions
    float laserMeasurement(std::vector<float> laserRanges);
    void move(std::vector<float> odometry);


private:
    uchar queryMapImage(float x, float y);
    std::vector<float> laserCast();
    Eigen::MatrixXd create2DHomogeneousTransform(std::vector<float> x_y_th);
    float weight;
    std::vector<float> state;
    cv::Mat map_image;

};




#endif /* SINGLEPARTICLE_H_ */
