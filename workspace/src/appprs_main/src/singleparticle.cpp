/*
 * singleparticle.cpp
 *
 *  Created on: Nov 16, 2015
 *      Author: Jamie
 */

#include <appprs_main/singleparticle.h>
#include <math.h>


single_particle::single_particle() {
    weight = 1;
    state.resize(3);
}

single_particle::~single_particle() {
}

single_particle::single_particle(std::vector<float> initialState) {
    weight = 1;
    state.resize(3);
    //state.swap(initialState);
    //std::cout << "new particle @ (x,y) = " << state.at(0) << "," << state.at(1) << std::endl;
}

float single_particle::getX() const {
    return state.at(0);
}
float single_particle::getY() const {
    return state.at(1);
}
float single_particle::getTh() const {
    return state.at(2);
}
float single_particle::getWeight() const {
    return weight;
}
std::vector<float> single_particle::getState() {
    return state;
}

void single_particle::setX(float x) {
    state.at(0) = x;
}
void single_particle::setY(float y) {
    state.at(1) = y;
}
void single_particle::setTh(float th) {
    state.at(2) = th;
}
void single_particle::setMapImage(cv::Mat &map_image_in) {
    map_image = map_image_in;
}

uchar single_particle::queryMapImage(float x, float y) {
    return map_image.at<uchar>(MAP_SIZE-1-y*MAP_RESOLUTION,x*MAP_RESOLUTION);
}



float single_particle::laserMeasurement(std::vector<float> laserRange) {

    std::vector<float> laserPrediction = laserCast();

    return 0;
}


void single_particle::move(std::vector<float> movement) {
    for (uint i = 0; i < 3; i++) {
        state.at(i) += movement.at(i);
    }
}

Eigen::MatrixXd single_particle::create2DHomogeneousTransform(std::vector<float> x_y_th) {
    float x, y, th;
    x = x_y_th.at(0);
    y = x_y_th.at(1);
    th = x_y_th.at(2);
    Eigen::MatrixXd output;
    output.resize(3,3);
    output(0,0) = cos(th); output(0,1) = sin(th); output(0,2) = x;
    output(1,0) = -sin(th); output(1,1) = cos(th); output(1,2) = y;
    output(2,0) = 0; output(2,1) = 0; output(2,2) = 1;
    return output;
}

std::vector<float> single_particle::laserCast() {
    // return 180 doubles that indicate the range at which a wall is predicted
    std::vector<double> result;
    result.resize(180);
    for (uint i = 0; i < result.size(); i++) {
        float thi = (float)i;
        Eigen::MatrixXd laserFrameRay = Eigen::MatrixXd::Ones(3,DENSITY_ALONG_RAY);
        laserFrameRay.row(0) = Eigen::VectorXd::LinSpaced(DENSITY_ALONG_RAY,0.0,RANGE_MAX*cos(thi));
        laserFrameRay.row(1) = Eigen::VectorXd::LinSpaced(DENSITY_ALONG_RAY,0.0,RANGE_MAX*sin(thi));
    }
}
