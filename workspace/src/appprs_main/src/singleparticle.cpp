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
    return 0;
}


void single_particle::move(std::vector<float> odometry) {

}
