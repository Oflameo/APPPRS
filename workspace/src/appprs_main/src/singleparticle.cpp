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
    single_particle();
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
    //std::cout << "x = " << round(MAP_SIZE-1-y*MAP_RESOLUTION) << "   y = " << round(x*MAP_RESOLUTION) << std::endl;
    if (round(MAP_SIZE-1-y*MAP_RESOLUTION) < 0 ||
        round(MAP_SIZE-1-y*MAP_RESOLUTION) > 799 ||
        round(x*MAP_RESOLUTION) < 0 ||
        round(x*MAP_RESOLUTION) > 799)
    {
        return 0;
    }
    return map_image.at<uchar>(round(MAP_SIZE-1-y*MAP_RESOLUTION),round(x*MAP_RESOLUTION));
}



void single_particle::laserMeasurement(std::vector<float> laserRanges, std::vector<float> laserWRTMap) {
    std::vector<float> results;
    results.resize(180);
    float rangeErrorSum = 0;
    for (int i = 0; i < 180; i++) {
        float thi = i*PI/180.0;
        for (float a = 0; a < RANGE_MAX; a += 1/MAP_RESOLUTION) {
            float x = laserWRTMap.at(0) + a*cos(thi + laserWRTMap.at(2));
            float y = laserWRTMap.at(1) + a*sin(thi + laserWRTMap.at(2));
            if (queryMapImage(x,y) < 250) {
                float result = pow(a - laserRanges.at(i),2);
                results.at(i) = result;
                rangeErrorSum += result;
                break;
            }
        }
    }
    weight *= exp(-1*rangeErrorSum/LASER_UNCERTAINTY_SCALAR);
}

void single_particle::weightCrush() {
    // check for certain criteria that make a particle completely invalid
    if (state.at(0) < 0 ||
        state.at(0) > MAP_SIZE/MAP_RESOLUTION ||
        state.at(1) < 0 ||
        state.at(1) > MAP_SIZE/MAP_RESOLUTION)
    {
        weight = 0;
    }
    if (queryMapImage(state.at(0),state.at(1)) < 200) {
        weight = 0;
    }
}


void single_particle::move(std::vector<float> movement) {
    for (uint i = 0; i < 3; i++) {
        state.at(i) += movement.at(i);
    }
    //weightCrush();
}
