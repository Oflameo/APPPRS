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
    state.push_back(0);
    state.push_back(0);
    state.push_back(0);
}

single_particle::~single_particle() {
}

single_particle::single_particle(std::vector<float> initialState) {
    weight = 1;
    state.swap(initialState);
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

float single_particle::laserMeasurement(std::vector<float> laserRange) {
    return 0;
}


void single_particle::move(std::vector<float> odometry) {

}
