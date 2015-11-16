/*
 * singleparticle.cpp
 *
 *  Created on: Nov 16, 2015
 *      Author: jamie
 */

#include "singleparticle.h"

single_particle::single_particle() {
	// TODO Auto-generated constructor stub

}

single_particle::~single_particle() {
	// TODO Auto-generated destructor stub
}

float single_particle::getX() const {
	return x;
}

float single_particle::getY() const {
	return y;
}

float single_particle::getTh() const {
	return theta;
}

float single_particle::getWeight() const {
	return weight;
}

void single_particle::setX(float var_x) {
	x=var_x;
}

void single_particle::setY(float var_y) {
    y=var_y;
}

void single_particle::setTh(float var_th) {
	theta=var_th;
}

void single_particle::setPosition(float var_x, float var_y, float var_Th) {
	x=var_x;
	y=var_y;
	theta=var_Th;
}

void single_particle::setWeight(float Weight) {
	weight=Weight;
}

float single_particle::evaluateLidar(float lidar[180]) {
}

void single_particle::evaluateOdometry(float dx, float dy, float dtheta, float dtime) {
	theta=theta+dtheta;

}
