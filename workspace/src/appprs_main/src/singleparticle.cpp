/*
 * singleparticle.cpp
 *
 *  Created on: Nov 16, 2015
 *      Author: Jamie
 */

#include <appprs_main/singleparticle.h>
#include <math.h>


single_particle::single_particle() {
	// TODO Auto-generated constructor stub
	x=0;
	y=0;
	theta=0;
	x_laz=0;
	y_laz=.25;
	weight=0;
}

single_particle::~single_particle() {
	// TODO Auto-generated destructor stub
}

single_particle::single_particle(float var_x, float var_y, float var_th) {
	x=var_x;
	y=var_y;
	theta=var_th;
	x_laz=x;
	y_laz=y+.25;
	weight=0;
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
	return 0;
}


void single_particle::evaluateOdometry(float dx, float dy, float dtheta, float dtime) {
	theta=theta+dtheta;
	x+=dx;
	y+=dy;
	x_laz=x-.25*sin(theta);
	y_laz=y+.25*cos(theta);

}
