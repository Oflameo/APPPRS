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
void single_particle::setWeight(float new_weight) {
	weight=new_weight;
}
void single_particle::setMapImage(cv::Mat &map_image_in) {
    map_image = map_image_in;
}

uchar single_particle::queryMapImage(float x, float y) {
    //std::cout << "x = " << round(MAP_SIZE-1-y*MAP_RESOLUTION) << "   y = " << round(x*MAP_RESOLUTION) << std::endl;
    if (round(MAP_SIZE-1-y*MAP_RESOLUTION) < 0 ||
        round(MAP_SIZE-1-y*MAP_RESOLUTION) > MAP_SIZE-1 ||
        round(x*MAP_RESOLUTION) < 0 ||
        round(x*MAP_RESOLUTION) > MAP_SIZE-1)
    {
        //std::cout << "queried outside of map, returning zero" << std::endl;
        return 0;
    }
    return map_image.at<uchar>(round(MAP_SIZE-1-y*MAP_RESOLUTION),round(x*MAP_RESOLUTION));
}



void single_particle::laserMeasurement(std::vector<float> laserRanges, std::vector<float> laserWRTMap) {
    std::vector<float> results;
    results.resize(180);
    float rangeErrorSum = 0;

    float state_x = state.at(0);
    float state_y = state.at(1);
    float state_th = state.at(2);

#pragma omp parallel for schedule(static,1) num_threads(8)
    for (int i = 0; i < 180; i++) {
        float thi = i*PI/180.0;
        float laserRange = laserRanges.at(i);
        for (int j = 0; j < DENSITY_ALONG_RAY; j++) {
            float a = j*RANGE_MAX/DENSITY_ALONG_RAY;
            float x = state_x + 0.25*cos(state_th) + a*cos(thi + state_th);
            float y = state_y + 0.25*sin(state_th) + a*sin(thi + state_th);

            int mapValue = queryMapImage(x,y);
            //std::cout << "j = " << j << " a = " << a << " map at (" << x << "," << y << ") = " << mapValue << std::endl;

            //std::this_thread::sleep_for(std::chrono::milliseconds(20));

            if (mapValue < 250) {
                //std::cout << "thi = " << thi << " measurement range = " << laserRanges.at(i)/ODOMETRY_RESOLUTION << "   predicted range = " << a << std::endl;
                float result = pow(a - laserRange/ODOMETRY_RESOLUTION,2);
                results.at(i) = result;
                rangeErrorSum += result;
                break;
            }
        }
    }
    //std::cout << "\nparticle id " << id << " weight before laser udpate = " << weight;
    weight *= exp(-1*rangeErrorSum/LASER_UNCERTAINTY_SCALAR);
    //std::cout << "\nparticle id " << id << " weight after laser udpate = " << weight << std::endl;
}

void single_particle::weightCrush() {
    // check for certain criteria that make a particle completely invalid
    if (state.at(0) < 0 ||
        state.at(0) > MAP_SIZE/MAP_RESOLUTION ||
        state.at(1) < 0 ||
        state.at(1) > MAP_SIZE/MAP_RESOLUTION)
    {
        //weight *= 0.001;
    }
    if (queryMapImage(state.at(0),state.at(1)) < 200) {
        //weight *= 0.001;
    }
}


void single_particle::move(std::vector<float> movement) {
    for (uint i = 0; i < 3; i++) {
        state.at(i) += movement.at(i);
    }
    weightCrush();
}

void single_particle::setId(int newId) {
    id = newId;
}
int single_particle::getId() {
    return id;
}


