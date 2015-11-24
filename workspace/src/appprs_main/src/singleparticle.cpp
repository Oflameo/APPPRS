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
    std::vector<float> laserWRTRobot;
    laserWRTRobot.resize(3);
    laserWRTRobot.at(0) = 0.25;
    laserWRTRobot.at(1) = 0;
    laserWRTRobot.at(2) = 0;
    T_laserWRTRobot = create2DHomogeneousTransform(laserWRTRobot);
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
void single_particle::setLaserRays(std::vector<Eigen::MatrixXf> &laserFrameRaysInput) {
    laserFrameRays = boost::make_shared<std::vector<Eigen::MatrixXf>> (laserFrameRaysInput);
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



float single_particle::laserMeasurement(std::vector<float> laserRange) {

    std::vector<float> laserPrediction = laserCast();

    return 0;
}


void single_particle::move(std::vector<float> movement) {
    for (uint i = 0; i < 3; i++) {
        state.at(i) += movement.at(i);
    }
}

Eigen::MatrixXf single_particle::create2DHomogeneousTransform(std::vector<float> x_y_th) {
    float x, y, th;
    x = x_y_th.at(0);
    y = x_y_th.at(1);
    th = x_y_th.at(2);
    Eigen::MatrixXf output;
    output.resize(3,3);
    output(0,0) = cos(th); output(0,1) = sin(th); output(0,2) = x;
    output(1,0) = -sin(th); output(1,1) = cos(th); output(1,2) = y;
    output(2,0) = 0; output(2,1) = 0; output(2,2) = 1;
    return output;
}

std::vector<float> single_particle::laserCast() {
    // return 180 doubles that indicate the range at which a wall is predicted
    std::vector<float> result;
    result.resize(180);
    auto T_robotWRTMap = create2DHomogeneousTransform(state);
    auto T_laserWRTMap = T_robotWRTMap*T_laserWRTRobot;
    Eigen::MatrixXf laserWRTMap;
    for (uint i = 0; i < result.size(); i++) {
        // laserFrameRays->at(i) returns a 3xDENSITY_ALONG_RAY Eigen::MatrixXf for angle (float)i
        laserWRTMap = T_laserWRTMap*(laserFrameRays->at(i)); // transform all laser rays into the global frame
        for (uint j = 0; j < laserWRTMap.cols(); j++) {
            //std::cout << "x = " << laserWRTMap(0,j) << "  y = " << laserWRTMap(1,j) << std::endl;

            //queryMapImage(laserWRTMap(0,j), laserWRTMap(1,j));
            if (queryMapImage(laserWRTMap(0,j), laserWRTMap(1,j)) > 250) {
                result.at(i) = (float)j*RANGE_MAX/DENSITY_ALONG_RAY;
                break;
            }
        }

        //std::cout << laserWRTMap << std::endl;
    }
    return result;
}
