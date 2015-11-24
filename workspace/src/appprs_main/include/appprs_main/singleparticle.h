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

    //Output Functions
    float laserMeasurement(std::vector<float> laserRange);
    void move(std::vector<float> odometry);


private:
    float weight;
    std::vector<float> state;

};




#endif /* SINGLEPARTICLE_H_ */
