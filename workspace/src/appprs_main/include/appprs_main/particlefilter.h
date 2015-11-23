#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

#include <vector>
#include <appprs_main/singleparticle.h>
#include "boost/shared_ptr.hpp"
#include <random>
#include "Eigen/Dense"

const int STEPS_PER_RESAMPLE = 10;
const int NUMBER_OF_PARTICLES = 1000;

class ParticleFilter {
    public:
        ParticleFilter();
        ~ParticleFilter();
        void odometry(std::vector<double> newOdometry);
        void laser();
        void resample();

    private:
        std::vector<double> lastOdometry;
        std::vector<boost::shared_ptr<single_particle>> ptr_container;
        void normalizeParticleWeights();
};

#endif // PARTICLEFILTER_H
