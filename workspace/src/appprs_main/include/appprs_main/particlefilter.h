#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

#include <vector>
#include <appprs_main/singleparticle.h>
#include "boost/shared_ptr.hpp"
#include <random>
#include "Eigen/Dense"
#include <opencv2/opencv.hpp>
#include <string>


class ParticleFilter {
    public:
        ParticleFilter();
        ~ParticleFilter();
        void odometry(std::vector<double> newOdometry);
        void laser();
        void resample();
        std::vector<boost::shared_ptr<single_particle>> getParticles();
        int getStepsPerResample();
        int getNumberOfParticles();

    private:
        void normalizeParticleWeights();
        const int STEPS_PER_RESAMPLE = 10;
        const int NUMBER_OF_PARTICLES = 1000;
        std::vector<double> lastOdometry;
        std::vector<boost::shared_ptr<single_particle>> ptr_container;
    	cv::Mat map_image;
};

#endif // PARTICLEFILTER_H
