#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

#include <vector>
#include <appprs_main/singleparticle.h>
#include <random>
#include "Eigen/Dense"
#include <opencv2/opencv.hpp>
#include <string>
#include <fstream>
#include "boost/shared_ptr.hpp"
#include "boost/make_shared.hpp"
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/split.hpp>
#include <random>
#include <stdio.h>

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
        const int STEPS_PER_RESAMPLE = 10;
        const int NUMBER_OF_PARTICLES = 10000;
        const int MAP_SIZE = 800;
        const float MAP_RESOLUTION = 10.0;
        const float PI = 3.14159265359;

        Eigen::MatrixXd map;
        void initializeParticles();
        void normalizeParticleWeights();
        std::vector<double> lastOdometry;
        std::vector<boost::shared_ptr<single_particle>> particlesContainer;
        cv::Mat map_image;
};

#endif // PARTICLEFILTER_H
