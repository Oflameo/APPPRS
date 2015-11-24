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
#include <iostream>

class ParticleFilter {
    public:
        ParticleFilter();
        ~ParticleFilter();
        void odometry(std::vector<float> newOdometry);
        void laser();
        void resample();
        std::vector<boost::shared_ptr<single_particle>> getParticles();
        int getStepsUntilResample();
        int getNumberOfParticles();

    private:
        int stepsUntilResample;
        Eigen::MatrixXd map;
        void initializeParticles();
        void normalizeParticleWeights();
        std::vector<float> lastOdometry;
        std::vector<boost::shared_ptr<single_particle>> particlesContainer;
        cv::Mat map_image;
        //boost::shared_ptr<std::random_device> rd;
        boost::shared_ptr<std::mt19937> generator;
        boost::shared_ptr<std::normal_distribution<>> movementNoise;
        boost::shared_ptr<std::normal_distribution<>> bearingNoise;
};

#endif // PARTICLEFILTER_H
