#include <appprs_main/particlefilter.h>

ParticleFilter::ParticleFilter()
{

}

ParticleFilter::~ParticleFilter()
{

}

void ParticleFilter::odometry(std::vector<double> newOdometry) {

    lastOdometry.swap(newOdometry);
}

void ParticleFilter::laser() {

}

void ParticleFilter::resample() {

}

void ParticleFilter::normalizeParticleWeights() {

}
