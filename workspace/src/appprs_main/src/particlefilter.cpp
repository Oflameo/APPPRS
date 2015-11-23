#include <appprs_main/particlefilter.h>

ParticleFilter::ParticleFilter()
{

	std::string imageName("/home/jamie/workspace/ConvertToImage/src/wean_map_uint8.jpg"); // by default
	map_image=cv::imread(imageName,CV_LOAD_IMAGE_GRAYSCALE);

	//Check that you got the image
	if(!map_image.data )
	{
	    throw 0;
	}

}

ParticleFilter::~ParticleFilter()
{

}



void ParticleFilter::odometry(std::vector<double> newOdometry) {

	for(int i=0; i<ptr_container.size(); i++)
	{

	}

    lastOdometry.swap(newOdometry);
}

void ParticleFilter::laser() {

}

void ParticleFilter::resample() {

}

void ParticleFilter::normalizeParticleWeights() {

}

std::vector<boost::shared_ptr<single_particle> > ParticleFilter::getParticles() {
	return ptr_container;
}

int ParticleFilter::getStepsPerResample() {
	return STEPS_PER_RESAMPLE;
}

int ParticleFilter::getNumberOfParticles() {
	return NUMBER_OF_PARTICLES;
}
