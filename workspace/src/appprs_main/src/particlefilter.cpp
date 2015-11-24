#include <appprs_main/particlefilter.h>

ParticleFilter::ParticleFilter()
{

    //std::string imageName("/home/jamie/workspace/ConvertToImage/src/wean_map_uint8.jpg"); // by default
    std::string imageName("/home/jazen/Documents/Classes/2015_Fall/16-831_Stats_in_Robotics/HW/HW_4/APPPRS/workspace/src/appprs_main/maps/wean_map_uint8.bmp"); // by default
    map_image=cv::imread(imageName,CV_LOAD_IMAGE_GRAYSCALE);

    //Check that you got the image
    if(!map_image.data )
    {
        std::cout << "Could not load map" << std::endl;
        throw 0;
    }

    // open map
    /*
    std::ifstream robotMap("/home/jazen/Documents/Classes/2015_Fall/16-831_Stats_in_Robotics/HW/HW_4/data/map/wean.dat");
    std::string mapLine;
    map.resize(MAP_SIZE,MAP_SIZE);
    if (robotMap.is_open()) {
        for (int i = 0; i < 7; i++) {
            getline(robotMap,mapLine); // skip 7 header lines
        }
        int mapRow = 0;
        while (getline(robotMap,mapLine)) {
            boost::trim(mapLine);
            std::vector<std::string> mapLineSplit;
            boost::split(mapLineSplit,mapLine,boost::is_any_of(" "),boost::token_compress_on);
            for (uint i = 0; i < mapLineSplit.size(); i++) {
                double mapValue = std::atof(mapLineSplit.at(i).c_str());
                map(mapRow,i) = mapValue;
            }
            mapRow++;
        }
    }
    robotMap.close();
    */

    initializeParticles();
    stepsUntilResample = STEPS_PER_RESAMPLE;

}

ParticleFilter::~ParticleFilter()
{

}

void ParticleFilter::initializeParticles() {
    std::random_device rd;
    std::mt19937 generator(rd());
    std::uniform_int_distribution<> xIndex(350,700);
    std::uniform_int_distribution<> yIndex(0,MAP_SIZE-1);
    std::uniform_real_distribution<> th(0,2.0*PI);

    while (particlesContainer.size() < NUMBER_OF_PARTICLES) {
        int xIdx = xIndex(generator);
        int yIdx = yIndex(generator);
        //std::cout << "map coord (x,y) = " << xIdx/MAP_RESOLUTION << ", " << yIdx/MAP_RESOLUTION << " image coord (x,y) = " << MAP_SIZE-1-yIdx << ", " << xIdx << std::endl;
        if (map_image.at<uchar>(MAP_SIZE-1-yIdx,xIdx) > 250) {
            single_particle particle;
            particle.setX(xIdx/MAP_RESOLUTION);
            particle.setY(yIdx/MAP_RESOLUTION);
            particle.setTh(th(generator));
            particle.setMapImage(map_image);
            auto p = boost::make_shared<single_particle> (particle);
            particlesContainer.push_back(p);
        }
    }
    std::cout << "finished initializing particles" << std::endl;
}


void ParticleFilter::odometry(std::vector<float> newOdometry) {

    for(int i = 0; i < particlesContainer.size(); i++) {

    }

    lastOdometry.swap(newOdometry);
}

void ParticleFilter::laser() {

    stepsUntilResample--;
    std::cout << "There are " << stepsUntilResample << " steps until resample" << std::endl;

}

void ParticleFilter::resample() {
    std::cout << "Resampling..." << std::endl;
    stepsUntilResample = STEPS_PER_RESAMPLE;
}

void ParticleFilter::normalizeParticleWeights() {

}

std::vector<boost::shared_ptr<single_particle> > ParticleFilter::getParticles() {
    return particlesContainer;
}

int ParticleFilter::getStepsUntilResample() {
    return stepsUntilResample;
}

int ParticleFilter::getNumberOfParticles() {
    return NUMBER_OF_PARTICLES;
}
