#include <appprs_main/particlefilter.h>

ParticleFilter::ParticleFilter()
{

    std::string imageName("/home/jamie/APPPRS/workspace/src/appprs_main/maps/wean_map_uint8.bmp");
    //std::string imageName("/home/jazen/Documents/Classes/2015_Fall/16-831_Stats_in_Robotics/HW/HW_4/APPPRS/workspace/src/appprs_main/maps/wean_map_uint8.bmp"); // by default
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
    /*
    for (int i = 0; i < 180; i++) {
        float thi = (float)i;
        Eigen::MatrixXf laserFrameRay = Eigen::MatrixXf::Ones(3,DENSITY_ALONG_RAY);
        laserFrameRay.row(0) = Eigen::VectorXf::LinSpaced(DENSITY_ALONG_RAY,0.0,RANGE_MAX*cos(thi));
        laserFrameRay.row(1) = Eigen::VectorXf::LinSpaced(DENSITY_ALONG_RAY,0.0,RANGE_MAX*sin(thi));
        laserFrameRays.push_back(laserFrameRay);
    }
    */

    initializeParticles();
    stepsUntilResample = STEPS_PER_RESAMPLE;
    lastOdometry.resize(3);
    lastOdometry.at(0) = 0;
    lastOdometry.at(1) = 0;
    lastOdometry.at(2) = 0;


    std::random_device rd_temp;
    std::mt19937 generator_temp(rd_temp());
    std::normal_distribution<> movementNoise_temp(0,MOVEMENT_STD_DEV);
    std::normal_distribution<> bearingNoise_temp(0,BEARING_STD_DEV);

    generator = boost::make_shared<std::mt19937> (generator_temp);
    movementNoise = boost::make_shared<std::normal_distribution<>> (movementNoise_temp);
    bearingNoise = boost::make_shared<std::normal_distribution<>> (bearingNoise_temp);
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
            //particle.setLaserRays(laserFrameRays);
            auto p = boost::make_shared<single_particle> (particle);
            particlesContainer.push_back(p);
        }
    }
    std::cout << "finished initializing particles" << std::endl;
}


void ParticleFilter::odometry(std::vector<float> newOdometry) {
    if (lastOdometry.at(0) + lastOdometry.at(1) + lastOdometry.at(2) != 0.0) {
        std::vector<float> movement;
        for (int i = 0; i < 3; i++) {
            movement.push_back((newOdometry.at(i) - lastOdometry.at(i))/ODOMETRY_RESOLUTION);
        }
        for(int i = 0; i < particlesContainer.size(); i++) {
            movement.at(0) += (*movementNoise)(*generator);
            movement.at(1) += (*movementNoise)(*generator);
            movement.at(2) += (*bearingNoise)(*generator);
            particlesContainer.at(i)->move(movement);
        }
    }
    lastOdometry.swap(newOdometry);
}

void ParticleFilter::laser(std::vector<float> laserRanges, std::vector<float> laserWRTMap) {

    stepsUntilResample--;    
    //std::cout << "There are " << stepsUntilResample << " steps until resample" << std::endl;
    for(int i = 0; i < particlesContainer.size(); i++) {
        particlesContainer.at(i)->laserMeasurement(laserRanges, laserWRTMap);
    }

}

void ParticleFilter::resample() {
    std::cout << "Resampling..." << std::endl;
    stepsUntilResample = STEPS_PER_RESAMPLE;
    float M=particlesContainer.size();
    normalizeParticleWeights();
    std::cout << "Done normalizing Weights..." << std::endl;

    //std::uniform_real_distribution<double> resampling_base_distribution(0.,1.);
  //  seed_clock::duration duration = myClock.now() - beginning;
//    unsigned int seed_input = int(time(NULL));
   // (*generator);
    //   generator.seed(seed_input);
    float resampling_base = (rand()%10001)/10000.0*1.0/M;
    float current_importance = resampling_base;
    int current_particle = 0;
    std::vector<boost::shared_ptr<single_particle>> temp_particlesContainer;
    if (current_particle>particlesContainer.size())
    {
std::cout<<"your countins is off. Current_particle is above size of array"<<std::endl;
    }

    float importance_sum=particlesContainer.at(current_particle)->getWeight();
    for(int i=0; i<(M);i++)
    {
    	float U=resampling_base+(i)*1/M;
    	while (U>importance_sum)
    	{
    		current_particle++;
    		importance_sum+=particlesContainer.at(i)->getWeight();
    	}
    	temp_particlesContainer.push_back(particlesContainer.at(current_particle));
    }
    std::cout<<"temp container has "<<temp_particlesContainer.size()<<"orig has "<< particlesContainer.size()<<std::endl;

    particlesContainer.swap(temp_particlesContainer);

    resetParticleWeights();
}

void ParticleFilter::normalizeParticleWeights() {
    std::cout << "Normalizing weights..." << std::endl;

	float total_importance=0;
	int M=particlesContainer.size();
	std::cout<<"M has:"<<M<<" particles"<<std::endl;
	for(int i=0; i < particlesContainer.size(); i++)
	{
		total_importance+=particlesContainer.at(i)->getWeight();
	}

	float new_weight;
	for(int i=0; i < particlesContainer.size(); i++)
		{
		new_weight=(particlesContainer.at(i)->getWeight())*1/total_importance;
		particlesContainer.at(i)->setWeight(new_weight);
  std::cout<< "weight = " << new_weight << " *******************************************************" << std::endl;
  //std::this_thread::sleep_for(std::chrono::milliseconds(20));

		}

}
void ParticleFilter::resetParticleWeights() {
    std::cout << "Setting weights to 1..." << std::endl;

for(int i=0; i<particlesContainer.size(); i++)
	particlesContainer.at(i)->setWeight(1);
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


