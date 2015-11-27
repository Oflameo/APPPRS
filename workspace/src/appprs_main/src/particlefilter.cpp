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
    std::uniform_real_distribution<> resamplingBaseDistribution_temp(0.0,1.0);




    generator = boost::make_shared<std::mt19937> (generator_temp);

//    generator2 = boost::make_shared<std::default_random_engine> (generator2_tmp);

    movementNoise = boost::make_shared<std::normal_distribution<>> (movementNoise_temp);
    bearingNoise = boost::make_shared<std::normal_distribution<>> (bearingNoise_temp);
    resamplingBaseDistribution = boost::make_shared<std::uniform_real_distribution<>> (resamplingBaseDistribution_temp);
    //dx = boost::make_shared<std::normal_distribution<>> (dx_temp);
    //dy = boost::make_shared<std::normal_distribution<>> (dy_temp);
    //dth = boost::make_shared<std::normal_distribution<>> (dth_temp);

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

    int id = -1;
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
            id++;
            particle.setId(id);
            auto p = boost::make_shared<single_particle> (particle);
            particlesContainer.push_back(p);
        }
    }
    std::cout << "finished initializing particles" << std::endl;
    resetParticleWeights();

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
    std::cout << "There are " << stepsUntilResample << " steps until resample" << std::endl;
    for(int i = 0; i < particlesContainer.size(); i++) {
        if (particlesContainer.at(i)->getWeight() > 0.005) {
            particlesContainer.at(i)->laserMeasurement(laserRanges, laserWRTMap);
        }
        else {
            //particlesContainer.at(i)->setWeight(particlesContainer.at(i)->getWeight()*0.001);
        }
    }

}

void ParticleFilter::resample() {
    std::cout << "Resampling..." << std::endl;
    std::normal_distribution<float> dx(0,.05);
    std::normal_distribution<float> dy(0,.05);
    std::normal_distribution<float> dth(0,6);
    std::default_random_engine generator2;

    stepsUntilResample = STEPS_PER_RESAMPLE;
    float M = particlesContainer.size();
    normalizeParticleWeights();   

    std::cout << "Done normalizing weights..." << std::endl;

    float resampling_base = (*resamplingBaseDistribution)(*generator)*1.0/M;
    float sampling_arrow = resampling_base;
    int current_particle = 0;
    std::vector<boost::shared_ptr<single_particle>> temp_particlesContainer;

    float importance_sum = particlesContainer.at(current_particle)->getWeight();

    //for(int i=0; i < (int)M ; i++)
    int i = 0;
    while (temp_particlesContainer.size() < int(M))
    {
        i++;
        //std::cout << "\n i = " << i << "\n right edge of current particle = " << importance_sum
        //          << "\n sampling arrow = " << sampling_arrow;

        if (i > 2*M) {
            std::cout << "\n SOMETHING TOTALLY MESSED UP - ALL PARTICLES OUTSIDE OF VIABLE MAP" << std::endl;
            throw 0;
        }


        if (sampling_arrow < importance_sum) {

            //temp_particlesContainer.push_back(particlesContainer.at(current_particle));
            single_particle particle((*particlesContainer.at(current_particle)));
            float DX=dx(generator2);
            float DY=dy(generator2);
            float DTH=dth(generator2);
            particle.perturbPoint(DX, DY, DTH);
            //particle.setX(particlesContainer.at(current_particle)->getX());
            //particle.setY(particlesContainer.at(current_particle)->getY());
            //particle.setTh(particlesContainer.at(current_particle)->getTh());
            //particle.setMapImage(map_image);
            //particle.setId(particlesContainer.at(current_particle)->getId());
            auto p = boost::make_shared<single_particle> (particle);
            temp_particlesContainer.push_back(p);

            //std::cout << "\n sampling arrow is inside a particle --> sample retains particle id " << particlesContainer.at(current_particle)->getId();
            //std::cout << "\n we have retained " << temp_particlesContainer.size() << " particles so far";
            if (temp_particlesContainer.size() == (int)M) {
                //std::cout << std::endl;
                break;
            }
        }

        sampling_arrow += 1/M;
        //std::cout << "\n new sampling arrow = " << sampling_arrow;

        while (sampling_arrow > importance_sum) {
            if (current_particle < particlesContainer.size()-1) {
                current_particle++;
                //std::cout << "\n sampling arrow has moved past current particle --> incrementing current_particle";
            }
            else {
                //std::cout << "\n already reached end of particles, just sample the last one until the vector is full";
                importance_sum = 999;
                break;
            }
            importance_sum += particlesContainer.at(current_particle)->getWeight();
            //std::cout << "\n new importance sum = " << importance_sum;
        }
        //std::cout << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    //std::cout<<"temp container has "<<temp_particlesContainer.size()<<"  orig has "<< particlesContainer.size()<<std::endl;

    /*
    std::cout << "\n BEFORE particle container swap:" << std::endl;
    std::cout << "IDs:" << std::endl;
    for (int i = 0; i < particlesContainer.size(); i++) {
        std::cout << "original = " << particlesContainer.at(i)->getId() << "   resampled = " << temp_particlesContainer.at(i)->getId() << std::endl;
    }
    */

    particlesContainer.swap(temp_particlesContainer);

    /*
    std::cout << "\n AFTER particle container swap:" << std::endl;
    std::cout << "IDs:" << std::endl;
    for (int i = 0; i < particlesContainer.size(); i++) {
        std::cout << "original = " << particlesContainer.at(i)->getId() << "   resampled = " << temp_particlesContainer.at(i)->getId() << std::endl;
    }
    */
    std::cout<<"There are: "<< particlesContainer.size()<<"Particles now"<<std::endl;

    resetParticleWeights();
}

void ParticleFilter::normalizeParticleWeights() {
    //std::cout << "Normalizing weights..." << std::endl;

    float totalWeight = 0;
    float M = particlesContainer.size();
    //std::cout << "M has:" << M << " particles" << std::endl;
	for(int i=0; i < particlesContainer.size(); i++)
	{
        totalWeight += particlesContainer.at(i)->getWeight();
        //std::cout << "\n particle id " << particlesContainer.at(i)->getId() << " has weight = " << particlesContainer.at(i)->getWeight()
        //          << "  --->  total weight now = " << totalWeight;
	}
    //std::cout << std::endl;
    std::cout << "total particle weight BEFORE = " << totalWeight << std::endl;
    //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    if(totalWeight<.00001)
    {
    	std::cout<<"Weights all dropped too crazy low, you fail"<<std::endl;
    	particlesContainer.clear();
    	initializeParticles();
    	totalWeight=particlesContainer.size();
    }


	float new_weight;
	for(int i=0; i < particlesContainer.size(); i++)
    {
        new_weight = (particlesContainer.at(i)->getWeight())/totalWeight;

        //std::cout << "\n old weight = " << particlesContainer.at(i)->getWeight();
        //std::cout << "\n new weight = " << new_weight << std::endl;

		particlesContainer.at(i)->setWeight(new_weight);        
        //std::this_thread::sleep_for(std::chrono::milliseconds(20));

    }

    totalWeight = 0;
    for(int i=0; i < particlesContainer.size(); i++)
    {
        totalWeight += particlesContainer.at(i)->getWeight();
        //std::cout << "\n particle id " << particlesContainer.at(i)->getId() << " has weight = " << particlesContainer.at(i)->getWeight()
        //          << "  --->  total weight now = " << totalWeight;
    }
    //std::cout << std::endl;
    std::cout << "total particle weight AFTER = " << totalWeight << std::endl;
    //std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

void ParticleFilter::resetParticleWeights() {
    //std::cout << "Setting weights to 1..." << std::endl;

    for (int i=0; i<particlesContainer.size(); i++) {
        particlesContainer.at(i)->setWeight(1.0);
    }
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


