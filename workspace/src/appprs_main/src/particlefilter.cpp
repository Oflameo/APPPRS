#include <appprs_main/particlefilter.h>

ParticleFilter::ParticleFilter()
{

    //std::string imageName("/home/jamie/APPPRS/workspace/src/appprs_main/maps/wean_map_uint8.bmp");
    std::string imageName("/home/jamie/APPPRS/workspace/src/appprs_main/maps/wean_map_uint8_rot.bmp");
    //std::string imageName("/home/jazen/Documents/Classes/2015_Fall/16-831_Stats_in_Robotics/HW/HW_4/APPPRS/workspace/src/appprs_main/maps/wean_map_uint8.bmp"); // by default
    map_image=cv::imread(imageName,CV_LOAD_IMAGE_GRAYSCALE);

    //Check that you got the image
    if(!map_image.data )
    {
        std::cout << "Could not load map" << std::endl;
        throw 0;
    }

    // open map

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
    movementNoise = boost::make_shared<std::normal_distribution<>> (movementNoise_temp);
    bearingNoise = boost::make_shared<std::normal_distribution<>> (bearingNoise_temp);
    resamplingBaseDistribution = boost::make_shared<std::uniform_real_distribution<>> (resamplingBaseDistribution_temp);

    omp_set_num_threads(32);
}

ParticleFilter::~ParticleFilter()
{

}

void ParticleFilter::initializeParticles() {
    std::random_device rd;
    std::mt19937 generator(rd());
    std::uniform_int_distribution<> xIndex(330,MAP_SIZE-1);
    std::uniform_int_distribution<> yIndex(0,MAP_SIZE-1);
    remainingParticles=NUMBER_OF_PARTICLES;

    std::uniform_real_distribution<> th(0,2.0*PI);

    int id = -1;
    while (particlesContainer.size() < remainingParticles) {
        int xIdx = xIndex(generator);
        int yIdx = yIndex(generator);
        //std::cout << "map coord (x,y) = " << xIdx/MAP_RESOLUTION << ", " << yIdx/MAP_RESOLUTION << " image coord (x,y) = " << MAP_SIZE-1-yIdx << ", " << xIdx << std::endl;
        if (map_image.at<uchar>(MAP_SIZE-1-yIdx,xIdx) > 250) {
            single_particle particle;
            particle.setX(xIdx/MAP_RESOLUTION);
            particle.setY(yIdx/MAP_RESOLUTION);
            particle.setTh(th(generator));

            particle.setMapImage(map_image);            
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
#pragma omp parallel for
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
#pragma omp parallel for
    for(int i = 0; i < particlesContainer.size(); i++) {
        if (particlesContainer.at(i)->getWeight() > 0.00001) {
            particlesContainer.at(i)->laserMeasurement(laserRanges, laserWRTMap);
        }
        else {
            particlesContainer.at(i)->setWeight(particlesContainer.at(i)->getWeight()*0.1);
        }
    }

}

void ParticleFilter::parallelResample() {
    std::cout << "Parallel Resampling..." << std::endl;

    stepsUntilResample = STEPS_PER_RESAMPLE;
    normalizeParticleWeights();

    std::cout << "Done normalizing weights..." << std::endl;

    adjustParticleCount();
    float M = remainingParticles;

    float resampling_base = (*resamplingBaseDistribution)(*generator)*1.0/M;
    float sampling_arrow = resampling_base;
    int current_particle = 0;
    std::vector<boost::shared_ptr<single_particle>> temp_particlesContainer;

    std::vector<float> rightEdgesOfParticles;
    std::vector<float> samplingArrows;
    rightEdgesOfParticles.resize(particlesContainer.size());
    samplingArrows.resize(M);

#pragma omp parallel for
    for (int i = 0; i < (int)M; i++) {
        samplingArrows.at(i) = resampling_base + i*1/M;
    }

    float rightEdgeOfParticlesSum = 0;
    for (int i = 0; i < particlesContainer.size(); i++) {
        rightEdgeOfParticlesSum += particlesContainer.at(i)->getWeight();
        //std::cout << "right edge of partcle sum = " << rightEdgeOfParticlesSum << std::endl;
        rightEdgesOfParticles.at(i) = rightEdgeOfParticlesSum;
    }

//#pragma omp parallel for
    int lastParticle = 0;
    for (int i = 0; i < (int)M; i++) {
        float samplingArrow = samplingArrows.at(i);
        for (int j = lastParticle; j < particlesContainer.size(); j++) {

            //std::cout << "sampling arrow = " << samplingArrow << ",   right edge of particle = " << rightEdgesOfParticles.at(j) << std::endl;

            if (samplingArrow < rightEdgesOfParticles.at(j)) {

                //std::cout << "adding particle id # " << particlesContainer.at(j)->getId() << std::endl;

                // add particle
                single_particle particle;
                particle.setX(particlesContainer.at(j)->getX());
                particle.setY(particlesContainer.at(j)->getY());
                particle.setTh(particlesContainer.at(j)->getTh());
                particle.setId(particlesContainer.at(j)->getId());
                particle.setWeight(1.0);
                particle.setMapImage(map_image);
                auto p = boost::make_shared<single_particle> (particle);
                temp_particlesContainer.push_back(p);

                lastParticle = j;

                break;
            }
        }
    }

    particlesContainer.swap(temp_particlesContainer);
    temp_particlesContainer.clear();
    perturbParticles();
    //resetParticleWeights();
}

void ParticleFilter::resample() {
    std::cout << "Resampling..." << std::endl;


    stepsUntilResample = STEPS_PER_RESAMPLE;
    float M = remainingParticles;
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
            //single_particle particle((*particlesContainer.at(current_particle)));
            single_particle particle;
            particle.setX(particlesContainer.at(current_particle)->getX());
            particle.setY(particlesContainer.at(current_particle)->getY());
            particle.setTh(particlesContainer.at(current_particle)->getTh());
            particle.setId(particlesContainer.at(current_particle)->getId());
            particle.setMapImage(map_image);
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


    particlesContainer.swap(temp_particlesContainer);


    std::cout<<"There are: "<< particlesContainer.size()<<"Particles now"<<std::endl;
    perturbParticles();
    resetParticleWeights();
    if (remainingParticles>500)
    {
    	//remainingParticles=round(remainingParticles*.9);
    }
    temp_particlesContainer.clear();

}

void ParticleFilter::normalizeParticleWeights() {
    //std::cout << "Normalizing weights..." << std::endl;

    float totalWeight = 0;
    float M = particlesContainer.size();
    //std::cout << "M has:" << M << " particles" << std::endl;

//#pragma omp parallel for
	for(int i=0; i < particlesContainer.size(); i++)
	{
        totalWeight += particlesContainer.at(i)->getWeight();
        //std::cout << "\n particle id " << particlesContainer.at(i)->getId() << " has weight = " << particlesContainer.at(i)->getWeight()
        //          << "  --->  total weight now = " << totalWeight;
	}
    //std::cout << std::endl;
    std::cout << "total particle weight BEFORE = " << totalWeight << std::endl;
    //std::this_thread::sleep_for(std::chrono::milliseconds(1000));


    if (totalWeight ==0)
    {
        std::cout<<"All particles have been crushed. Reset all particles."<<std::endl;
    	particlesContainer.clear();
    	initializeParticles();
    	totalWeight=particlesContainer.size();
    }



	float new_weight;
//#pragma omp parallel for
	for(int i=0; i < particlesContainer.size(); i++)
    {
        new_weight = (particlesContainer.at(i)->getWeight())/totalWeight;

        //std::cout << "\n old weight = " << particlesContainer.at(i)->getWeight();
        //std::cout << "\n new weight = " << new_weight << std::endl;

		particlesContainer.at(i)->setWeight(new_weight);        
        //std::this_thread::sleep_for(std::chrono::milliseconds(20));

    }

    totalWeight = 0;
//#pragma omp parallel for
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

#pragma omp parallel for
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
    return particlesContainer.size();
}

void ParticleFilter::perturbParticles() {

    std::normal_distribution<float> dx(0,0.1);
    std::normal_distribution<float> dy(0,0.1);
    std::normal_distribution<float> dth(0,PI/20.0);
    std::default_random_engine generator2;



#pragma omp parallel for
    for (int i=0; i < particlesContainer.size(); i++)
    {
        float DX=dx(generator2);
        float DY=dy(generator2);
        float DTH=dth(generator2);
        particlesContainer.at(i)->perturbPoint(DX,DY,DTH);
    }

}

void ParticleFilter::adjustParticleCount() {
    // find bounding box of particles. If it is small, like a few meters, reduce the number of particles by resampling
    float minX = 1e9;
    float maxX = -1e9;
    float minY = 1e9;
    float maxY = -1e9;
    for (int i = 0; i < particlesContainer.size(); i++) {
        float x = particlesContainer.at(i)->getX();
        float y = particlesContainer.at(i)->getY();
        if (x < minX) {
            minX = x;
        }
        if (x > maxX) {
            maxX = x;
        }
        if (y < minY) {
            minY = y;
        }
        if (y > maxY) {
            maxY = y;
        }
    }

    if ((maxX - minX) < 5.0 && (maxY - minY) < 5.0) {
        remainingParticles = ((int)remainingParticles/2 > 1000) ? (int)remainingParticles/2 : 1000;
    }
}
