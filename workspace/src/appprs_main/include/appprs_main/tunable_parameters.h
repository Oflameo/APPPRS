#ifndef TUNABLE_PARAMETERS
#define TUNABLE_PARAMETERS

const int STEPS_PER_RESAMPLE = 10;
const int NUMBER_OF_PARTICLES = 1000;
const int MAP_SIZE = 800;

const float MAP_RESOLUTION = 10.0;
const float ODOMETRY_RESOLUTION = 100.0;
const float PI = 3.14159265359;

const float MOVEMENT_STD_DEV = 0.001;
const float BEARING_STD_DEV = 0.0005;

const float RANGE_MAX = 15.0;
const int DENSITY_ALONG_RAY = 75;

const float LASER_UNCERTAINTY_SCALAR = 1.0;

#endif // TUNABLE_PARAMETERS

