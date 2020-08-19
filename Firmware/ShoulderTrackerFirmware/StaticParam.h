/** ShoulderTracking device firmware
 * 
 * Copyright Vincent Crocher - Unimelb - 2016, 2020
 * License MIT license
 */

//For use with MinIMU-9 v3: https://www.pololu.com/product/2468/
#ifdef V1_IMU02A
#include <LSM303.h>
#include <L3G.h>
#define GYRO_2_DPS 1/32768. //Gyroscope conversion rate to degree per seconds
#define ACC_2_MS2 1/16276. //Acceleration convertion to m.s-2

#define BuzzPin 6
#define BeepPin 11
#define ModePin 12
#endif

//For use with AltIMU-10 v5: https://www.pololu.com/product/2739
#ifdef V2_ALTIMUv10
#define GYRO_2_DPS 1/32768. //Gyroscope conversion rate to degree per seconds
#define ACC_2_MS2 1/16276. //Acceleration convertion to m.s-2
#include <LIS3MDL.h>
#include <LSM6.h>
//Extreme magnetometer values to calibrate
LIS3MDL::vector<int16_t> m_min = { -4395,  -3894,  -4690};
LIS3MDL::vector<int16_t> m_max = { +3276,  +3562,  +3024};

#define BuzzPin 5
#define BeepPin 11
#define ModePin 12
#endif


#define FILT_ORDER 2
const float FILT_COEFS_a[FILT_ORDER]={1.0000, -0.5095};
const float FILT_COEFS_b[FILT_ORDER]={0.2452, 0.2452};

enum MODE {STATIC, DYNAMIC};

typedef struct
{
	float A[3], ARef[3];
	float MAngleRef;
	float AngleThresh;
}Static_param;


typedef struct
{
  float A[2];
  float v_c;
  float v[FILT_ORDER], vf[FILT_ORDER];
}Dynamic_param;
