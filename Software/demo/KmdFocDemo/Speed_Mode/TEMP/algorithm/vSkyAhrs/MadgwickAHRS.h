#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h


#define MadgwickAhrsAngle 57.295779513082320876798154814105f
void  MadgwickAHRSClear(void);
float MadgwickAHRSupdate(float *accel, float *gyro, float *mag,float *angle,float Cycle);
float MadgwickAHRSupdateIMU(float *accel, float *gyro,float *angle,float Cycle);

#endif
