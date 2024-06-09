
#ifndef MahonyAHRS_h
#define MahonyAHRS_h



#define MahonyAhrsAngle 57.295779513082320876798154814105f

extern volatile float twoKp;			
extern volatile float twoKi;			
extern volatile float mq0, mq1, mq2, mq3;	

void MahonyAHRSClear();
float MahonyAHRSupdate(float *accel, float *gyro, float *mag,float *angle,float Cycle);
float MahonyAHRSupdateIMU(float *accel, float *gyro,float *angle,float Cycle);

#endif
