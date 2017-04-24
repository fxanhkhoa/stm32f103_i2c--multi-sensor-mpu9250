#ifndef KALMAN_H 
#define KALMAN_H

#define R_matrix          0.590 
#define Q_Gyro_matrix     0.002 
#define Q_Accel_matrix    0.001

/* 
#define R_matrix          0.390 
#define Q_Gyro_matrix     0.012 
#define Q_Accel_matrix    0.021
*/
typedef struct 
{ 
    // Two states, angle and gyro bias. Unbiased angular rate is a byproduct. 
    float x_bias;
    float x_rate; 
    float x_angle; 
 
    // Covariance of estimation error matrix. 
    float P_00; 
    float P_01; 
    float P_10; 
    float P_11; 
 
    // State constants. 
    //float dt; 
    float R_angle; 
    float Q_gyro; 
    float Q_angle; 
     
} kalman; 
 
 
void kalman_init(kalman *filter, float R_angle, float Q_gyro, float Q_angle); 
void kalman_predict(kalman *filter, float dot_angle,  float dt); 
void kalman_update(kalman *filter, float angle_measured); 
 
// Get the bias. 
float kalman_get_bias(kalman *filter) 
{ 
    return filter->x_bias; 
} 
// Get the rate. 
float kalman_get_rate(kalman *filter) 
{ 
    return filter->x_rate; 
} 
// Get the angle. 
float kalman_get_angle(kalman *filter) 
{ 
    return filter->x_angle; 
} 
 
#endif
