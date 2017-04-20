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
    double x_bias;
    double x_rate; 
    double x_angle; 
 
    // Covariance of estimation error matrix. 
    double P_00; 
    double P_01; 
    double P_10; 
    double P_11; 
 
    // State constants. 
    //double dt; 
    double R_angle; 
    double Q_gyro; 
    double Q_angle; 
     
} kalman; 
 
 
void kalman_init(kalman *filter, double R_angle, double Q_gyro, double Q_angle); 
void kalman_predict(kalman *filter, double dot_angle,  double dt); 
void kalman_update(kalman *filter, double angle_measured); 
 
// Get the bias. 
double kalman_get_bias(kalman *filter) 
{ 
    return filter->x_bias; 
} 
// Get the rate. 
double kalman_get_rate(kalman *filter) 
{ 
    return filter->x_rate; 
} 
// Get the angle. 
double kalman_get_angle(kalman *filter) 
{ 
    return filter->x_angle; 
} 
 
#endif
