#include "kalman.h" 

/*********************************************************************  
 The implememted kalman filter estimates the angle that will be used 
 on the PID controller alogrithm. The filter consists of two states 
 [angle, gyro_bais].
 *********************************************************************/ 
 
/******************************************************************** 
*    Function Name:  kalman_init                                    * 
*    Return Value:   none                                           * 
*    Parameters:     struct filter, dt, R_angle, Q_gyro, Q_angle    * 
*    Description:    Initialize the kalman Filter parameters.       * 
********************************************************************/  

void kalman_init(kalman *filter, float R_angle, float Q_gyro, float Q_angle) // float dt,
{ 
    // Initialize the two states, the angle and the gyro bias. As a 
    // byproduct of computing the angle, we also have an unbiased 
    // angular rate available. 
    filter->x_bias = 0.0; 
    filter->x_rate = 0.0; 
    filter->x_angle = 0.0; 
 
 
    // Initialize the delta in seconds between gyro samples. 
    //filter->dt = dt; 
 
    // Initialize the measurement noise covariance matrix values. 
    // In this case, R is a 1x1 matrix tha represents expected 
    // jitter from the accelerometer. 
    filter->R_angle = R_angle; 
 
    // Initialize the process noise covariance matrix values. 
    // In this case, Q indicates how much we trust the acceleromter 
    // relative to the gyros. 
    // Q_gyro set to 0.003 and Q_angle set to 0.001. 
    
    filter->Q_gyro = Q_gyro; 
    filter->Q_angle = Q_angle; 
 
    // Initialize covariance of estimate state.  This is updated 
    // at every time step to determine how well the sensors are 
    // tracking the actual state. 
    
    filter->P_00 = 1.0; 
    filter->P_01 = 0.0; 
    filter->P_10 = 0.0; 
    filter->P_11 = 1.0; 
} 

/******************************************************************** 
*    Function Name:  kalman_predict                            * 
*    Return Value:   none                                           * 
*    Parameters:     struct filter, measured gyroscope value        * 
*    Description:    Called every dt(Timer 1 overflow with a biased * 
*                    gyro. Also updates the current rate and angle  * 
*                    estimate).                                     * 
********************************************************************/  
void kalman_predict(kalman *filter, float dot_angle,  float dt) 
{ 
    // Static so these are kept off the stack. 
    static float gyro_rate_unbiased; 
    static float Pdot_00; 
    static float Pdot_01; 
    static float Pdot_10; 
    static float Pdot_11;
    
    // Unbias our gyro. 
    gyro_rate_unbiased= dot_angle - filter->x_bias; 
 
    // Store our unbiased gyro estimate. 
    filter->x_rate= gyro_rate_unbiased; 
     
    // Update the angle estimate. 
    filter->x_angle= filter->x_angle + (dot_angle - filter->x_bias)*dt; 
 
    // Compute the derivative of the covariance matrix 
    // Pdot = A*P + P*A' + Q 
    Pdot_00 = filter->Q_angle - filter->P_01 - filter->P_10; 
    Pdot_01 = -filter->P_11; 
    Pdot_10 = -filter->P_11; 
    Pdot_11 = filter->Q_gyro; 
 
    // Update the covariance matrix. 
    filter->P_00 += Pdot_00 * dt; 
    filter->P_01 += Pdot_01 * dt; 
    filter->P_10 += Pdot_10 * dt; 
    filter->P_11 += Pdot_11 * dt;
}
/********************************************************************* 
*    Function Name:  kalman_update                                   * 
*    Return Value:   none                                            * 
*    Parameters:     struct filter, measured angle value             *
*    Description:    Called when a new accelerometer angle           * 
*                    measurement is available. Updates the estimated * 
*                    angle that will be used.                        * 
*********************************************************************/
void kalman_update(kalman *filter, float angle_measured)
{
    // Static so these are kept off the stack. 
    static float y;
    static float S;
    static float K_0;
    static float K_1;
    
    // Compute the error in the estimate. 
    // Innovation or Measurement Residual 
    // y = z - Hx 
    y= angle_measured - filter->x_angle; 
 
    // Compute the error estimate. 
    // S = C P C' + R 
    S = filter->R_angle + filter->P_00; 
 
    // Compute the kalman filter gains. 
    // K = P C' inv(S) 
     K_0 = filter->P_00 / S; 
     K_1 = filter->P_10 / S; 
 
    // Update covariance matrix. 
    // P = P - K C P 
    filter->P_00 -= K_0 * filter->P_00; 
    filter->P_01 -= K_0 * filter->P_01; 
    filter->P_10 -= K_1 * filter->P_00; 
    filter->P_11 -= K_1 * filter->P_01; 
   
    // Update the state (new)estimates (Correct the prediction of the state). 
    // Also adjust the bias on the gyro at every iteration. 
    // x = x + K * y 
    filter->x_angle = filter->x_angle + K_0 * y; 
    filter->x_bias = filter->x_bias + K_1 * y;  
     
}
