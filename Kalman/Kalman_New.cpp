#include "Kalman_New.h"

Kalman::Kalman()
{
	// Set variables
	Q_angle = 0.001f;
	Q_bias = 0.003f;
	R_measure = 0.03f;
	
	angle = 0.0f; // Reset angle
	bias = 0.0f; // Reset bias
	
	// Reset covariance matrix
	P[0][0] = 0.0f; 
	P[0][1] = 0.0f;
	P[1][0] = 0.0f;
	P[1][1] = 0.0f;
}


Kalman::~Kalman()
{
}

float Kalman::GetAngle(float newAngle, float newRate,float dt)
{
	// Step 1 : Discrete Kalman filter time update equations - Time Update ("Predict")
	rate = newRate - bias;
	angle += dt * rate;
	
	// Step 2: Update estimation error covariance - Project the error covariance ahead
	P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
	P[0][1] -= dt * P[1][1];
	P[1][0] -= dt * P[1][1];
	P[1][1] += Q_bias * dt;
	
	// Step 4 : Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
	// Calculate Kalman gain - Compute the Kalman gain
	float S = P[0][0] + R_measure;
	
	// Step 5: Kalman gain
	float K[2];
	K[0] = P[0][0] / S;
	K[1] = P[1][0] / S;
	
	// Step 3: Calculate angle and bias - Update estimate with measurement zk (newAngle)
	float y = newAngle - angle; // Angle difference
	
	// Step 6
	angle += K[0] * y;
	bias += K[1] * y;
	
	// Step 7: Calculate estimation error covariance - Update the error covariance
	float P00_temp = P[0][0];
	float P01_temp = P[0][1];
	
	P[0][0] -= K[0] * P00_temp;
	P[0][1] -= K[0] * P01_temp;
	P[1][0] -= K[1] * P00_temp;
	P[1][1] -= K[1] * P01_temp;
	
	return angle;
}

void Kalman::setAngle(float angle) { this->angle = angle; }; // Used to set angle, this should be set as the starting angle
