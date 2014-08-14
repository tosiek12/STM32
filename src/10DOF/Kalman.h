/*
 * Kalman.h
 *
 *  Created on: 14 sie 2014
 *      Author: Antonio
 */


#ifndef KALMAN_H_
#define KALMAN_H_
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "arm_math.h"

class Kalman {
private:
	/* Kalman filter variables */
	float64_t Q_angle; // Process noise variance for the accelerometer
	float64_t Q_bias; // Process noise variance for the gyro bias
	float64_t R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

	float64_t angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
	float64_t bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
	float64_t rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

	float64_t P[2][2]; // Error covariance matrix - This is a 2x2 matrix
	float64_t K[2]; // Kalman gain - This is a 2x1 vector
	float64_t y; // Angle difference
	float64_t S; // Estimate error
public:
	Kalman() {
		/* We will set the variables like so, these can also be tuned by the user */
		Q_angle = 0.001;
		Q_bias = 0.003;
		R_measure = 0.03;

		angle = 0; // Reset the angle
		bias = 0; // Reset bias

		P[0][0] = 0; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
		P[0][1] = 0;
		P[1][0] = 0;
		P[1][1] = 0;
	}

	;
	// The angle in degrees and the rate in degrees per second and the dt in seconds
	void step(float64_t newAngle, float64_t newRate, float64_t dt) {
		// KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
		// Modified by Kristian Lauszus
		// See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

		// Time Update ("Predict")
		// Update xhat - Project the state ahead
		/* Step 1 */
		rate = newRate - bias;
		angle += dt * rate;

		// Update estimation error covariance
		/* Step 2 */
		P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
		P[0][1] -= dt * P[1][1];
		P[1][0] -= dt * P[1][1];
		P[1][1] += Q_bias * dt;

		// Measurement Update ("Correct")
		// Calculate Kalman gain
		/* Step 4 */
		S = P[0][0] + R_measure;
		/* Step 5 */
		K[0] = P[0][0] / S;
		K[1] = P[1][0] / S;

		// Calculate angle and bias - Update estimate with measurement zk (newAngle)
		/* Step 3 */
		y = newAngle - angle;
		/* Step 6 */
		angle += K[0] * y;
		bias += K[1] * y;

		// Update the error covariance
		/* Step 7 */
		P[0][0] -= K[0] * P[0][0];
		P[0][1] -= K[0] * P[0][1];
		P[1][0] -= K[1] * P[0][0];
		P[1][1] -= K[1] * P[0][1];
	}

	float64_t getAngle() {
		return angle;
	}
	 // Used to set angle, this should be set as the starting angle
	void setAngle(float64_t newAngle) {
		angle = newAngle;
	}

	// Return the unbiased rate
	float64_t getRate() {
		return rate;
	}

	/* These are used to tune the Kalman filter */
	void setQangle(float64_t newQ_angle) {
		Q_angle = newQ_angle;
	}

	void setQbias(float64_t newQ_bias) {
		Q_bias = newQ_bias;
	}

	void setRmeasure(float64_t newR_measure) {
		R_measure = newR_measure;
	}

	float64_t getQangle() {
		return Q_angle;
	}

	float64_t getQbias() {
		return Q_bias;
	}

	float64_t getRmeasure() {
		return R_measure;
	}
};

#endif /* KALMAN_H_ */
