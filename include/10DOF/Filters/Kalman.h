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
	/* OLD APPROACH - MANUALLY COMPUTED */
	/* Kalman filter variables */
	float32_t Q_angle; // Process noise variance for the accelerometer
	float32_t Q_bias; // Process noise variance for the gyro bias
	float32_t R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

	float32_t angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
	float32_t bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
	float32_t P[2][2]; // Error covariance matrix - This is a 2x2 matrix
	float32_t K[2]; // Kalman gain - This is a 2x1 vector
	float32_t y; // Angle difference
	float32_t S; // Estimate error
public:
	Kalman();

	// The angle in degrees and the rate in degrees per second and the dt in seconds
	void stepOldVersion(float32_t newAngle, float32_t newRate, float32_t dt);

	float32_t getAngle() {
		return angle;
	}

	 // Used to set angle, this should be set as the starting angle
	void setAngle(float32_t newAngle) {
		angle = newAngle;
	}

	/* These are used to tune the Kalman filter */
	void setQangle(float32_t newQ_angle) {
		Q_angle = newQ_angle;
	}

	void setQbias(float32_t newQ_bias) {
		Q_bias = newQ_bias;
	}

	void setRmeasure(float32_t newR_measure) {
		R_measure = newR_measure;
	}

	float32_t getQangle() {
		return Q_angle;
	}

	float32_t getQbias() {
		return Q_bias;
	}

	float32_t getRmeasure() {
		return R_measure;
	}

	void testMatrixOperations();
	void sendMatrixViaCom(arm_matrix_instance_f32* matrix);
	/* ----------------------------------------------------------------------
	 * Max magnitude FFT Bin test
	 * ------------------------------------------------------------------- */
	int32_t testExample(void);
};
#endif /* KALMAN_H_ */
