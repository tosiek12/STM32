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

	arm_matrix_instance_f32 P_matrix;
	float32_t P_new[2*2];
	arm_matrix_instance_f32 A_matrix;
	float32_t A[2*2];
	arm_matrix_instance_f32 B_matrix;
	float32_t B[2*1];
	arm_matrix_instance_f32 C_matrix;
	float32_t C[2*1];
	arm_matrix_instance_f32 X_matrix;
	float32_t X[2*1];

	float64_t P[2][2]; // Error covariance matrix - This is a 2x2 matrix
	float64_t K[2]; // Kalman gain - This is a 2x1 vector
	float64_t y; // Angle difference
	float64_t S; // Estimate error
public:
	Kalman();

	// The angle in degrees and the rate in degrees per second and the dt in seconds
	void step(float64_t newAngle, float64_t newRate, float64_t dt);

	float64_t getAngle() {
		return angle;
	}

	 // Used to set angle, this should be set as the starting angle
	void setAngle(float64_t newAngle) {
		angle = newAngle;
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
