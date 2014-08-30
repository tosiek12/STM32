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
	Kalman() {
		/* We will set the variables like so, these can also be tuned by the user */
		Q_angle = 0.001;
		Q_bias = 0.003;
		R_measure = 0.3;

		//A = [1, -dt; 0, 1]
		//B = [1;0]
		//C = [1,0]
		//D = [0]
		X_matrix.numCols = 1;
		X_matrix.numRows = 2;
		X_matrix.pData = X;

		P_matrix.numCols = 2;
		P_matrix.numRows = 2;
		P_matrix.pData = P_new;



		A_matrix.numCols = 2;
		A_matrix.numRows = 2;
		A_matrix.pData = A;

		C_matrix.numCols = 2;
		C_matrix.numRows = 1;
		C_matrix.pData = C;

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

		// Stage 1: Predict
		// Update xhat - Project the state ahead
		angle += dt * (newRate - bias);
		// Update estimation error covariance
		P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
		P[0][1] += - dt * P[1][1];
		P[1][0] += - dt * P[1][1];
		P[1][1] += Q_bias * dt;

		// Stage 2: Correct
		// Calculate Kalman gain
		S = P[0][0] + R_measure;
		/* Step 5 */
		K[0] = P[0][0] / S;
		K[1] = P[1][0] / S;

		// Update estimate with measurement zk (newAngle)
		y = newAngle - angle;
		/* Step 6 */
		angle += K[0] * y;
		bias += K[1] * y;

		// Update the error covariance
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
