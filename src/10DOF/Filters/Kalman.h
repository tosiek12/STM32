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

	/* NEW APPROACH - USING ARM_MATRIX OPERATIONS */
	//State: x = [angle; bias];
	arm_matrix_instance_f32 xMatrix;
	float32_t xSource[2*1];

	//Control: u = omega; //angular velocity on given axis
	//ProcessErros = [angleNoise; biasNoise];

	//a = F = [1, -dt; 0, 1]
	arm_matrix_instance_f32 aMatrix;
	float32_t aSource[2*2];

	//b = [1;0]
	arm_matrix_instance_f32 bMatrix;
	float32_t bSource[2*1];

	//c= [1, 0]
	arm_matrix_instance_f32 cMatrix;
	float32_t cSource[2*1];

	//p = [0, 0; 0, 0]
	arm_matrix_instance_f32 pMatrix;
	float32_t pSource[2*2];

	//q = [angleNoise, 0; 0, biasNoise]
	arm_matrix_instance_f32 qMatrix;
	float32_t qSource[2*2];

	//r = [measurment noise];
	arm_matrix_instance_f32 rMatrix;
	float32_t rSource[1*1];

public:
	Kalman();

	// The angle in degrees and the rate in degrees per second and the dt in seconds
	void stepOldVersion(float32_t newAngle, float32_t newRate, float32_t dt);
	void stepNewVersion(float32_t newAngle, float32_t newRate, float32_t dt);

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
	void ble();
	void sendMatrixViaCom(arm_matrix_instance_f32* matrix);
	void sendStateViaCom();
	/* ----------------------------------------------------------------------
	 * Max magnitude FFT Bin test
	 * ------------------------------------------------------------------- */
	int32_t testExample(void);
};
#endif /* KALMAN_H_ */
