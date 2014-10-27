/*
 * Kalman.cpp
 *
 *  Created on: 14 sie 2014
 *      Author: Antonio
 */

#include "Kalman.h"


Kalman::Kalman() {
		/* We will set the variables like so, these can also be tuned by the user */
		Q_angle = 0.001;
		Q_bias = 0.003;
		R_measure = 0.3;

		//State: x = [angle; bias]
		//Control: u = omega;
		//bledy = [szum omegi; blad przesuniecia]

		//A = F = [1, -dt; 0, 1]
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

// The angle in degrees and the rate in degrees per second and the dt in seconds
	void Kalman::step(float64_t newAngle, float64_t newRate, float64_t dt) {
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
