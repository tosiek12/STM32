/*
 * Kalman.cpp
 *
 *  Created on: 14 sie 2014
 *      Author: Antonio
 */

#include "10DOF/Filters/Kalman.h"
extern "C" {
#include <usbd_cdc_if_template.h>
}

Kalman::Kalman() {
	/* The variables are set like so, these can also be tuned by the user. */
	Q_angle = 0.001;
	Q_bias = 0.003;
	R_measure = 0.3;
	angle = 0; // Reset the angle
	bias = 0; // Reset bias

	P[0][0] = 0; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
	P[0][1] = 0;
	P[1][0] = 0;
	P[1][1] = 0;
}

// The angle in degrees and the rate in degrees per second and the dt in seconds
void Kalman::stepOldVersion(float32_t newAngle, float32_t newRate, float32_t dt) {
	// KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
	// See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

	// Stage 1: Predict
	// Update xhat - Project the state ahead
	angle += dt * (newRate - bias);
	// Update estimation error covariance
	P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
	P[0][1] += -dt * P[1][1];
	P[1][0] += -dt * P[1][1];
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

void Kalman::testMatrixOperations() {
	arm_matrix_instance_f32 matrix1, vector, result;
	float32_t matrixArray[2 * 2] = { 1, 0, 0, 1 },
			vectorArray[2 * 1] = { 1, 2 }, resultArray[1 * 2] = { 0, 0 };
	volatile arm_status status = ARM_MATH_TEST_FAILURE;

	arm_mat_init_f32(&matrix1, 2, 2, (float32_t*) matrixArray);
	sendMatrixViaCom(&matrix1);

	arm_mat_init_f32(&vector, 2, 1, (float32_t*) vectorArray);
	sendMatrixViaCom(&vector);

	arm_mat_init_f32(&result, 2, 1, (float32_t*) resultArray);

	status = arm_mat_mult_f32(&matrix1, &vector, &result);
	sendMatrixViaCom(&result);
	if (status != ARM_MATH_SUCCESS) {
		VCP_write("Erroor\n", 7);
	}
	//arm_mat_trans_f32();
	VCP_write("END\n", 4);

}

void Kalman::sendMatrixViaCom(arm_matrix_instance_f32* matrix) {
	char buf[10] = { 0 };
	uint8_t numberOfCharsInBuffer;
	volatile float32_t number;
	uint8_t i = 0, j = 0;

	for (i = 0; i < matrix->numRows; i++) {
		for (j = 0; j < matrix->numCols; j++) {
			number = *(matrix->pData + matrix->numCols * i + j);
			numberOfCharsInBuffer = sprintf(buf, "%ld,",
					(int32_t) (100 * number));
			VCP_write(buf, numberOfCharsInBuffer);
		}
		numberOfCharsInBuffer = sprintf(buf, "\n");
		VCP_write(buf, numberOfCharsInBuffer);
	}

	VCP_write(buf, numberOfCharsInBuffer);
}

int32_t Kalman::testExample(void) {
	/* --------------------------------------------------------------------------------
	 * Test input data(Cycles) taken from FIR Q15 module for different cases of blockSize
	 * and tapSize
	 * --------------------------------------------------------------------------------- */
	const float32_t B_f32[4] = { 782.0, 7577.0, 470.0, 4505.0 };
	/* --------------------------------------------------------------------------------
	 * Formula to fit is  C1 + C2 * numTaps + C3 * blockSize + C4 * numTaps * blockSize
	 * -------------------------------------------------------------------------------- */
	const float32_t A_f32[16] = {/* Const,       numTaps,        blockSize,      numTaps*blockSize */
	1.0, 32.0, 4.0, 128.0, 1.0, 32.0, 64.0, 2048.0, 1.0, 16.0, 4.0, 64.0, 1.0,
			16.0, 64.0, 1024.0 };
	/* ----------------------------------------------------------------------
	 * Temporary buffers  for storing intermediate values
	 * ------------------------------------------------------------------- */
	float32_t AT_f32[16]; /* Transpose of A Buffer */
	float32_t ATMA_f32[16]; /* (Transpose of A * A) Buffer */
	float32_t ATMAI_f32[16]; /* Inverse(Transpose of A * A)  Buffer */
	float32_t X_f32[4]; /* Test Output Buffer */
	/* ----------------------------------------------------------------------
	 * Reference ouput buffer C1, C2, C3 and C4 taken from MATLAB
	 * ------------------------------------------------------------------- */
	const float32_t xRef_f32[4] = { 73.0, 8.0, 21.25, 2.875 };
	arm_matrix_instance_f32 A; /* Matrix A Instance */
	arm_matrix_instance_f32 At; /* Matrix AT(A transpose) instance */
	arm_matrix_instance_f32 At_m_A; /* Matrix ATMA( AT multiply with A) instance */
	arm_matrix_instance_f32 At_m_A_I; /* Matrix ATMAI(Inverse of ATMA) instance */
	arm_matrix_instance_f32 B; /* Matrix B instance */
	arm_matrix_instance_f32 X; /* Matrix X(Unknown Matrix) instance */
	arm_status status;
	arm_mat_init_f32(&A, 4, 4, (float32_t*) (A_f32));
	arm_mat_init_f32(&At, 4, 4, AT_f32);
	status = arm_mat_trans_f32(&A, &At);
	arm_mat_init_f32(&At_m_A, 4, 4, ATMA_f32);
	status = arm_mat_mult_f32(&At, &A, &At_m_A);
	arm_mat_init_f32(&At_m_A_I, 4, 4, ATMAI_f32);
	/* calculation of Inverse((Transpose(A) * A) */
	status = arm_mat_inverse_f32(&At_m_A, &At_m_A_I);
	/* calculation of (Inverse((Transpose(A) * A)) *  Transpose(A)) */
	status = arm_mat_mult_f32(&At_m_A_I, &At, &At_m_A);
	/* Initialise B Matrix Instance with numRows, numCols and data array(B_f32) */
	arm_mat_init_f32(&B, 4, 1, (float32_t*) (B_f32));
	/* Initialise X Matrix Instance with numRows, numCols and data array(X_f32) */
	arm_mat_init_f32(&X, 4, 1, X_f32);
	/* calculation ((Inverse((Transpose(A) * A)) *  Transpose(A)) * B) */
	status = arm_mat_mult_f32(&At_m_A, &B, &X);
	sendMatrixViaCom(&X);
	return 0;
}
