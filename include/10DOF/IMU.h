#ifndef IMU_H_
#define IMU_H_
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "arm_math.h"

#include "../NokiaLCD/nokiaLCD.h"
#include "../Delay/delay.h"

#include "I2C.h"
#include "itg3200.h"
#include "adxl345.h"
#include "hmc5883l.h"
#include "bmp085.h"
#include "Filters/Kalman.h"
#include "GPS/gps.h"

class IMU {
public:

private:
	/* Sensors and measurment parameters */
	ITG3200 gyro;
	ADXL345 accelerometer;

	HMC5883L magnetometer;
	BMP085 pressure;
	uint16_t samplingFrequencyInHz;
	float32_t samplingPeriodInS;
	const uint8_t I2C_ID_BMP085 = 0x77 << 1;		//Barometer? Id Address

	/* Status */
	volatile uint8_t newRawDataAvailable;
	uint8_t connected;
	uint8_t request;
	uint8_t error;
	volatile uint8_t semahpore_computationInProgress;

	/*	Position */
	struct gpsData_t gps;
	float32_t positionInGlobalFrame_IMU[3];
	float32_t positionDeltaInGlobalFrame_IMU[3];
	float32_t velocityInGlobalFrame[3];		//	in m/s
	float32_t accelerationInGlobalFrame[3];	//in m/s*s

	/*	Filter */
	Kalman kalmanX;
	Kalman kalmanY;
	float32_t XRollAngleInRad;	//Range -180,180
	float32_t YPitchAngleInRad;	//Range -90,90
	float32_t TiltAngleInRad;		//Range 0,180	//odchylenie od pionu (grawitacji)
	float32_t ZYawAngleInRad;	//Range -180,180
	float32_t eulerAnglesInRadMahony[3];

	/* All the angles start at 180 degrees */
	float64_t gyroXangle = 180;
	float64_t gyroYangle = 180;
	float64_t compAngleX = 180;
	float64_t compAngleY = 180;

	/* Temporary buffers */
	//volatile int16_t measurements[6][9];	//nie używane - do usuniecia!
	volatile uint16_t numberOfGatheredSamples;
	uint16_t numberOfSamplesToGather;
#define IMU_BUF_SIZE 300
	uint8_t buf[IMU_BUF_SIZE];

	inline void __attribute__((always_inline)) initializeTimerForUpdate(void);

	/*  */
	void doAllComputation(void);
	/* Only from accelerometer without any filtering. */
	void computePitchRollTilt(void);
	/* Only from magnetometer with use of computed raw euler angles from accelerometer. */
	void computeYaw(void);

	void removeCentrifugalForceEffect(void);
	void transformAccelerationToGlobalFrame(void);
	void computeMovementAndAddToPossition(void);
public:
	IMU();
	~IMU() {
	}
	TIM_HandleTypeDef TimHandle;
	void initialize(void);

	/* Diagnostic function */
	void __attribute__((always_inline)) selfTests(NokiaLCD &nokiaLCD);
	void calibrateGyroAndAccStationary(void);

	/* main actions - computation */
	void timerAction(void);
	void mahonyStepAction(void);
	void kalmanStepAction(void);
	void updateGPSData(const struct gpsData_t *_gpsdata);
	void updatePositionFromIMU(void);
	void setPossitionFromGPSToIMU(void);

	/* Logging, debugging and sending functions */
	void logDataOnSD(void);
	void sendGatheredDataViaVCOM(void);
	void showAnglesKalman(NokiaLCD& nokiaLCD);
	void showMeasurment(NokiaLCD& nokiaLCD);
	void prepareDataFrame(uint8_t * const pBuff, int16_t buffSize);

	/* State changing functions */
	void startTimerUpdate(void);
	void stopTimerUpdate(void);
	void setConnected(void);
	void setDisconnected(void);
	void setRequestOfData(void);
	void startDataGathering(void);
	void requestDataGathering(uint16_t numberOfSamples);
	uint8_t isDataGatheringComplete(void);
	void zerosAllComputedValues(void);
};

extern IMU imu10DOF;
#endif
