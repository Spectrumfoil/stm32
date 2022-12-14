#include "ICM20948_register_addresses.h"
#include "AK09918_register_addresses.h"
#include "ICM20948_register_options.h"
#include <stdbool.h>
#include "stm32f4xx_hal.h"

/**
 * initialize ICM20948
 * initialize AK09918
 * reset ICM20948
 * reset AK09918
 *
 * _read sensors_:
 * read all axises of all Sensors
 * read all axises of Accelerometer
 * read all axises of Gyroscope
 * read all axises of Magnetometer
 * read Temperature
 *
 * _configure sensors_:
 * set Magnetometer data size (boolean full_two_byte_data)
 * set Accelerometer sensitivity
 * set Gyroscope sensitivity
 *
 *
 *
 * void initAK09916(); - Analyze I2c Method inputs to create enums for this.
 * void initICM20948(); - Analyze I2c Method inputs to create enums for this.
 * void readAccelData(int16_t *);
 * void readGyroData(int16_t *);
 * void readMagData(int16_t *);
 * int16_t readTempData();
*/
// For Inits input sensitivity enums
// (int accelerometerConfigEnum, int gyroscopeConfigEnum, int magnetometerConfigEnum, int temperatureConfigEnum)
// void ICM20948_init(void);
// void AK09918_init(void);
// void ICM20948_reset(void);
// void AK09918_reset(void);

//not sure if this type is right
// void ICM20948_readAllSensors_allAxises(int *ax, int *ay, int *az, int *gx, int *gy, int *gz, int *mx, int *my, int *mz, int *t );
// void ICM20948_readAccelerometer_allAxises(int *x, int *y, int *z);
// void ICM20948_readGyroscope_allAxises(int *x, int *y, int *z);
// void ICM20948_readMagnetometer_allAxises(int *x, int *y, int *z);
// void ICM20948_readTemperature(int *t);

// void ICM20948_setAccelerometerSensitivity(int sensitivityEnum);
// void ICM20948_setGyroscopeSensitivity(int sensitivityEnum);
// void ICM20948_setMagnetometerDataSize(bool is16Bit);

uint8_t ICM20948_isI2cAddress1(I2C_HandleTypeDef * hi2c);
uint8_t ICM20948_isI2cAddress2(I2C_HandleTypeDef * hi2c);
void ICM20948_init(I2C_HandleTypeDef * hi2c, uint8_t const selectI2cAddress, uint8_t const selectGyroSensitivity, uint8_t const selectAccelSensitivity);
void ICM20948_readGyroscope_allAxises(I2C_HandleTypeDef * hi2c, uint8_t const selectI2cAddress, uint8_t const selectGyroSensitivity, int16_t readings[3]);
void ICM20948_readTemperature(I2C_HandleTypeDef * hi2c, uint8_t const selectI2cAddress, int16_t * reading);
void ICM20948_readAccelerometer_allAxises(I2C_HandleTypeDef * hi2c, uint8_t const selectI2cAddress, uint8_t const selectAccelSensitivity, int16_t readings[3]);
void ICM20948_readMagnetometer_allAxises(I2C_HandleTypeDef * hi2c, int16_t readings[3]);
