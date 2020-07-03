/* 09/23/2017 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Butterfly default), respectively, and it uses the Butterfly STM32L433CU Breakout Board.
  The ICM42605 is a sensor hub with embedded accel and gyro, here used as 6 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/

#include "ICM42605.h"
#include "I2Cdev.h"

ICM42605::ICM42605(I2Cdev* i2c_bus)
{
  _i2c_bus = i2c_bus;
}


uint8_t ICM42605::getChipID()
{
  uint8_t c = _i2c_bus->readByte(ICM42605_ADDRESS, ICM42605_WHO_AM_I);
  return c;
}


float ICM42605::getAres(uint8_t Ascale) {
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    case AFS_2G:
      _aRes = 2.0f / 32768.0f;
      return _aRes;
      break;
    case AFS_4G:
      _aRes = 4.0f / 32768.0f;
      return _aRes;
      break;
    case AFS_8G:
      _aRes = 8.0f / 32768.0f;
      return _aRes;
      break;
    case AFS_16G:
      _aRes = 16.0f / 32768.0f;
      return _aRes;
      break;
  }
}

float ICM42605::getGres(uint8_t Gscale) {
  switch (Gscale)
  {
    case GFS_15_125DPS:
      _gRes = 15.125f / 32768.0f;
      return _gRes;
      break;
    case GFS_31_25DPS:
      _gRes = 31.25f / 32768.0f;
      return _gRes;
      break;
    case GFS_62_5DPS:
      _gRes = 62.5f / 32768.0f;
      return _gRes;
      break;
    case GFS_125DPS:
      _gRes = 125.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_250DPS:
      _gRes = 250.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_500DPS:
      _gRes = 500.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_1000DPS:
      _gRes = 1000.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_2000DPS:
      _gRes = 2000.0f / 32768.0f;
      return _gRes;
      break;
  }
}


void ICM42605::reset()
{
  // reset device
  uint8_t temp = _i2c_bus->readByte(ICM42605_ADDRESS, ICM42605_DEVICE_CONFIG);
  _i2c_bus->writeByte(ICM42605_ADDRESS, ICM42605_DEVICE_CONFIG, temp | 0x01); // Set bit 0 to 1 to reset ICM42605
  delay(1); // Wait for all registers to reset
}


uint8_t ICM42605::status()
{
  // reset device
  uint8_t temp = _i2c_bus->readByte(ICM42605_ADDRESS, ICM42605_INT_STATUS);
  return temp;
}


void ICM42605::init(uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR)
{
  uint8_t temp = _i2c_bus->readByte(ICM42605_ADDRESS, ICM42605_PWR_MGMT0); // make sure not to distrub reserved bit values
  _i2c_bus->writeByte(ICM42605_ADDRESS, ICM42605_PWR_MGMT0, temp | 0x0F);  // enable gyro and accel in low noise mode

   temp = _i2c_bus->readByte(ICM42605_ADDRESS, ICM42605_GYRO_CONFIG0);
  _i2c_bus->writeByte(ICM42605_ADDRESS, ICM42605_GYRO_CONFIG0, temp | GODR | Gscale << 5); // gyro full scale and data rate

   temp = _i2c_bus->readByte(ICM42605_ADDRESS, ICM42605_ACCEL_CONFIG0);
  _i2c_bus->writeByte(ICM42605_ADDRESS, ICM42605_ACCEL_CONFIG0, temp | AODR | Ascale << 5); // set accel full scale and data rate

   temp = _i2c_bus->readByte(ICM42605_ADDRESS, ICM42605_GYRO_CONFIG1);
  _i2c_bus->writeByte(ICM42605_ADDRESS, ICM42605_GYRO_CONFIG1, temp | 0xD0); // set temperature sensor low pass filter to 5Hz, use first order gyro filter

   temp = _i2c_bus->readByte(ICM42605_ADDRESS, ICM42605_INT_CONFIG);
  _i2c_bus->writeByte(ICM42605_ADDRESS, ICM42605_INT_CONFIG, temp | 0x18 | 0x03 ); // set both interrupts active high, push-pull, pulsed

   temp = _i2c_bus->readByte(ICM42605_ADDRESS, ICM42605_INT_CONFIG1);
  _i2c_bus->writeByte(ICM42605_ADDRESS, ICM42605_INT_CONFIG1, temp & ~(0x10) ); // set bit 4 to zero for proper function of INT1 and INT2
 
   temp = _i2c_bus->readByte(ICM42605_ADDRESS, ICM42605_INT_SOURCE0);
  _i2c_bus->writeByte(ICM42605_ADDRESS, ICM42605_INT_SOURCE0, temp | 0x08 ); // route data ready interrupt to INT1
 
   temp = _i2c_bus->readByte(ICM42605_ADDRESS, ICM42605_INT_SOURCE3);
  _i2c_bus->writeByte(ICM42605_ADDRESS, ICM42605_INT_SOURCE3, temp | 0x01 ); // route AGC interrupt interrupt to INT2

  // Select Bank 4
   temp = _i2c_bus->readByte(ICM42605_ADDRESS, ICM42605_REG_BANK_SEL);
  _i2c_bus->writeByte(ICM42605_ADDRESS, ICM42605_REG_BANK_SEL, temp | 0x04 ); // select Bank 4

     temp = _i2c_bus->readByte(ICM42605_ADDRESS, ICM42605_APEX_CONFIG5);
  _i2c_bus->writeByte(ICM42605_ADDRESS, ICM42605_APEX_CONFIG5, temp & ~(0x07) ); // select unitary mounting matrix

   temp = _i2c_bus->readByte(ICM42605_ADDRESS, ICM42605_REG_BANK_SEL);
  _i2c_bus->writeByte(ICM42605_ADDRESS, ICM42605_REG_BANK_SEL, temp & ~(0x07) ); // select Bank 0
}

/*
void ICM42605::selfTest()
{
  int16_t temp[7] = {0, 0, 0, 0, 0, 0, 0};
  int16_t accelPTest[3] = {0, 0, 0}, accelNTest[3] = {0, 0, 0}, gyroPTest[3] = {0, 0, 0}, gyroNTest[3] = {0, 0, 0};
  int16_t accelNom[3] = {0, 0, 0}, gyroNom[3] = {0, 0, 0};

  readData(temp);
  accelNom[0] = temp[4];
  accelNom[1] = temp[5];
  accelNom[2] = temp[6];
  gyroNom[0]  = temp[1];
  gyroNom[1]  = temp[2];
  gyroNom[2]  = temp[3];

  _i2c_bus->writeByte(ICM42605_ADDRESS, ICM42605_CTRL5_C, 0x01); // positive accel self test
  delay(100); // let accel respond
  readData(temp);
  accelPTest[0] = temp[4];
  accelPTest[1] = temp[5];
  accelPTest[2] = temp[6];

  _i2c_bus->writeByte(ICM42605_ADDRESS, ICM42605_CTRL5_C, 0x03); // negative accel self test
  delay(100); // let accel respond
  readData(temp);
  accelNTest[0] = temp[4];
  accelNTest[1] = temp[5];
  accelNTest[2] = temp[6];

  _i2c_bus->writeByte(ICM42605_ADDRESS, ICM42605_CTRL5_C, 0x04); // positive gyro self test
  delay(100); // let gyro respond
  readData(temp);
  gyroPTest[0] = temp[1];
  gyroPTest[1] = temp[2];
  gyroPTest[2] = temp[3];

  _i2c_bus->writeByte(ICM42605_ADDRESS, ICM42605_CTRL5_C, 0x0C); // negative gyro self test
  delay(100); // let gyro respond
  readData(temp);
  gyroNTest[0] = temp[1];
  gyroNTest[1] = temp[2];
  gyroNTest[2] = temp[3];

  _i2c_bus->writeByte(ICM42605_ADDRESS, ICM42605_CTRL5_C, 0x00); // normal mode
  delay(100); // let accel and gyro respond

  Serial.println("Accel Self Test:");
  Serial.print("+Ax results:"); Serial.print(  (accelPTest[0] - accelNom[0]) * _aRes * 1000.0); Serial.println(" mg");
  Serial.print("-Ax results:"); Serial.println((accelNTest[0] - accelNom[0]) * _aRes * 1000.0);
  Serial.print("+Ay results:"); Serial.println((accelPTest[1] - accelNom[1]) * _aRes * 1000.0);
  Serial.print("-Ay results:"); Serial.println((accelNTest[1] - accelNom[1]) * _aRes * 1000.0);
  Serial.print("+Az results:"); Serial.println((accelPTest[2] - accelNom[2]) * _aRes * 1000.0);
  Serial.print("-Az results:"); Serial.println((accelNTest[2] - accelNom[2]) * _aRes * 1000.0);
  Serial.println("Should be between 90 and 1700 mg");

  Serial.println("Gyro Self Test:");
  Serial.print("+Gx results:"); Serial.print((gyroPTest[0] - gyroNom[0]) * _gRes); Serial.println(" dps");
  Serial.print("-Gx results:"); Serial.println((gyroNTest[0] - gyroNom[0]) * _gRes);
  Serial.print("+Gy results:"); Serial.println((gyroPTest[1] - gyroNom[1]) * _gRes);
  Serial.print("-Gy results:"); Serial.println((gyroNTest[1] - gyroNom[1]) * _gRes);
  Serial.print("+Gz results:"); Serial.println((gyroPTest[2] - gyroNom[2]) * _gRes);
  Serial.print("-Gz results:"); Serial.println((gyroNTest[2] - gyroNom[2]) * _gRes);
  Serial.println("Should be between 20 and 80 dps");
  delay(2000);


}

*/
void ICM42605::offsetBias(float * dest1, float * dest2)
{
  int16_t temp[7] = {0, 0, 0, 0, 0, 0, 0};
  int32_t sum[7] = {0, 0, 0, 0, 0, 0, 0};

  Serial.println("Calculate accel and gyro offset biases: keep sensor flat and motionless!");
  delay(4000);

  for (int ii = 0; ii < 128; ii++)
  {
    readData(temp);
    sum[1] += temp[1];
    sum[2] += temp[2];
    sum[3] += temp[3];
    sum[4] += temp[4];
    sum[5] += temp[5];
    sum[6] += temp[6];
    delay(50);
  }

  dest1[0] = sum[1] * _aRes / 128.0f;
  dest1[1] = sum[2] * _aRes / 128.0f;
  dest1[2] = sum[3] * _aRes / 128.0f;
  dest2[0] = sum[4] * _gRes / 128.0f;
  dest2[1] = sum[5] * _gRes / 128.0f;
  dest2[2] = sum[6] * _gRes / 128.0f;

  if (dest1[0] > 0.8f)  {
    dest1[0] -= 1.0f; // Remove gravity from the x-axis accelerometer bias calculation
  }
  if (dest1[0] < -0.8f) {
    dest1[0] += 1.0f; // Remove gravity from the x-axis accelerometer bias calculation
  }
  if (dest1[1] > 0.8f)  {
    dest1[1] -= 1.0f; // Remove gravity from the y-axis accelerometer bias calculation
  }
  if (dest1[1] < -0.8f) {
    dest1[1] += 1.0f; // Remove gravity from the y-axis accelerometer bias calculation
  }
  if (dest1[2] > 0.8f)  {
    dest1[2] -= 1.0f; // Remove gravity from the z-axis accelerometer bias calculation
  }
  if (dest1[2] < -0.8f) {
    dest1[2] += 1.0f; // Remove gravity from the z-axis accelerometer bias calculation
  }

}


void ICM42605::readData(int16_t * destination)
{
  uint8_t rawData[14];  // x/y/z accel register data stored here
  _i2c_bus->readBytes(ICM42605_ADDRESS, ICM42605_TEMP_DATA1, 14, &rawData[0]);  // Read the 14 raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
  destination[3] = ((int16_t)rawData[6] << 8) | rawData[7] ;
  destination[4] = ((int16_t)rawData[8] << 8) | rawData[9] ;
  destination[5] = ((int16_t)rawData[10] << 8) | rawData[11] ;
  destination[6] = ((int16_t)rawData[12] << 8) | rawData[13] ;
}

