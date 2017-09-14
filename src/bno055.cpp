/*
 * Copyright (c) 2016, CATIE, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "bno055.hpp"

namespace sixtron {

namespace {

/* MACROS */
#define	TEMP_SOURCE_ACC                0x00
#define TEMP_SOURCE_GYR                0x01
#define RESET_COMMAND                  0x20
#define RAW_TO_MICRO_TESLA             16.0
#define RAW_TO_METERS_PER_SECOND       100.0
#define RAW_TO_RADIANS                 900.0
#define RAW_TO_UNITARY_QUATERNIONS     16384.0
#define CHIP_ID                        0xA0
#define TIME_TO_RESET                  800

} // namespace

/** Default constructor
 *
 * @param i2c pointer to mbed I2C object
 * @param hz frequency of the I2C interface. Default is 400kHz
 *
 */
BNO055::BNO055(I2C * i2c, I2CAddress address, int hz):
        _i2cAddress(address), _mode(OperationMode::OperationMode_CONFIG)
{
	_i2c = i2c;
    _i2c->frequency(hz);
}

/** Initialize the BNO055
 *
 * @param mode operation mode to be run by the BNO
 * @param UseExtCrystal True if it should use a 32 kHz external crystal to improve the clock precision
 * 		 				False if it should use the internal clock
 *
 * @returns
 *      True on success,
 *      False on failure
 */
bool BNO055::initialize(OperationMode mode, bool use_ext_crystal)
{
	char reg = 0;
	printf("Initializing BNO055 ... \n");
	reset();
	i2c_read_register(RegisterAddress::ChipId, &reg);
	if (reg != CHIP_ID) {
		wait_ms(1000); //BNO055 may have not finishing to boot !
		i2c_read_register(RegisterAddress::ChipId, &reg);
		if (reg != CHIP_ID) {
			return false;
		}
	}

    i2c_set_register(RegisterAddress::PageId, static_cast<char>(PageId::PageZero));
    //Updating BNO055 informations
    i2c_read_register(RegisterAddress::ChipId, &_chipId);
    i2c_read_register(RegisterAddress::AccelRevId, &_accelerometerRevisionId);
    i2c_read_register(RegisterAddress::MagRevId, &_magnetometerRevisionId);
    i2c_read_register(RegisterAddress::GyroRevId, &_gyroscopeRevisionId);
    i2c_read_two_bytes_register(RegisterAddress::SwRevId, &_firmwareVersion);
    i2c_read_register(RegisterAddress::BlRevId, &_bootloaderVersion);

    set_operation_mode(OperationMode::OperationMode_CONFIG);
    wait_ms(20);
    i2c_set_register(RegisterAddress::PwrMode, static_cast<char>(PowerMode::PowerMode_NORMAL));
    wait_ms(10);

    /* Set the output units */
    uint8_t unitsel = (0 << 7) | // Orientation = Android
                      (0 << 4) | // Temperature = Celsius
                      (1 << 2) | // Euler = Rads
                      (1 << 1) | // Gyro = Rads
                      (0 << 0);  // Accelerometer = m/s^2
    i2c_set_register(RegisterAddress::UnitSel, unitsel);

    if (use_ext_crystal) {
    	printf("Using External crystal\n");
    	i2c_set_register(RegisterAddress::SysTrigger, 0X80);
    	wait_ms(10);
    }

    set_operation_mode(mode);
    _mode = mode;
    wait_ms(20);

    return true;
}

/** Set the BNO055 operation mode
 *
 * @param mode Operation mode to be run by the BNO
 *
 */
void BNO055::set_operation_mode(OperationMode mode)
{
	_mode = mode;
	i2c_set_register(RegisterAddress::OprMode, static_cast<char>(mode));
	wait_ms(20);
}

/** Set the BNO055 power mode
 *
 * @parma mode Power mode to be applied
 *
 */
void BNO055::set_power_mode(PowerMode mode)
{
    i2c_set_register(RegisterAddress::PwrMode, static_cast<char>(mode));
}

/** read the accelerometer value
 *
 * @param accel pointer to acceleromter structure to store the read values in m/s²
 *
 */
void BNO055::read_accel(bno055_accel_t* accel)
{
	static int16_t raw_acc[3];
	i2c_read_vector(RegisterAddress::AccelData_X_Lsb, raw_acc);

	accel->x = ((double)raw_acc[0])/RAW_TO_METERS_PER_SECOND;
	accel->y = ((double)raw_acc[1])/RAW_TO_METERS_PER_SECOND;
	accel->z = ((double)raw_acc[2])/RAW_TO_METERS_PER_SECOND;
}

/** read the gyrometer value
 *
 * @param gyro pointer to gyrometer structure to store the read values in rad/s
 *
 */
void BNO055::read_gyro(bno055_gyro_t* gyro)
{
	static int16_t raw_gyro[3];
	i2c_read_vector(RegisterAddress::GyroData_X_Lsb, raw_gyro);

	gyro->x = ((double)raw_gyro[0])/RAW_TO_RADIANS;
	gyro->y = ((double)raw_gyro[1])/RAW_TO_RADIANS;
	gyro->z = ((double)raw_gyro[2])/RAW_TO_RADIANS;
}

void BNO055::read_temperature(bno055_temperature_t *temp)
{
	static char data;

	i2c_set_register(BNO055::RegisterAddress::TempSource, TEMP_SOURCE_ACC); //accelerometer temperature
	wait_ms(1); // \TODO is it necessary ?
	i2c_read_register(BNO055::RegisterAddress::Temp, &data);
	temp->acc = data;

	i2c_set_register(BNO055::RegisterAddress::TempSource, TEMP_SOURCE_GYR); //gyrometer temperature
	wait_ms(1); // \TODO is it necessary ?
	i2c_read_register(BNO055::RegisterAddress::Temp, &data);
	temp->gyro = data;
}

/** read the magnetometer value
 *
 * @param mag pointer to magnetometer structure to store the read values in µT
 *
 */
void BNO055::read_mag(bno055_mag_t* mag)
{
	static int16_t raw_mag[3];
	i2c_read_vector(RegisterAddress::MagData_X_Lsb, raw_mag);

	mag->x = ((double)raw_mag[0])/RAW_TO_MICRO_TESLA;
	mag->y = ((double)raw_mag[1])/RAW_TO_MICRO_TESLA;
	mag->z = ((double)raw_mag[2])/RAW_TO_MICRO_TESLA;
}

/** read the acclerometer value with gravity compensation
 *
 * @param accel pointer to accelerometer structure to store the read values in m/s²
 *
 */
void BNO055::read_linear_accel(bno055_linear_accel_t* accel)
{
	static int16_t raw_acc[3];
	i2c_read_vector(RegisterAddress::LinearAccelData_X_Lsb, raw_acc);

	accel->x = ((double)raw_acc[0])/RAW_TO_METERS_PER_SECOND;
	accel->y = ((double)raw_acc[1])/RAW_TO_METERS_PER_SECOND;
	accel->z = ((double)raw_acc[2])/RAW_TO_METERS_PER_SECOND;
}

/** read the Euler angles value
 *
 * @param euler pointer Euler angles structure to store the read values in rad
 *
 */
void BNO055::read_euler(bno055_euler_t* euler)
{
	static int16_t raw_eul[3];
	i2c_read_vector(RegisterAddress::Euler_H_Lsb, raw_eul);

	euler->x = ((double)raw_eul[0])/RAW_TO_RADIANS;
	euler->y = ((double)raw_eul[1])/RAW_TO_RADIANS;
	euler->z = ((double)raw_eul[2])/RAW_TO_RADIANS;
}

/** read the quaternion value. The output quat in normalized and unitary
 *
 * @param quat pointer to quaternion structure to store the read values
 *
 */
void BNO055::read_quaternion(bno055_quaternion_t* quat)
{
	static char data[8];
	static int16_t raw_quat[4];
    char reg = static_cast<char>(RegisterAddress::QuaternionData_W_Lsb);

    _i2c->write(static_cast<int>(_i2cAddress) << 1, &reg, 1, true);
    _i2c->read(static_cast<int>(_i2cAddress) << 1, data, 8, false);

    raw_quat[0] = (data[1] << 8) | (0xFF & data[0]);
    raw_quat[1] = (data[3] << 8) | (0xFF & data[2]);
    raw_quat[2] = (data[5] << 8) | (0xFF & data[4]);
    raw_quat[3] = (data[7] << 8) | (0xFF & data[6]);

	quat->w = ((double)raw_quat[0])/RAW_TO_UNITARY_QUATERNIONS;
	quat->x = ((double)raw_quat[1])/RAW_TO_UNITARY_QUATERNIONS;
	quat->y = ((double)raw_quat[2])/RAW_TO_UNITARY_QUATERNIONS;
	quat->z = ((double)raw_quat[3])/RAW_TO_UNITARY_QUATERNIONS;
}

/** read the quaternion value.
 *
 * @param quat pointer to quaternion structure to store the read values
 *
 */
void BNO055::read_quaternion(bno055_raw_quaternion_t* quat)
{
	static char data[8];
    char reg = static_cast<char>(RegisterAddress::QuaternionData_W_Lsb);

    _i2c->write(static_cast<int>(_i2cAddress) << 1, &reg, 1, true);
    _i2c->read(static_cast<int>(_i2cAddress) << 1, data, 8, false);

    quat->w = (data[1] << 8) | (0xFF & data[0]);
    quat->x = (data[3] << 8) | (0xFF & data[2]);
    quat->y = (data[5] << 8) | (0xFF & data[4]);
    quat->z = (data[7] << 8) | (0xFF & data[6]);
}

/** read the gravity vector value
 *
 * @param gravity pointer to gravity structure to store the read values in m/s²
 *
 */
void BNO055::read_gravity(bno055_gravity_t* gravity)
{
	static int16_t raw_grav[3];
	i2c_read_vector(RegisterAddress::GravityData_X_Lsb, raw_grav);

	gravity->x = ((double)raw_grav[0])/RAW_TO_METERS_PER_SECOND;
	gravity->y = ((double)raw_grav[1])/RAW_TO_METERS_PER_SECOND;
	gravity->z = ((double)raw_grav[2])/RAW_TO_METERS_PER_SECOND;
}
/** get the calibrations state of the sensors and the system
 *
 * @param sys pointer to store system calibration state. Value between 0 and 3, where 3 indicates a full calibration
 * @param gyro pointer to storesystem calibration state. Value between 0 and 3, where 3 indicates a full calibration
 * @param accel pointer to store system calibration state. Value between 0 and 3, where 3 indicates a full calibration
 * @param mag pointer to store system calibration state. Value between 0 and 3, where 3 indicates a full calibration
 *
 */
void BNO055::get_calibration_status(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag) {
	static char cal_data;
	i2c_read_register(RegisterAddress::CalibStat, &cal_data);
	if (sys != NULL) {
	*sys = (cal_data >> 6) & 0x03;
	}
	if (gyro != NULL) {
	*gyro = (cal_data >> 4) & 0x03;
	}
	if (accel != NULL) {
	*accel = (cal_data >> 2) & 0x03;
	}
	if (mag != NULL) {
	*mag = cal_data & 0x03;
	}
}

/** get the sensor offsets calculated by the fusion algorithm
 *
 * @param sensor_offsets pointer to bno055_offsets_t structure to store the raw sensor offsets
 *
 */
void BNO055::get_sensor_offsets(bno055_offsets_t* sensor_offsets)
{
	static char calib_data[22];
	static char address = static_cast<char>(RegisterAddress::AccelOffset_X_Lsb);
	OperationMode last_mode = _mode;

	set_operation_mode(OperationMode::OperationMode_CONFIG);

	_i2c->write(static_cast<int>(_i2cAddress) << 1, &address, 1, true);
	_i2c->read(static_cast<int>(_i2cAddress) << 1, calib_data, 22);

	sensor_offsets->accel_offset_x = (calib_data[1] << 8) | (calib_data[0]);
	sensor_offsets->accel_offset_y = (calib_data[3] << 8) | (calib_data[2]);
	sensor_offsets->accel_offset_z = (calib_data[5] << 8) | (calib_data[4]);

	sensor_offsets->mag_offset_x = (calib_data[7] << 8) | (calib_data[6]);
	sensor_offsets->mag_offset_y = (calib_data[9] << 8) | (calib_data[8]);
	sensor_offsets->mag_offset_z = (calib_data[11] << 8) | (calib_data[10]);

	sensor_offsets->gyro_offset_x = (calib_data[13] << 8) | (calib_data[12]);
	sensor_offsets->gyro_offset_y = (calib_data[15] << 8) | (calib_data[14]);
	sensor_offsets->gyro_offset_z = (calib_data[17] << 8) | (calib_data[16]);

	sensor_offsets->accel_radius = (calib_data[19] << 8) | (calib_data[18]);
	sensor_offsets->mag_radius = (calib_data[21] << 8) | (calib_data[20]);

	set_operation_mode(last_mode);
}

/** set the sensor offsets given by the user to faster the calibration
 *
 * @param sensor_offsets pointer to bno055_offsets_t structure that has to be written in the BNO055 offsets registers
 *
 */
void BNO055::set_sensor_offsets(const bno055_offsets_t* sensor_offsets)
{
    OperationMode last_mode = _mode;
    char calib_data[22];

    set_operation_mode(OperationMode::OperationMode_CONFIG);

    calib_data[0] = (sensor_offsets->accel_offset_x) & 0x0FF;
    calib_data[1] = (sensor_offsets->accel_offset_x >> 8) & 0x0FF;
    calib_data[2] = (sensor_offsets->accel_offset_y) & 0x0FF;
    calib_data[3] = (sensor_offsets->accel_offset_y >> 8) & 0x0FF;
    calib_data[4] = (sensor_offsets->accel_offset_z) & 0x0FF;
    calib_data[5] = (sensor_offsets->accel_offset_z >> 8) & 0x0FF;

    calib_data[6] = (sensor_offsets->mag_offset_x) & 0x0FF;
    calib_data[7] = (sensor_offsets->mag_offset_x >> 8) & 0x0FF;
    calib_data[8] = (sensor_offsets->mag_offset_y) & 0x0FF;
    calib_data[9] = (sensor_offsets->mag_offset_y >> 8) & 0x0FF;
    calib_data[10] = (sensor_offsets->mag_offset_z) & 0x0FF;
    calib_data[11] = (sensor_offsets->mag_offset_z >> 8) & 0x0FF;

    calib_data[12] = (sensor_offsets->accel_offset_x) & 0x0FF;
    calib_data[13] = (sensor_offsets->accel_offset_x >> 8) & 0x0FF;
    calib_data[14] = (sensor_offsets->accel_offset_y) & 0x0FF;
    calib_data[15] = (sensor_offsets->accel_offset_y >> 8) & 0x0FF;
    calib_data[16] = (sensor_offsets->accel_offset_z) & 0x0FF;
    calib_data[17] = (sensor_offsets->accel_offset_z >> 8) & 0x0FF;

    calib_data[18] = (sensor_offsets->accel_radius) & 0x0FF;
    calib_data[19] = (sensor_offsets->accel_radius >> 8) & 0x0FF;
    calib_data[20] = (sensor_offsets->mag_radius) & 0x0FF;
    calib_data[21] = (sensor_offsets->mag_radius >> 8) & 0x0FF;

    _i2c->write(static_cast<int>(_i2cAddress), calib_data, 22);

    set_operation_mode(last_mode);
}

void BNO055::reset()
{
	i2c_set_register(RegisterAddress::SysTrigger, RESET_COMMAND);
	wait_ms(TIME_TO_RESET);
}
/** Set register value
 *
 * @param registerAddress register address
 * @param value value to write
 *
 * @returns
 *      O on success,
 *      non-0 on failure
 */
int BNO055::i2c_set_register(RegisterAddress registerAddress, char value)
{
    static char data[2];
    data[0] = static_cast<char>(registerAddress);
    data[1] = value;
    if (_i2c->write(static_cast<int>(_i2cAddress) << 1, data, 2, false) != 0) {
        return -1;
    }
    return 0;
}

/** Get register value
 *
 * @param registerAddress register address
 * @param value pointer to store read value to
 *
 * @returns
 *      O on success,
 *      non-0 on failure
 */
int BNO055::i2c_read_register(RegisterAddress registerAddress, char *value)
{
    char data = static_cast<char>(registerAddress);
    if (_i2c->write(static_cast<int>(_i2cAddress) << 1, &data, 1, true) != 0) {
        return -1;
    }
    if (_i2c->read(static_cast<int>(_i2cAddress) << 1, value, 1, false) != 0) {
        return -2;
    }
    return 0;
}

/** Get multi-byte register value (two-bytes)
 *
 * @param registerAddress register address of LSB
 * @param value pointer to store read value to
 *
 * @returns
 *      O on success,
 *      non-0 on failure
 */
int BNO055::i2c_read_two_bytes_register(RegisterAddress registerAddress, short *value)
{
    static char data[2];
    data[0] = static_cast<char>(registerAddress);
    if (_i2c->write(static_cast<int>(_i2cAddress) << 1, data, 1, true) != 0) {
        return -1;
    }
    if (_i2c->read(static_cast<int>(_i2cAddress) << 1, data, 2, false) != 0) {
        return -2;
    }
    *value = (data[1] << 8) | (0xFF & data[0]);
    
    return 0;
}

/** Get multi-byte register value (3*2-bytes) that are stored in a 3 dimensions vector
 *
 * @param registerAddress register address of LSB
 * @param value pointer to store read value to
 *
 * @returns
 *      O on success,
 *      non-0 on failure
 */
int BNO055::i2c_read_vector(RegisterAddress registerAddress, int16_t value[3])
{
    static char data[6];
    data[0] = static_cast<char>(registerAddress);
    if (_i2c->write(static_cast<int>(_i2cAddress) << 1, data, 1, true) != 0) {
        return -1;
    }
    if (_i2c->read(static_cast<int>(_i2cAddress) << 1, data, 6, false) != 0) {
        return -2;
    }
    value[0] = (data[1] << 8) | (0xFF & data[0]);
    value[1] = (data[3] << 8) | (0xFF & data[2]);
    value[2] = (data[5] << 8) | (0xFF & data[4]);

    return 0;
}

} // namespace sixtron