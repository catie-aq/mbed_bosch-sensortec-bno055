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
#include "bno055.h"

namespace sixtron {

namespace {

/* MACROS */
#define TEMP_SOURCE_ACC                0x00
#define TEMP_SOURCE_GYR                0x01
#define RESET_COMMAND                  0x20
#define RAW_TO_MICRO_TESLA             16.0
#define RAW_TO_METERS_PER_SECOND       100.0
#define RAW_TO_RADIANS                 900.0
#define RAW_TO_UNITARY_QUATERNIONS     16384.0
#define CHIP_ID                        0xA0
#define TIME_TO_RESET                  800

} // namespace

BNO055::BNO055(I2C *i2c, I2CAddress address, int hz):
    _i2cAddress(address), _mode(OperationMode::CONFIG), _currentPageID(PageId::PageZero)
{
    _i2c = i2c;
    _i2c->frequency(hz);
}

bool BNO055::initialize(OperationMode mode, bool use_ext_crystal)
{
    char reg = 0;

    reset();
    i2c_read_register(RegisterAddress::ChipId, &reg);
    if (reg != CHIP_ID) {
        wait_ms(1000); //BNO055 may have not finishing to boot !
        i2c_read_register(RegisterAddress::ChipId, &reg);
        if (reg != CHIP_ID) {
            return false;
        }
    }

    //Updating BNO055 informations
    i2c_read_register(RegisterAddress::ChipId, &_chipId);
    i2c_read_register(RegisterAddress::AccelRevId, &_accelerometerRevisionId);
    i2c_read_register(RegisterAddress::MagRevId, &_magnetometerRevisionId);
    i2c_read_register(RegisterAddress::GyroRevId, &_gyroscopeRevisionId);
    i2c_read_two_bytes_register(RegisterAddress::SwRevId, &_firmwareVersion);
    i2c_read_register(RegisterAddress::BlRevId, &_bootloaderVersion);

    set_operation_mode(OperationMode::CONFIG);
    wait_ms(20);
    i2c_set_register(RegisterAddress::PwrMode, static_cast<char>(PowerMode::NORMAL));
    wait_ms(10);

    /* Set the output units */
    uint8_t unitsel = (0 << 7) | // Orientation = Android
            (0 << 4) | // Temperature = Celsius
            (1 << 2) | // Euler = Rads
            (1 << 1) | // Gyro = Rads
            (0 << 0);  // Accelerometer = m/s^2
    i2c_set_register(RegisterAddress::UnitSel, unitsel);

    if (use_ext_crystal) {
        i2c_set_register(RegisterAddress::SysTrigger, 0X80);
        wait_ms(10);
    }

    set_operation_mode(mode);
    wait_ms(20);

    return true;
}

void BNO055::set_accel_configuration(AccSensorRange range, AccSensorBandWidth bandwidth,
        AccSensorOpeMode operation_mode)
{
    char reg_val = 0x00;
    // check if current page = pageID 1
    if (_currentPageID != PageId::PageOne) {
        //go to pageID 1
        set_pageID(PageId::PageOne);
    }
    // get user accel config
    reg_val |= (static_cast<char>(range) | static_cast<char>(bandwidth) | static_cast<char>(operation_mode));
    //set accel conf register
    i2c_set_register(RegisterAddress::AccelConfig, reg_val);
}

void BNO055::set_accel_range(AccSensorRange range)
{
    char reg = 0x00;
    // check if current page = pageID 1
    if (_currentPageID != PageId::PageOne) {
        //go to pageID 1
        set_pageID(PageId::PageOne);
    }
    // read acc config register
    i2c_read_register(RegisterAddress::AccelConfig, &reg);
    // fix new configuration
    reg |= static_cast<char>(range);
    //set new register value
    i2c_set_register(RegisterAddress::AccelConfig, reg);
}

void BNO055::set_accel_bandwidth(AccSensorBandWidth bandwidth)
{
    char reg = 0x00;
    // check if current page = pageID 1
    if (_currentPageID != PageId::PageOne) {
        //go to pageID 1
        set_pageID(PageId::PageOne);
    }
    // read acc config register
    i2c_read_register(RegisterAddress::AccelConfig, &reg);
    // fix new configuration
    reg |= static_cast<char>(bandwidth);
    //set new register value
    i2c_set_register(RegisterAddress::AccelConfig, reg);
}

void BNO055::set_accel_operation_mode(AccSensorOpeMode operation_mode)
{
    char reg = 0x00;
    // check if current page = pageID 1
    if (_currentPageID != PageId::PageOne) {
        //go to pageID 1
        set_pageID(PageId::PageOne);
    }
    // read acc config register
    i2c_read_register(RegisterAddress::AccelConfig, &reg);
    // fix new configuration
    reg |= static_cast<char>(operation_mode);
    //set new register value
    i2c_set_register(RegisterAddress::AccelConfig, reg);
}

void BNO055::set_gyro_configuration(GyroSensorRange range, GyroSensorBandWidth bandwidth,
        GyroSensorOpeMode operation_mode)
{
    char reg_val = 0x00;
    // check if current page = pageID 1
    if (_currentPageID != PageId::PageOne) {
        //go to pageID 1
        set_pageID(PageId::PageOne);
    }
    // get user gyro config for config0 register
    reg_val |= (static_cast<char>(range) | static_cast<char>(bandwidth));
    // set new value register for gyro_conf0 register
    i2c_set_register(RegisterAddress::GyroConfig0, reg_val);
    // get user gyro config for config1 register
    reg_val |= (0x00 | static_cast<char>(operation_mode));
    // set new value register for config1_register
    i2c_set_register(RegisterAddress::GyroConfig1, reg_val);
}

void BNO055::set_gyro_range(GyroSensorRange range)
{
    char reg = 0x00;
    // check if current page = pageID 1
    if (_currentPageID != PageId::PageOne) {
        //go to pageID 1
        set_pageID(PageId::PageOne);
    }
    // read gyro config register
    i2c_read_register(RegisterAddress::GyroConfig0, &reg);
    // fix new configuration
    reg |= static_cast<char>(range);
    //set new register value
    i2c_set_register(RegisterAddress::GyroConfig0, reg);
}

void BNO055::set_gyro_bandwidth(GyroSensorBandWidth bandwidth)
{
    char reg = 0x00;
    // check if current page = pageID 1
    if (_currentPageID != PageId::PageOne) {
        //go to pageID 1
        set_pageID(PageId::PageOne);
    }
    // read gyro config register
    i2c_read_register(RegisterAddress::GyroConfig0, &reg);
    // fix new configuration
    reg |= static_cast<char>(bandwidth);
    //set new register value
    i2c_set_register(RegisterAddress::GyroConfig0, reg);
}

void BNO055::set_gyro_operation_mode(GyroSensorOpeMode operation_mode)
{
    char reg = 0x00;
    // check if current page = pageID 1
    if (_currentPageID != PageId::PageOne) {
        //go to pageID 1
        set_pageID(PageId::PageOne);
    }
    // read gyro config register
    i2c_read_register(RegisterAddress::GyroConfig1, &reg);
    // fix new configuration
    reg |= static_cast<char>(operation_mode);
    //set new register value
    i2c_set_register(RegisterAddress::GyroConfig1, reg);
}

void BNO055::set_mag_configuration(MagSensorDataOutputRate data_output_rate, MagSensorOpeMode operation_mode,
        MagSensorPowerMode power_mode)
{
    char reg_val = 0x00;
    // check if current page = pageID 1
    if (_currentPageID != PageId::PageOne) {
        //go to pageID 1
        set_pageID(PageId::PageOne);
    }
    // get user mag config
    reg_val |= (static_cast<char>(data_output_rate) | static_cast<char>(operation_mode) | static_cast<char>(power_mode));
    //set mag conf register
    i2c_set_register(RegisterAddress::MagConfig, reg_val);
}

void BNO055::set_mag_data_output_rate(MagSensorDataOutputRate data_output_rate)
{
    char reg = 0x00;
    // check if current page = pageID 1
    if (_currentPageID != PageId::PageOne) {
        //go to pageID 1
        set_pageID(PageId::PageOne);
    }
    // read mag config register
    i2c_read_register(RegisterAddress::MagConfig, &reg);
    // fix new configuration
    reg |= static_cast<char>(data_output_rate);
    //set new register value
    i2c_set_register(RegisterAddress::MagConfig, reg);
}

void BNO055::set_mag_operation_mode(MagSensorOpeMode operation_mode)
{
    char reg = 0x00;
    // check if current page = pageID 1
    if (_currentPageID != PageId::PageOne) {
        //go to pageID 1
        set_pageID(PageId::PageOne);
    }
    // read mag config register
    i2c_read_register(RegisterAddress::MagConfig, &reg);
    // fix new configuration
    reg |= static_cast<char>(operation_mode);
    //set new register value
    i2c_set_register(RegisterAddress::MagConfig, reg);
}

void BNO055::set_mag_power_mode(MagSensorPowerMode power_mode)
{
    char reg = 0x00;
    // check if current page = pageID 1
    if (_currentPageID != PageId::PageOne) {
        //go to pageID 1
        set_pageID(PageId::PageOne);
    }
    // read mag config register
    i2c_read_register(RegisterAddress::MagConfig, &reg);
    // fix new configuration
    reg |= static_cast<char>(power_mode);
    //set new register value
    i2c_set_register(RegisterAddress::MagConfig, reg);
}

/** Get BNO055 operating mode
 *
 * @return : current operating mode of objet
 *
 */
BNO055::OperationMode BNO055::operating_mode()
{
    return (_mode);
}

void BNO055::set_pageID(PageId _page)
{
    i2c_set_register(RegisterAddress::PageId, static_cast<char>(_page));
    _currentPageID = _page;
}

/** Get BNO055 operating mode
 *
 * @return : the current page ID location
 *
 */
BNO055::PageId BNO055::pageID(void)
{
    return (_currentPageID);
}

void BNO055::set_operation_mode(OperationMode mode)
{
    // check current pageID
    if (_currentPageID != PageId::PageZero) {
        //go to pageID 0
        set_pageID(PageId::PageZero);
    }
    _mode = mode;
    i2c_set_register(RegisterAddress::OprMode, static_cast<char>(mode));
    wait_ms(20);
}

void BNO055::set_power_mode(PowerMode mode)
{
    // check current pageID
    if (_currentPageID != PageId::PageZero) {
        //go to pageID 0
        set_pageID(PageId::PageZero);
    }
    i2c_set_register(RegisterAddress::PwrMode, static_cast<char>(mode));
}

/** read the accelerometer value
 *
 * @param accel pointer to acceleromter structure to store the read values in m/s²
 *
 */
bno055_accel_t BNO055::accel()
{
    static int16_t raw_acc[3];
    bno055_accel_t accel_val;
    // check current pageID
    if (_currentPageID != PageId::PageZero) {
        //go to pageID 0
        set_pageID(PageId::PageZero);
    }
    // read accel data register
    i2c_read_vector(RegisterAddress::AccelData_X_Lsb, raw_acc);
    // formatting accel data
    accel_val.x = ((double)raw_acc[0]) / RAW_TO_METERS_PER_SECOND;
    accel_val.y = ((double)raw_acc[1]) / RAW_TO_METERS_PER_SECOND;
    accel_val.z = ((double)raw_acc[2]) / RAW_TO_METERS_PER_SECOND;
    // return accel values
    return accel_val;
}

/** read the gyrometer value
 *
 * @param gyro pointer to gyrometer structure to store the read values in rad/s
 *
 */
bno055_gyro_t BNO055::gyro()
{
    static int16_t raw_gyro[3];
    bno055_gyro_t gyro_val;
    // check current pageID
    if (_currentPageID != PageId::PageZero) {
        //go to pageID 0
        set_pageID(PageId::PageZero);
    }
    // read gyro data register
    i2c_read_vector(RegisterAddress::GyroData_X_Lsb, raw_gyro);
    // formatting gyro val
    gyro_val.x = ((double)raw_gyro[0]) / RAW_TO_RADIANS;
    gyro_val.y = ((double)raw_gyro[1]) / RAW_TO_RADIANS;
    gyro_val.z = ((double)raw_gyro[2]) / RAW_TO_RADIANS;
    // return gyro values
    return gyro_val;
}

/** read internal sensors temperatures
 *
 * @param temp pointer to temperature structure that store acceleromter
 *            and gyrometer temperature
 */
bno055_temperature_t BNO055::temperature()
{
    static char data;
    bno055_temperature_t temp;
    // check current pageID
    if (_currentPageID != PageId::PageZero) {
        //go to pageID 0
        set_pageID(PageId::PageZero);
    }
    // read temp source acc
    i2c_set_register(BNO055::RegisterAddress::TempSource, TEMP_SOURCE_ACC); //accelerometer temperature
    wait_ms(1); // \TODO is it necessary ?
    i2c_read_register(BNO055::RegisterAddress::Temp, &data);
    temp.acc = data;
    // read temp source gyro
    i2c_set_register(BNO055::RegisterAddress::TempSource, TEMP_SOURCE_GYR); //gyrometer temperature
    wait_ms(1); // \TODO is it necessary ?
    i2c_read_register(BNO055::RegisterAddress::Temp, &data);
    temp.gyro = data;
    // return temperature values
    return temp;
}

/** read the magnetometer value
 *
 * @param mag pointer to magnetometer structure to store the read values in µT
 *
 */
bno055_mag_t BNO055::mag()
{
    static int16_t raw_mag[3];
    bno055_mag_t mag_val;
    // check current pageID
    if (_currentPageID != PageId::PageZero) {
        //go to pageID 0
        set_pageID(PageId::PageZero);
    }
    // read mag data register
    i2c_read_vector(RegisterAddress::MagData_X_Lsb, raw_mag);
    // formatting mag data
    mag_val.x = ((double)raw_mag[0]) / RAW_TO_MICRO_TESLA;
    mag_val.y = ((double)raw_mag[1]) / RAW_TO_MICRO_TESLA;
    mag_val.z = ((double)raw_mag[2]) / RAW_TO_MICRO_TESLA;
    // return mag values
    return mag_val;
}

/** read the acclerometer value with gravity compensation
 *
 * @param accel pointer to accelerometer structure to store the read values in m/s²
 *
 */
bno055_linear_accel_t BNO055::linear_accel()
{
    static int16_t raw_acc[3];
    bno055_linear_accel_t linear_accel_val;
    // check current pageID
    if (_currentPageID != PageId::PageZero) {
        //go to pageID 0
        set_pageID(PageId::PageZero);
    }
    // read linear accel data register
    i2c_read_vector(RegisterAddress::LinearAccelData_X_Lsb, raw_acc);
    // formatting linear accel data
    linear_accel_val.x = ((double)raw_acc[0]) / RAW_TO_METERS_PER_SECOND;
    linear_accel_val.y = ((double)raw_acc[1]) / RAW_TO_METERS_PER_SECOND;
    linear_accel_val.z = ((double)raw_acc[2]) / RAW_TO_METERS_PER_SECOND;
    // return linear accel values
    return linear_accel_val;
}

/** read the Euler angles value
 *
 * @param euler pointer Euler angles structure to store the read values in rad
 *
 */
bno055_euler_t BNO055::euler()
{
    static int16_t raw_eul[3];
    bno055_euler_t euler_val;
    // check current pageID
    if (_currentPageID != PageId::PageZero) {
        //go to pageID 0
        set_pageID(PageId::PageZero);
    }
    // read euler data refister
    i2c_read_vector(RegisterAddress::Euler_H_Lsb, raw_eul);
    // formatting euler data
    euler_val.x = ((double)raw_eul[0]) / RAW_TO_RADIANS;
    euler_val.y = ((double)raw_eul[1]) / RAW_TO_RADIANS;
    euler_val.z = ((double)raw_eul[2]) / RAW_TO_RADIANS;
    // return euler data
    return euler_val;
}

/** read the quaternion value. The output quat in normalized and unitary
 *
 * @param quat pointer to quaternion structure to store the read values
 *
 */
bno055_quaternion_t BNO055::quaternion()
{
    static char data[8];
    static int16_t raw_quat[4];
    bno055_quaternion_t quaternion_val;
    char reg = static_cast<char>(RegisterAddress::QuaternionData_W_Lsb);
    // check current pageID
    if (_currentPageID != PageId::PageZero) {
        //go to pageID 0
        set_pageID(PageId::PageZero);
    }
    _i2c->write(static_cast<int>(_i2cAddress) << 1, &reg, 1, true);
    _i2c->read(static_cast<int>(_i2cAddress) << 1, data, 8, false);
    // formatting data
    raw_quat[0] = (data[1] << 8) | (0xFF & data[0]);
    raw_quat[1] = (data[3] << 8) | (0xFF & data[2]);
    raw_quat[2] = (data[5] << 8) | (0xFF & data[4]);
    raw_quat[3] = (data[7] << 8) | (0xFF & data[6]);

    quaternion_val.w = ((double)raw_quat[0]) / RAW_TO_UNITARY_QUATERNIONS;
    quaternion_val.x = ((double)raw_quat[1]) / RAW_TO_UNITARY_QUATERNIONS;
    quaternion_val.y = ((double)raw_quat[2]) / RAW_TO_UNITARY_QUATERNIONS;
    quaternion_val.z = ((double)raw_quat[3]) / RAW_TO_UNITARY_QUATERNIONS;

    return quaternion_val;
}

/** read the quaternion value.
 *
 * @param quat pointer to quaternion structure to store the read values
 *
 */
bno055_raw_quaternion_t BNO055::raw_quaternion()
{
    static char data[8];
    char reg = static_cast<char>(RegisterAddress::QuaternionData_W_Lsb);
    bno055_raw_quaternion_t raw_quaternion_val;
    // check current pageID
    if (_currentPageID != PageId::PageZero) {
        //go to pageID 0
        set_pageID(PageId::PageZero);
    }
    _i2c->write(static_cast<int>(_i2cAddress) << 1, &reg, 1, true);
    _i2c->read(static_cast<int>(_i2cAddress) << 1, data, 8, false);

    raw_quaternion_val.w = (data[1] << 8) | (0xFF & data[0]);
    raw_quaternion_val.x = (data[3] << 8) | (0xFF & data[2]);
    raw_quaternion_val.y = (data[5] << 8) | (0xFF & data[4]);
    raw_quaternion_val.z = (data[7] << 8) | (0xFF & data[6]);
    // return raw quaternion values
    return raw_quaternion_val;
}

/** read the gravity vector value
 *
 * @param gravity pointer to gravity structure to store the read values in m/s²
 *
 */
bno055_gravity_t BNO055::gravity()
{
    static int16_t raw_grav[3];
    bno055_gravity_t gravity_val;
    // check current pageID
    if (_currentPageID != PageId::PageZero) {
        //go to pageID 0
        set_pageID(PageId::PageZero);
    }
    // read gravity data register
    i2c_read_vector(RegisterAddress::GravityData_X_Lsb, raw_grav);
    // formatting data
    gravity_val.x = ((double)raw_grav[0]) / RAW_TO_METERS_PER_SECOND;
    gravity_val.y = ((double)raw_grav[1]) / RAW_TO_METERS_PER_SECOND;
    gravity_val.z = ((double)raw_grav[2]) / RAW_TO_METERS_PER_SECOND;
    // return gravity values
    return gravity_val;
}

/** get the calibrations state of the sensors and the system
 *
 * @param sys pointer to store system calibration state. Value between 0 and 3, where 3 indicates a full calibration
 * @param gyro pointer to storesystem calibration state. Value between 0 and 3, where 3 indicates a full calibration
 * @param accel pointer to store system calibration state. Value between 0 and 3, where 3 indicates a full calibration
 * @param mag pointer to store system calibration state. Value between 0 and 3, where 3 indicates a full calibration
 *
 */
bno055_calibration_t BNO055::calibration_status()
{
    static char cal_data;
    bno055_calibration_t calibration_val;
    // check current pageID
    if (_currentPageID != PageId::PageZero) {
        //go to pageID 0
        set_pageID(PageId::PageZero);
    }
    // read calibration status register
    i2c_read_register(RegisterAddress::CalibStat, &cal_data);
    // formatting data
    calibration_val.sys = (cal_data >> 6) & 0x03;
    calibration_val.gyro = (cal_data >> 4) & 0x03;
    calibration_val.acc = (cal_data >> 2) & 0x03;
    calibration_val.mag = cal_data & 0x03;
    // return calibration values
    return calibration_val;
}

/** get the sensor offsets calculated by the fusion algorithm
 *
 * @param sensor_offsets pointer to bno055_offsets_t structure to store the raw sensor offsets
 *
 */
bno055_offsets_t BNO055::sensor_offsets()
{
    static char calib_data[22];
    static char address = static_cast<char>(RegisterAddress::AccelOffset_X_Lsb);
    bno055_offsets_t sensor_offset_val;
    OperationMode last_mode = _mode;

    set_operation_mode(OperationMode::CONFIG);

    _i2c->write(static_cast<int>(_i2cAddress) << 1, &address, 1, true);
    _i2c->read(static_cast<int>(_i2cAddress) << 1, calib_data, 22);

    sensor_offset_val.accel_offset_x = (calib_data[1] << 8) | (calib_data[0]);
    sensor_offset_val.accel_offset_y = (calib_data[3] << 8) | (calib_data[2]);
    sensor_offset_val.accel_offset_z = (calib_data[5] << 8) | (calib_data[4]);

    sensor_offset_val.mag_offset_x = (calib_data[7] << 8) | (calib_data[6]);
    sensor_offset_val.mag_offset_y = (calib_data[9] << 8) | (calib_data[8]);
    sensor_offset_val.mag_offset_z = (calib_data[11] << 8) | (calib_data[10]);

    sensor_offset_val.gyro_offset_x = (calib_data[13] << 8) | (calib_data[12]);
    sensor_offset_val.gyro_offset_y = (calib_data[15] << 8) | (calib_data[14]);
    sensor_offset_val.gyro_offset_z = (calib_data[17] << 8) | (calib_data[16]);

    sensor_offset_val.accel_radius = (calib_data[19] << 8) | (calib_data[18]);
    sensor_offset_val.mag_radius = (calib_data[21] << 8) | (calib_data[20]);

    set_operation_mode(last_mode);

    return sensor_offset_val;
}

void BNO055::set_sensor_offsets(const bno055_offsets_t *sensor_offsets)
{
    OperationMode last_mode = _mode;
    char calib_data[22];

    set_operation_mode(OperationMode::CONFIG);

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
    // check current pageID
    if (_currentPageID != PageId::PageZero) {
        //go to pageID 0
        set_pageID(PageId::PageZero);
    }
    i2c_set_register(RegisterAddress::SysTrigger, RESET_COMMAND);
    // set the default mode value
    _mode = OperationMode::CONFIG;
    wait_ms(TIME_TO_RESET);
}

char BNO055::chip_id()
{
    return _chipId;
}

char BNO055::accelerometer_revision_id()
{
    return _accelerometerRevisionId;
}

char BNO055::magnetometer_revision_id()
{
    return _magnetometerRevisionId;
}

char BNO055::gyroscope_revision_id()
{
    return _gyroscopeRevisionId;
}

short BNO055::firmware_version()
{
    return _firmwareVersion;
}

char BNO055::bootloader_version()
{
    return _bootloaderVersion;
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
