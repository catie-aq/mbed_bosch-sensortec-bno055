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
#ifndef BNO055_H
#define BNO055_H

#include "mbed.h"

namespace sixtron {

/* Raw values of sensors offsets */
typedef struct {
    int16_t accelerometer_offset_x;
    int16_t accelerometer_offset_y;
    int16_t accelerometer_offset_z;
    int16_t gyroscope_offset_x;
    int16_t gyroscope_offset_y;
    int16_t gyroscope_offset_z;
    int16_t magnetometer_offset_x;
    int16_t magnetometer_offset_y;
    int16_t magnetometer_offset_z;
    int16_t accelerometer_radius;
    int16_t magnetometer_radius;
} bno055_sensors_offsets_t;

/* raw acceleration values in m/s^2 */
typedef struct {
    double x;
    double y;
    double z;
} bno055_acceleration_t;

/* accel values with gravity compensated in m/s^2 */
typedef struct {
    double x;
    double y;
    double z;
} bno055_linear_acceleration_t;

/* angular velocity values in rad/s */
typedef struct {
    double x;
    double y;
    double z;
} bno055_angular_velocity_t;

/* mag values in uT */
typedef struct {
    double x;
    double y;
    double z;
} bno055_magnetic_field_t;

/* euler values in rad */
typedef struct {
    double x;
    double y;
    double z;
} bno055_euler_t;

/* unitary quaternion (unitless) */
typedef struct {
    double w;
    double x;
    double y;
    double z;
} bno055_quaternion_t;

/* raw quaternion as read in the register(unitless) */
typedef struct {
    int16_t w;
    int16_t x;
    int16_t y;
    int16_t z;
} bno055_raw_quaternion_t;

/* gravity values in m/s?? */
typedef struct {
    double x;
    double y;
    double z;
} bno055_gravity_t;

/* calibration values */
typedef struct {
    int8_t system;
    int8_t accelerometer;
    int8_t gyroscope;
    int8_t magnetometer;
} bno055_calibration_status_t;

/*!
 *  \class BNO055
 *  BNO055 IMU driver
 */
class BNO055
{
public:
    enum class I2CAddress {
        Address1 = 0x28,
        Address2 = 0x29
    };

    enum class RegisterAddress : char {
        /* Page id register definition */
        PageId                   = (0x07),

        /*
         * Page 0 register definition
         */

        ChipId                  = (0x00),
        AccelRevId              = (0x01),
        MagRevId                = (0x02),
        GyroRevId               = (0x03),
        SwRevId_Lsb             = (0x04),
        SwRevId_Msb             = (0x05),
        SwRevId                 = (SwRevId_Lsb),
        BlRevId                 = (0x06),

        /* Accel data register */
        AccelData_X_Lsb         = (0x08),
        AccelData_X_Msb         = (0x09),
        AccelData_Y_Lsb         = (0x0A),
        AccelData_Y_Msb         = (0x0B),
        AccelData_Z_Lsb         = (0x0C),
        AccelData_Z_Msb         = (0x0D),

        /* Mag data register */
        MagData_X_Lsb           = (0x0E),
        MagData_X_Msb           = (0x0F),
        MagData_Y_Lsb           = (0x10),
        MagData_Y_Msb           = (0x11),
        MagData_Z_Lsb           = (0x12),
        MagData_Z_Msb           = (0x13),

        /* Gyro data registers */
        GyroData_X_Lsb          = (0x14),
        GyroData_X_Msb          = (0x15),
        GyroData_Y_Lsb          = (0x16),
        GyroData_Y_Msb          = (0x17),
        GyroData_Z_Lsb          = (0x18),
        GyroData_Z_Msb          = (0x19),

        /* Euler data registers */
        Euler_H_Lsb              = (0x1A),
        Euler_H_Msb              = (0x1B),

        Euler_R_Lsb              = (0x1C),
        Euler_R_Msb              = (0x1D),

        Euler_P_Lsb              = (0x1E),
        Euler_P_Msb              = (0x1F),

        /*Quaternion data registers */
        QuaternionData_W_Lsb    = (0x20),
        QuaternionData_W_Msb    = (0x21),
        QuaternionData_X_Lsb    = (0x22),
        QuaternionData_X_Msb    = (0x23),
        QuaternionData_Y_Lsb    = (0x24),
        QuaternionData_Y_Msb    = (0x25),
        QuaternionData_Z_Lsb    = (0x26),
        QuaternionData_Z_Msb    = (0x27),

        /* Linear acceleration data registers */
        LinearAccelData_X_Lsb   = (0x28),
        LinearAccelData_X_Msb   = (0x29),
        LinearAccelData_Y_Lsb   = (0x2A),
        LinearAccelData_Y_Msb   = (0x2B),
        LinearAccelData_Z_Lsb   = (0x2C),
        LinearAccelData_Z_Msb   = (0x2D),

        /*Gravity data registers */
        GravityData_X_Lsb       = (0x2E),
        GravityData_X_Msb       = (0x2F),
        GravityData_Y_Lsb       = (0x30),
        GravityData_Y_Msb       = (0x31),
        GravityData_Z_Lsb       = (0x32),
        GravityData_Z_Msb       = (0x33),

        /* Temperature data register */
        Temp                    = (0x34),

        /* Status registers */
        CalibStat               = (0x35),
        SelftestResult          = (0x36),
        IntrStat                = (0x37),
        SysClkStat              = (0x38),
        SysStat                 = (0x39),
        SysErr                  = (0x3A),

        /* Unit selection register */
        UnitSel                 = (0x3B),
        DataSelect              = (0x3C),

        /* Mode registers */
        OprMode                 = (0x3D),
        PwrMode                 = (0x3E),

        SysTrigger              = (0x3F),
        TempSource              = (0x40),
        /* Axis remap registers */
        AxisMapConfig           = (0x41),
        AxisMapSign             = (0x42),

        /* SIC registers */
        SicMatrix0_Lsb          = (0x43),
        SicMatrix0_Msb          = (0x44),
        SicMatrix1_Lsb          = (0x45),
        SicMatrix1_Msb          = (0x46),
        SicMatrix2_Lsb          = (0x47),
        SicMatrix2_Msb          = (0x48),
        SicMatrix3_Lsb          = (0x49),
        SicMatrix3_Msb          = (0x4A),
        SicMatrix4_Lsb          = (0x4B),
        SicMatrix4_Msb          = (0x4C),
        SicMatrix5_Lsb          = (0x4D),
        SicMatrix5_Msb          = (0x4E),
        SicMatrix6_Lsb          = (0x4F),
        SicMatrix6_Msb          = (0x50),
        SicMatrix7_Lsb          = (0x51),
        SicMatrix7_Msb          = (0x52),
        SicMatrix8_Lsb          = (0x53),
        SicMatrix8_Msb          = (0x54),

        /* Accelerometer Offset registers */
        AccelOffset_X_Lsb       = (0x55),
        AccelOffset_X_Msb       = (0x56),
        AccelOffset_Y_Lsb       = (0x57),
        AccelOffset_Y_Msb       = (0x58),
        AccelOffset_Z_Lsb       = (0x59),
        AccelOffset_Z_Msb       = (0x5A),

        /* Magnetometer Offset registers */
        MagOffset_X_Lsb         = (0x5B),
        MagOffset_X_Msb         = (0x5C),
        MagOffset_Y_Lsb         = (0x5D),
        MagOffset_Y_Msb         = (0x5E),
        MagOffset_Z_Lsb         = (0x5F),
        MagOffset_Z_Msb         = (0x60),

        /* Gyroscope Offset registers */
        GyroOffset_X_Lsb        = (0x61),
        GyroOffset_X_Msb        = (0x62),
        GyroOffset_Y_Lsb        = (0x63),
        GyroOffset_Y_Msb        = (0x64),
        GyroOffset_Z_Lsb        = (0x65),
        GyroOffset_Z_Msb        = (0x66),

        /* Radius registers */
        AccelRadius_Lsb         = (0x67),
        AccelRadius_Msb         = (0x68),
        MagRadius_Lsb           = (0x69),
        MagRadius_Msb           = (0x6A),

        /*
         * Page1 registers definition
         */

        /* Configuration registers */
        AccelConfig             = (0x08),
        MagConfig               = (0x09),
        GyroConfig0             = (0x0A),
        GyroConfig1             = (0x0B),
        AccelSleepConfig        = (0x0C),
        GyroSleepConfig         = (0x0D),
        MagSleepConfig          = (0x0E),

        /* Interrupt registers */
        IntMask                 = (0x0F),
        Int                     = (0x10),
        AccelAnyMotionThres     = (0x11),
        AccelIntrSettings       = (0x12),
        AccelHighGDurn          = (0x13),
        AccelHighGThres         = (0x14),
        AccelNoMotionThres      = (0x15),
        AccelNoMotionSet        = (0x16),
        GyroIntrSeting          = (0x17),
        GyroHighrate_X_Set      = (0x18),
        GyroDurn_X              = (0x19),
        GyroHighrate_Y_Set      = (0x1A),
        GyroDurn_Y              = (0x1B),
        GyroHighrate_Z_Set      = (0x1C),
        GyroDurn_Z              = (0x1D),
        GyroAnyMotionThres      = (0x1E),
        GyroAnyMotionSet        = (0x1F)
    };

    enum class PageId : char {
        PageZero                = (0X00),
        PageOne                 = (0X01),
    };

    enum class PowerMode : char {
        NORMAL        = (0X00),
        LOWPOWER      = (0X01),
        SUSPEND       = (0X02)
    };

    enum class OperationMode : char {
        /* Operation mode settings*/
        CONFIG                           = 0X00,
        ACCONLY                          = 0X01,
        MAGONLY                          = 0X02,
        GYRONLY                          = 0X03,
        ACCMAG                           = 0X04,
        ACCGYRO                          = 0X05,
        MAGGYRO                          = 0X06,
        AMG                              = 0X07,
        IMUPLUS                          = 0X08,
        COMPASS                          = 0X09,
        M4G                              = 0X0A,
        NDOF_FMC_OFF                     = 0X0B,
        NDOF                             = 0X0C
    };

    enum class AccelerometerSensorRange : uint8_t {
        /* Accelerometer sensor range */
        _2G         = 0x00,
        _4G         = 0x01,
        _8G         = 0x02,
        _16G        = 0x03
    };

    enum class AccelerometerSensorBandWidth : uint8_t {
        /* Accelerometer sensor bandwidth */
        _7Hz        = 0x00,
        _15Hz       = 0x04,
        _31Hz       = 0x8,
        _62Hz       = 0xC,
        _125Hz      = 0x10,
        _250Hz      = 0x14,
        _500Hz      = 0x18,
        _1000Hz     = 0x1C
    };

    enum class AccelerometerSensorOperationMode : uint8_t {
        /* Accelerometer sensor Operating Mode */
        Normal          = 0x00,
        Suspend         = 0x20,
        LowPower1       = 0x40,
        Standby         = 0x60,
        LowPower2       = 0x80,
        DeepSuspend     = 0xA0
    };

    enum class GyroscopeSensorRange : uint8_t {
        /* Gyroscope sensor range */
        _2000DPS        = 0x00,
        _1000DPS        = 0x01,
        _500DPS         = 0x02,
        _250DPS         = 0x03,
        _125DPS         = 0x04
    };

    enum class GyroscopeSensorBandWidth : uint8_t {
        /* Gyroscope sensor bandwidth */
        _523Hz          = 0x00,
        _230Hz          = 0x08,
        _116Hz          = 0x10,
        _47Hz           = 0x18,
        _23Hz           = 0x20,
        _12Hz           = 0x28,
        _64Hz           = 0x30,
        _32Hz           = 0x38
    };

    enum class GyroscopeSensorOperationMode : uint8_t {
        /* Gyroscope sensor operating mode */
        Normal          = 0x00,
        FastPowerUp     = 0x01,
        DeepSuspend     = 0x02
    };

    enum class MagnetometerSensorDataOutputRate : uint8_t {
        /* Magnetometer sensor Data output rate */
        _2Hz            = 0x00,
        _6Hz            = 0x01,
        _8Hz            = 0x02,
        _10Hz           = 0x03,
        _15Hz           = 0x04,
        _20Hz           = 0x05,
        _25Hz           = 0x06,
        _30Hz           = 0x07
    };

    enum class MagnetometerSensorOperationMode : uint8_t {
        /* Magnetometer sensor operating mode */
        LowPower            = 0x00,
        Regular             = 0x08,
        EnhancedRegular     = 0x10,
        HighAccuracy        = 0x18
    };

    enum class MagnetometerSensorPowerMode : uint8_t {
        /* Magnetometer sensor power mode */
        Normal              = 0x00,
        Sleep               = 0x02,
        Suspend             = 0x04,
        ForceMode           = 0x06
    };

    enum class AccelerationInterruptMode : uint8_t {
        /* Acceleration interrupt mode */
        NoMotion            = 0x80,
        AnyMotion           = 0x40,
        HighAcceleration    = 0x20
    };

    enum class AccelerationInterruptAxisMap : uint8_t {
        /* Acceleration interrupt mapping axis */
        X                   = 0x01,
        Y                   = 0x02,
        Z                   = 0x04,
        XY                  = (X | Y),
        XZ                  = (X | Z),
        YZ                  = (Y | Z),
        XYZ                 = (X | Y | Z)
    };

    enum class AccelerationInterrutpPinMask : uint8_t {
        /* Acceleration interrupt settings */
        NoMotionPinMask             = 0x80,
        AnyMotionPinMask            = 0x40,
        HighAccelerationPinMask     = 0x20,
        NoPinMask                   = 0x00
    };

    /*! Default constructor
     *
     * \param i2c pointer to mbed I2C object
     * \param address i2c address of the bno055
     * \param hz frequency of the I2C interface
     *
     */
    BNO055(I2C *i2c, I2CAddress address = I2CAddress::Address1, int hz = 400000);

    /*! Constructor
     *
     * \param i2c pointer to mbed I2C object
     * \param interrupt_pin interrupt pin name
     * \param address i2c address of the bno055
     * \param hz frequency of the I2C interface
     *
     */
    BNO055(I2C *i2c, PinName interrupt_pin, I2CAddress address = I2CAddress::Address1, int hz = 400000);


    /*! Initialize the BNO055
     *
     * \param mode operation mode to be run by the BNO
     * \param use_ext_crystal use external 32 kHz crystal
     *
     * \returns True on success,
     *          False on failure
     */
    bool initialize(OperationMode mode = OperationMode::NDOF, bool use_ext_crystal = false);

    /*! Set the BNO055 operation mode
     *
     * \param mode Operation mode to be run by the BNO
     *
     */
    void set_operation_mode(OperationMode mode);

    /*! Set the BNO055 power mode
     *
     * \param mode Power mode to be applied
     *
     */
    void set_power_mode(PowerMode mode);

    /*! Set BNO055 accelerometer configuration
     *
     * \param range acceleration 2g/4g/8g/16g
     * \param bandwidth Low-pass filter 7.81Hz/15.63Hz/31.25Hz/62.5Hz/125Hz/250Hz/500Hz/1000Hz
     * \param operation_mode associated in accelerometer (Normal/Suspend/LowPower1/Standby/LowPower2/DeepSuspend)
     *
     */
    void set_accelerometer_configuration(AccelerometerSensorRange range, AccelerometerSensorBandWidth bandwidth,
            AccelerometerSensorOperationMode operation_mode);

    /** Set BNO055  range accelerometer configuration
     *
     * \param range acceleration 2g/4g/8g/16g
     *
     */
    void set_accelerometer_range(AccelerometerSensorRange range);

    /*! Set BNO055  bandwidth accelerometer configuration
     *
     * \param bandwidth Low-pass filter 7.81Hz/15.63Hz/31.25Hz/62.5Hz/125Hz/250Hz/500Hz/1000Hz
     *
     */
    void set_accelerometer_bandwidth(AccelerometerSensorBandWidth bandwidth);

    /*! Set BNO055  operating mode accelerometer configuration
     *
     * \param operation_mode associated in accelerometer (Normal/Suspend/LowPower1/Standby/LowPower2/DeepSuspend)
     *
     */
    void set_accelerometer_operation_mode(AccelerometerSensorOperationMode operation_mode);

    /*! Set BNO055 gyroscope configuration
     *
     * \param range gyroscope 2000dps/1000dps/500dps/250dps/125dps
     * \param bandwidth Low-pass filter 523Hz/230Hz/116Hz/47Hz/23Hz/12Hz/64Hz/32Hz
     * \param operation_mode associated in gyroscope (Normal/FastPowerUp/DeepSuspend/Suspend/AdvancedPowersave)
     *
     */
    void set_gyroscope_configuration(GyroscopeSensorRange range, GyroscopeSensorBandWidth bandwidth,
            GyroscopeSensorOperationMode operation_mode);

    /*! Set BNO055  range gyroscope configuration
     *
     * \param range gyroscope 2000dps/1000dps/500dps/250dps/125dps
     *
     */
    void set_gyroscope_range(GyroscopeSensorRange range);

    /*! Set BNO055  bandwidth gyroscope configuration
     *
     * \param bandwidth Low-pass filter 523Hz/230Hz/116Hz/47Hz/23Hz/12Hz/64Hz/32Hz
     *
     */
    void set_gyroscope_bandwidth(GyroscopeSensorBandWidth bandwidth);

    /*! Set BNO055  operating mode gyroscope configuration
     *
     * \param operation_mode associated in gyroscope (Normal/FastPowerUp/DeepSuspend/Suspend/AdvancedPowersave)
     *
     */
    void set_gyroscope_operation_mode(GyroscopeSensorOperationMode operation_mode);

    /*! Set BNO055 magnetometer configuration
     *
     * \param data_output_rate 2Hz/6Hz/8Hz/10Hz/15Hz/20Hz/25Hz/30Hz
     * \param operation _ode associated in magnetometer (LowPower/Regular/EnhancedRegular/HighAccuracy)
     * \param power_mode Normal/Sleep/Suspend/Force
     *
     */
    void set_magnetometer_configuration(MagnetometerSensorDataOutputRate data_output_rate,
            MagnetometerSensorOperationMode operation_mode,
            MagnetometerSensorPowerMode power_mode);

    /*! Set BNO055  data output rate magnetometer configuration
     *
     * \param data_output_rate 2Hz/6Hz/8Hz/10Hz/15Hz/20Hz/25Hz/30Hz
     *
     */
    void set_magnetometer_data_output_rate(MagnetometerSensorDataOutputRate data_output_rate);

    /*! Set BNO055  operating mode magnetometer configuration
     *
     * \param operation_mode LowPower/Regular/EnhancedRegular/HighAccuracy
     *
     */
    void set_magnetometer_operation_mode(MagnetometerSensorOperationMode operation_mode);

    /*! Set BNO055  power_mode magnetometer configuration
     *
     * \param power mode Normal/Sleep/Suspend/Force
     *
     */
    void set_magnetometer_power_mode(MagnetometerSensorPowerMode power_mode);

    /*! Get BNO055 operating mode
     *
     * \return current operating mode of device
     *
     */
    OperationMode operating_mode();


    /*! Set BNO055 page ID
     *
     * \param page pointed on the the next registers page
     *
     */
    void set_pageID(PageId page);

    /*! Get BNO055 page ID
     *
     * \return the current page ID location
     *
     */
    PageId pageID(void);

    /*! Get the acceleration values
     *
     * \returns the acceleration values structure in m/s??
     *
     */
    bno055_acceleration_t acceleration();

    /*! Get the angular velocity values
     *
     * \returns the angular velocity values structure in rad/s
     *
     */
    bno055_angular_velocity_t angular_velocity();

    /*! Get the magnetic field values
     *
     * \returns the magnetic field values structure  in ??T
     *
     */
    bno055_magnetic_field_t magnetic_field();

    /*! Select the temperature source
     *
     *  @param temperature_source : The value of selected temperature source
     *
     *     temperature_source | result
     *    ----------------  |---------------
     *      0x00            | BNO055_ACCEL_TEMP_EN
     *      0X01            | BNO055_GYRO_TEMP_EN
     *      0X03            | BNO055_MCU_TEMP_EN
     *
     */
    void set_temperature_source(uint8_t temperature_source);

    /*! Get internal sensor temperature according to temperature source
     *
     * \returns the temperature in ??C
     *
     */
    int8_t temperature_sensor();

    /*! Get the accelerometer value with gravity compensation
     *
     * \returns the accelerometer values structure with gravity compensation in m/s??
     *
     */
    bno055_linear_acceleration_t linear_acceleration();

    /*! Get the Euler angles value
     *
     * \returns the Euler angles values structure in rad
     *
     */
    bno055_euler_t euler();

    /*! Get the quaternion value. The output quat in normalized and unitary
     *
     * \returns the quaternion values structure
     *
     */
    bno055_quaternion_t quaternion();

    /*! Get the quaternion raw value.
     *
     * \returns the quaternion raw values structure
     *
     */
    bno055_raw_quaternion_t raw_quaternion();

    /*! Get the gravity vector value
     *
     * \returns the gravity values structure in m/s??
     *
     */
    bno055_gravity_t gravity();

    /*! Get the calibrations state of the sensors and the system
     *
     * \returns the system calibration state structure of each device (sys, acc, gyro, mag). Value between 0 and 3, where 3 indicates a full calibration
     *
     */
    bno055_calibration_status_t calibration_status();

    /*! Get the sensor offsets calculated by the fusion algorithm
     *
     * \return the raw sensor offsets values structure
     *
     */
    bno055_sensors_offsets_t sensor_offsets();

    /*! set the sensor offsets given by the user to faster the calibration
     *
     * \param sensor_offsets pointer to bno055_offsets_t structure that has to be written in the BNO055 offsets registers
     *
     */
    void set_sensor_offsets(const bno055_sensors_offsets_t *sensor_offsets);

    /*! Reset the bno055. All register values goes back to default
     * values and calibrations values are lost
     */
    void reset();

    /*! Get the BNO055 chip ID
     *
     * \return BNO055 chip ID
     *
     */
    char chip_id();

    /*! Get the BNO055 accelerometer revision ID
     *
     * \return BNO055 accelerometer revision ID
     *
     */
    char accelerometer_revision_id();

    /*! Get the BNO055 magnetometer revision ID
     *
     * \return BNO055 magnetometer revision ID
     *
     */
    char magnetometer_revision_id();

    /*! Get the BNO055 gyroscope revision ID
     *
     * \return BNO055 gyroscope revision ID
     *
     */
    char gyroscope_revision_id();

    /*! Get the BNO055 firmware version
     *
     * \return BNO055 firmware version
     *
     */
    short firmware_version();

    /*! Get the BNO055 bootloader version
     *
     * \return BNO055 bootloader version
     *
     */
    char bootloader_version();


    /*! Set BNO055 Enable High-G acceleration interrupt
     *
     * \param map_axis the axis mapping to interrupt will be available
     * \param acceleratgion_threshold configure the interrupt threshold in accord with current range configured (percentage of current range)
     * \param interrupt_duration duration of interrupt level
     * \param enable_mask_interrupt_pin the mask to define if the interrupt pin is map to interrupt event, false to set as none
     *
     */
    void enable_highAcceleration_interrupt(AccelerationInterruptAxisMap map_axis, uint8_t acceleration_threshold,
            uint8_t interrupt_duration, bool enable_mask_interrupt_pin = false);

    /*! Set BNO055: Enable No-Motion acceleration interrupt
     *
     * \param map_axis the axis mapping to interrupt will be available
     * \param acceleratgion_threshold configure the interrupt threshold in accord with current range configured (percentage of current range)
     * \param interrupt_duration duration of interrupt level
     * \param enable_mask_interrupt_pin the mask to define if the interrupt pin is map to interrupt event, false to set as none
     *
     */
    void enable_noMotion_acceleration_interrupt(AccelerationInterruptAxisMap map_axis, uint8_t acceleration_threshold,
            uint8_t interrupt_duration, bool enable_mask_interrupt_pin =  false);

    /*! Set BNO055 Enable Any-Motion acceleration interrupt
     *
     * \param map_axis the axis mapping to interrupt will be available
     * \param acceleratgion_threshold configure the interrupt threshold in accord with current range configured (percentage of current range)
     * \param interrupt_duration duration of interrupt level
     * \param enable_mask_interrupt_pin the mask to define if the interrupt pin is map to interrupt event, false to set as none
     *
     */
    void enable_anyMotion_acceleration_interrupt(AccelerationInterruptAxisMap map_axis, uint8_t acceleration_threshold,
            uint8_t interrupt_duration, bool enable_mask_interrupt_pin = false);

    /*! Set BNO055 Enable Any-Motion acceleration interrupt
     *
     * \param acceleration_interrupt_mode the type of acceleration mode
     *
     */
    void disable_acceleration_interrupt(AccelerationInterruptMode acceleration_interrupt_mode);

    /*! Set BNO055 acceleration interrupt callback
     *
     * \param func A pointer to a void function, or 0 to set as none
     *
     */
    void acceleration_interrupt_callback(Callback<void()> func);

    /*! BNO055 acceleration interrupt
     *
     * \return the register value of interrupt status
     *
     */
    uint8_t acceleration_interrupt();

    /*! BNO055 clear interrupt flag
     *
     */
    void clear_interrupt_flag();

    /*! BNO055 system status
     *
     * \return the status of system (see p.68 of datasheet)
     *
     */
    uint8_t system_status();

    /*! BNO055 system error
     *
     * \return the error of system (see p.69 of datasheet)
     *
     */
    uint8_t system_error();

private:

    /*! Set register value
     *
     * \param registerAddress register address
     * \param value, value to write
     *
     * \returns 0 on success,
     *          no-0 on failure
     */
    int i2c_set_register(RegisterAddress registerAddress, char value);

    /*! Get register value
     *
     * \param registerAddress register address
     * \param value pointer to store read value to
     *
     * \returns 0 on success,
     *          no-0 on failure
     */
    int i2c_read_register(RegisterAddress registerAddress, char *value);

    /*! Get multi-byte register value (two-bytes)
     *
     * \param registerAddress register address of LSB
     * \param value pointer to store read value to
     *
     * \returns 0 on success,
     *          no-0 on failure
     */
    int i2c_read_two_bytes_register(RegisterAddress registerAddress, short *value);

    /*! Get multi-byte register value (3*2-bytes) that are stored in a 3 dimensions vector
     *
     * \param registerAddress register address of LSB
     * \param value pointer to store read value to
     *
     * \returns 0 on success,
     *          no-0 on failure
     */
    int i2c_read_vector(RegisterAddress registerAddress, int16_t value[3]);

    I2C *_i2c;
    I2CAddress _i2cAddress;
    OperationMode _mode;
    PageId _currentPageID;
    InterruptIn _interrupt_pin;

    char _chipId = 0;
    char _accelerometerRevisionId = 0;
    char _magnetometerRevisionId = 0;
    char _gyroscopeRevisionId = 0;
    short _firmwareVersion = 0;
    char _bootloaderVersion = 0;

};

} // namespace sixtron

#endif // BNO055_H
