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
//#include "mbed-drivers/v2/I2C.hpp" // TODO Use I2C asynchronous API

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
        GyroConfig              = (0x0A),
        GyroModeConfig          = (0x0B),
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

    enum class PageId: char {
        PageZero                = (0X00),
        PageOne                 = (0X01)
    };
    
    enum class PowerMode: char {
    	PowerMode_NORMAL		=(0X00),
    	PowerMode_LOWPOWER		=(0X01),
		PowerMode_SUSPEND		=(0X02)
    };

    enum class OperationMode: char {
        /* Operation mode settings*/
        OperationMode_CONFIG                           = 0X00,
        OperationMode_ACCONLY                          = 0X01,
        OperationMode_MAGONLY                          = 0X02,
        OperationMode_GYRONLY                          = 0X03,
        OperationMode_ACCMAG                           = 0X04,
        OperationMode_ACCGYRO                          = 0X05,
        OperationMode_MAGGYRO                          = 0X06,
        OperationMode_AMG                              = 0X07,
        OperationMode_IMUPLUS                          = 0X08,
        OperationMode_COMPASS                          = 0X09,
        OperationMode_M4G                              = 0X0A,
        OperationMode_NDOF_FMC_OFF                     = 0X0B,
        OperationMode_NDOF                             = 0X0C
    };

    BNO055(I2C * i2c, I2CAddress address = I2CAddress::Address1, int hz = 400000);

    bool initialize(OperationMode mode = OperationMode::OperationMode_NDOF, bool UseExtCristal = false);
    void set_mode(OperationMode mode);
    void getCalibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag);

    char chip_id() { return _chipId; }
    char accelerometer_revision_id() { return _accelerometerRevisionId; }
    char magnetometer_revision_id() { return _magnetometerRevisionId; }
    char gyroscope_revision_id() { return _gyroscopeRevisionId; }
    short firmware_version() { return _firmwareVersion; }
    char bootloader_version() { return _bootloaderVersion; }

private:
    /** Set register value
     *
     * @param registerAddress register address
     * @param value value to write
     *
     * @returns
     *      O on success,
     *      non-0 on failure
     */
    int i2c_set_register(RegisterAddress registerAddress, char value);

    /** Get register value
     *
     * @param registerAddress register address
     * @param value pointer to store read value to
     *
     * @returns
     *      O on success,
     *      non-0 on failure
     */
    int i2c_read_register(RegisterAddress registerAddress, char *value);

    /** Get multi-byte register value (two-bytes)
     *
     * @param registerAddress register address of LSB
     * @param value pointer to store read value to
     *
     * @returns
     *      O on success,
     *      non-0 on failure
     */
    int i2c_read_two_bytes_register(RegisterAddress registerAddress, short *value);

    I2C _i2c;
    I2CAddress _i2cAddress;
    OperationMode _mode;


    char _chipId = 0;
    char _accelerometerRevisionId = 0;
    char _magnetometerRevisionId = 0;
    char _gyroscopeRevisionId = 0;
    short _firmwareVersion = 0;
    char _bootloaderVersion = 0;

};

#endif
