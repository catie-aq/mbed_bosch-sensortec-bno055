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
#include "bno055/bno055.hpp"

namespace {
#define I2C_BUFFER_SIZE_ 32
}

BNO055::BNO055(PinName sda, PinName scl, I2CAddress address, int hz):
        i2c_(sda, scl),
        i2cAddress_(address)
{
    i2c_.frequency(hz);
}

void BNO055::initialize()
{
    i2c_set_register(RegisterAddress::PageId, static_cast<char>(PageId::PageZero));

    i2c_register(RegisterAddress::ChipId, &chipId_);
    i2c_register(RegisterAddress::AccelRevId, &accelerometerRevisionId_);
    i2c_register(RegisterAddress::MagRevId, &magnetometerRevisionId_);
    i2c_register(RegisterAddress::GyroRevId, &gyroscopeRevisionId_);
    i2c_register(RegisterAddress::SwRevId, &firmwareVersion_);
    i2c_register(RegisterAddress::BlRevId, &bootloaderVersion_);
}

int BNO055::i2c_set_register(RegisterAddress registerAddress, char value)
{
    static char data[2];
    data[0] = static_cast<char>(registerAddress);
    data[1] = value;
    if (i2c_.write(static_cast<int>(i2cAddress_), data, 2, false) != 0) {
        return -1;
    }
    return 0;
}

int BNO055::i2c_register(RegisterAddress registerAddress, char *value)
{
    char data = static_cast<char>(registerAddress);
    if (i2c_.write(static_cast<int>(i2cAddress_), &data, 1, true) != 0) {
        return -1;
    }
    if (i2c_.read(static_cast<int>(i2cAddress_), value, 1, false) != 0) {
        return -2;
    }
    return 0;
}

int BNO055::i2c_register(RegisterAddress registerAddress, short *value)
{
    static char data[2];
    data[0] = static_cast<char>(registerAddress);
    if (i2c_.write(static_cast<int>(i2cAddress_), data, 1, true) != 0) {
        return -1;
    }
    if (i2c_.read(static_cast<int>(i2cAddress_), data, 2, false) != 0) {
        return -2;
    }
    *value = (data[1] << 8) | (0xFF & data[0]);
    
    return 0;
}
