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

BNO055::BNO055(I2C * i2c, I2CAddress address, int hz):
        _i2cAddress(address), _mode(OperationMode::OPERATION_MODE_CONFIG)
{
	_i2c = i2c;
    _i2c.frequency(hz);
}

bool BNO055::initialize(OperationMode mode = OperationMode::OperationMode_NDOF, bool UseExtCristal)
{
	char reg = 0;
	i2c_read_register(RegisterAddress::ChipId, &reg);
	if (reg != 0XA0) {
		wait_ms(1000); //BNO055 may have not finishing to boot !
		i2c_read_register(RegisterAddress::ChipId, &reg);
		if (reg != 0XA0) {
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

    i2c_set_register(RegisterAddress::OprMode, static_cast<char>(OperationMode::OperationMode_CONFIG));
    wait_ms(20);
    i2c_set_register(RegisterAddress::PwrMode, static_cast<char>(PowerMode::PowerMode_NORMAL));
    wait_ms(10);

    if (UseExtCristal) {
    	i2c_set_register(RegisterAddress::SysTrigger, 0X80);
    	wait_ms(10);
    }
    i2c_set_register(RegisterAddress::OprMode, static_cast<char>(mode));
	_mode = mode;
    wait_ms(20);

    return true;
}

void BNO055::set_mode(OperationMode mode)
{
	_mode = mode;
	i2c_set_register(RegisterAddress::OprMode, static_cast<char>(mode));
	wait_ms(20);
}

int BNO055::i2c_set_register(RegisterAddress registerAddress, char value)
{
    static char data[2];
    data[0] = static_cast<char>(registerAddress);
    data[1] = value;
    if (_i2c.write(static_cast<int>(_i2cAddress) << 1, data, 2, false) != 0) {
        return -1;
    }
    return 0;
}

int BNO055::i2c_read_register(RegisterAddress registerAddress, char *value)
{
    char data = static_cast<char>(registerAddress);
    if (_i2c.write(static_cast<int>(_i2cAddress) << 1, &data, 1, true) != 0) {
        return -1;
    }
    if (_i2c.read(static_cast<int>(_i2cAddress) << 1, value, 1, false) != 0) {
        return -2;
    }
    return 0;
}

int BNO055::i2c_read_two_bytes_register(RegisterAddress registerAddress, short *value)
{
    static char data[2];
    data[0] = static_cast<char>(registerAddress);
    if (_i2c.write(static_cast<int>(_i2cAddress) << 1, data, 1, true) != 0) {
        return -1;
    }
    if (_i2c.read(static_cast<int>(_i2cAddress) << 1, data, 2, false) != 0) {
        return -2;
    }
    *value = (data[1] << 8) | (0xFF & data[0]);
    
    return 0;
}
