/*
 * This file is part of INAV.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/resource.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"

// FIXME: Hack until we rework SPI driver
#define BUS_SPI1    SPIDEV_1
#define BUS_SPI2    SPIDEV_2
#define BUS_SPI3    SPIDEV_3
#define BUS_SPI4    SPIDEV_4

typedef enum {
    BUS_SPEED_INITIALIZATION = 0,
    BUS_SPEED_SLOW           = 1,
    BUS_SPEED_STANDARD       = 2,
    BUS_SPEED_FAST           = 3,
    BUS_SPEED_ULTRAFAST      = 4
} busSpeed_e;

typedef enum {
    BUSTYPE_ANY  = 0,
    BUSTYPE_NONE = 0,
    BUSTYPE_I2C  = 1,
    BUSTYPE_SPI  = 2
} busType_e;

/* Ultimately all hardware descriptors will go to target definition files. 
 * Driver code will merely query for it's HW descriptor and initialize it */
typedef enum {
    DEVHW_NONE = 0,
    DEVHW_MPU6000,
    DEVHW_MAX7456,
    DEVHW_BMP280
} devHardwareType_e;

typedef struct busDeviceDescriptor_s {
    busType_e           busType;
    devHardwareType_e   devHwType;
    union {
        struct {
            SPIDevice   spiBus;
            ioTag_t     csnPin;
        } spi;
        struct {
            I2CDevice   i2cBus;
            uint8_t     address;
        } i2c;
    } busdev;
    ioTag_t irqPin;
} busDeviceDescriptor_t;

typedef struct busDevice_s {
    const busDeviceDescriptor_t * descriptor;
    busType_e busType;              // Bus type 
    union {
        struct {
            SPIDevice spiBus;       // SPI bus ID
            IO_t csnPin;            // IO for CS# pin
        } spi;
        struct {
            I2CDevice i2cBus;       // I2C bus ID
            uint8_t address;        // I2C bus device address
        } i2c;
    } busdev;
    IO_t irqPin;                    // Device IRQ pin. Bus system will only assign IO_t object to this var. Initialization is up to device driver
    uint32_t scratchpad;            // Memory where device driver can store persistent data. Zeroed out when initializing the device for the first time
                                    // Useful when once device is shared between several sensors (like MPU/ICM acc-gyro sensors)
} busDevice_t;

#ifdef __APPLE__
extern const busDeviceDescriptor_t __busdev_registry_start[] __asm("section$start$__DATA$__busdev_registry");
extern const busDeviceDescriptor_t __busdev_registry_end[] __asm("section$end$__DATA$__busdev_registry");
#define BUSDEV_REGISTER_ATTRIBUTES __attribute__ ((section("__DATA,__busdev_registry"), used, aligned(4)))
#else
extern const busDeviceDescriptor_t __busdev_registry_start[];
extern const busDeviceDescriptor_t __busdev_registry_end[];
#define BUSDEV_REGISTER_ATTRIBUTES __attribute__ ((section(".busdev_registry"), used, aligned(4)))
#endif

#define BUSDEV_REGISTER_SPI(_name, _devHw, _spiBus, _csnPin, _irqPin)               \
    extern const busDeviceDescriptor_t _name ## _registry;                          \
    const busDeviceDescriptor_t _name ## _registry BUSDEV_REGISTER_ATTRIBUTES = {   \
        .busType = BUSTYPE_SPI,                                                     \
        .devHwType = _devHw,                                                        \
        .busdev.spi = {                                                             \
            .spiBus = _spiBus,                                                      \
            .csnPin = IO_TAG(_csnPin)                                               \
        },                                                                          \
        .irqPin = IO_TAG(_irqPin)                                                   \
    };                                                                              \
    /**/


/* Internal abstraction function */
bool i2cBusWriteRegister(const busDevice_t * dev, uint8_t reg, uint8_t data);
bool i2cBusReadBuffer(const busDevice_t * dev, uint8_t reg, uint8_t * data, uint8_t length);
bool i2cBusReadRegister(const busDevice_t * dev, uint8_t reg, uint8_t * data);


void spiBbusSetSpeed(const busDevice_t * dev, busSpeed_e speed);
bool spiBusWriteRegister(const busDevice_t * dev, uint8_t reg, uint8_t data);
bool spiBusReadBuffer(const busDevice_t * dev, uint8_t reg, uint8_t * data, uint8_t length);
bool spiBusReadRegister(const busDevice_t * dev, uint8_t reg, uint8_t * data);

/* Pre-initialize all known device descriptors to make sure hardware state is consistent and known 
 * Initialize bus hardware */
void busInit(void);

/* Finds a device in registry. First matching device is returned. Also performs the low-level initialization of the hardware (CS line for SPI) */
busDevice_t * busDeviceInit(busType_e bus, devHardwareType_e hw, resourceOwner_e owner);
busDevice_t * busDeviceOpen(busType_e bus, devHardwareType_e hw, resourceOwner_e owner);
void busDeviceDeInit(busDevice_t * dev);

void busSetSpeed(const busDevice_t * dev, busSpeed_e speed);
bool busWriteBuf(const busDevice_t * busdev, uint8_t reg, uint8_t * data, uint8_t length);
bool busWrite(const busDevice_t * busdev, uint8_t reg, uint8_t data);
bool busReadBuf(const busDevice_t * busdev, uint8_t reg, uint8_t * data, uint8_t length);
bool busRead(const busDevice_t * busdev, uint8_t reg, uint8_t * data);