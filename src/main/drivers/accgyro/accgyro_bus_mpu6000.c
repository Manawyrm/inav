/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Authors:
 * Dominic Clifton - Cleanflight implementation
 * John Ihlein - Initial FF32 code
 * Konstantin Sharlaimov - busDevice refactoring
*/

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/system.h"
#include "drivers/time.h"
#include "drivers/io.h"
#include "drivers/exti.h"
#include "drivers/bus.h"

#include "drivers/gyro_sync.h"

#include "drivers/sensor.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"

#if defined(USE_GYRO_SPI_MPU6000) || defined(USE_ACC_SPI_MPU6000)
#include "drivers/accgyro/accgyro_bus_mpu6000.h"

// Bits
#define BIT_H_RESET                 0x80
#define MPU_CLK_SEL_PLLGYROX        0x01
#define MPU_CLK_SEL_PLLGYROZ        0x03
#define BIT_I2C_IF_DIS              0x10
#define BIT_GYRO                    3
#define BIT_ACC                     2
#define BIT_TEMP                    1

// Product ID Description for MPU6000
// high 4 bits low 4 bits
// Product Name Product Revision
#define MPU6000ES_REV_C4 0x14
#define MPU6000ES_REV_C5 0x15
#define MPU6000ES_REV_D6 0x16
#define MPU6000ES_REV_D7 0x17
#define MPU6000ES_REV_D8 0x18
#define MPU6000_REV_C4 0x54
#define MPU6000_REV_C5 0x55
#define MPU6000_REV_D6 0x56
#define MPU6000_REV_D7 0x57
#define MPU6000_REV_D8 0x58
#define MPU6000_REV_D9 0x59
#define MPU6000_REV_D10 0x5A

static bool mpu6000InitDone = false;

static void mpu6000AccAndGyroInit(gyroDev_t *gyro)
{
    mpuIntExtiInit(gyro);

    spiSetSpeed(MPU6000_SPI_INSTANCE, SPI_CLOCK_INITIALIZATON);

    if (!mpu6000InitDone) {
        // Device Reset
        busWrite(gyro->dev, MPU_RA_PWR_MGMT_1, BIT_H_RESET);
        delay(150);

        busWrite(gyro->dev, MPU_RA_SIGNAL_PATH_RESET, BIT_GYRO | BIT_ACC | BIT_TEMP);
        delay(150);

        // Clock Source PPL with Z axis gyro reference
        busWrite(gyro->dev, MPU_RA_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
        delayMicroseconds(15);

        // Disable Primary I2C Interface
        busWrite(gyro->dev, MPU_RA_USER_CTRL, BIT_I2C_IF_DIS);
        delayMicroseconds(15);

        busWrite(gyro->dev, MPU_RA_PWR_MGMT_2, 0x00);
        delayMicroseconds(15);

        // Accel Sample Rate 1kHz
        // Gyroscope Output Rate =  1kHz when the DLPF is enabled
        busWrite(gyro->dev, MPU_RA_SMPLRT_DIV, gyroMPU6xxxGetDividerDrops(gyro));
        delayMicroseconds(15);

        // Gyro +/- 1000 DPS Full Scale
        busWrite(gyro->dev, MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3);
        delayMicroseconds(15);

        // Accel +/- 8 G Full Scale
        busWrite(gyro->dev, MPU_RA_ACCEL_CONFIG, INV_FSR_8G << 3);
        delayMicroseconds(15);

        busWrite(gyro->dev, MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);  // INT_ANYRD_2CLEAR
        delayMicroseconds(15);

#ifdef USE_MPU_DATA_READY_SIGNAL
        busWrite(gyro->dev, MPU_RA_INT_ENABLE, MPU_RF_DATA_RDY_EN);
        delayMicroseconds(15);
#endif

        spiSetSpeed(MPU6000_SPI_INSTANCE, SPI_CLOCK_FAST);
        delayMicroseconds(1);

        mpu6000InitDone = true;
    }

    // Accel and Gyro DLPF Setting
    busWrite(gyro->dev, MPU_RA_CONFIG, gyro->lpf);
    delayMicroseconds(1);

    spiSetSpeed(MPU6000_SPI_INSTANCE, SPI_CLOCK_FAST);  // 18 MHz SPI clock

    mpuGyroRead(gyro);

    if (((int8_t)gyro->gyroADCRaw[1]) == -1 && ((int8_t)gyro->gyroADCRaw[0]) == -1) {
        failureMode(FAILURE_GYRO_INIT_FAILED);
    }
}

static void mpu6000AccInit(accDev_t *acc)
{
    acc->acc_1G = 512 * 8;
}

bool mpu6000AccDetect(accDev_t *acc)
{
    acc->dev = busDeviceOpen(BUSTYPE_ANY, DEVHW_MPU6000, OWNER_MPU);
    if (acc->dev == NULL) {
        return false;
    }

    if (acc->dev->scratchpad != 0xFFFF6000) {
        return false;
    }

    acc->initFn = mpu6000AccInit;
    acc->readFn = mpuAccRead;

    return true;
}

static bool mpu6000DeviceDetect(busDevice_t * dev)
{
    uint8_t in;
    uint8_t attemptsRemaining = 5;

    busSetSpeed(dev, BUS_SPEED_INITIALIZATION);

    debug[0] = 0x100;
    busWrite(dev, MPU_RA_PWR_MGMT_1, BIT_H_RESET);
    
    do {
        delay(150);

        busRead(dev, MPU_RA_WHO_AM_I, &in);
        debug[1] = in;
        if (in == MPU6000_WHO_AM_I_CONST) {
            break;
        }
        if (!attemptsRemaining) {
            return false;
        }
    } while (attemptsRemaining--);

    busRead(dev, MPU_RA_PRODUCT_ID, &in);

    /* look for a product ID we recognise */
    switch (in) {
        case MPU6000ES_REV_C4:
        case MPU6000ES_REV_C5:
        case MPU6000_REV_C4:
        case MPU6000_REV_C5:
        case MPU6000ES_REV_D6:
        case MPU6000ES_REV_D7:
        case MPU6000ES_REV_D8:
        case MPU6000_REV_D6:
        case MPU6000_REV_D7:
        case MPU6000_REV_D8:
        case MPU6000_REV_D9:
        case MPU6000_REV_D10:
            return true;
    }

    return false;
}

bool mpu6000GyroDetect(gyroDev_t *gyro)
{
    gyro->dev = busDeviceInit(BUSTYPE_ANY, DEVHW_MPU6000, OWNER_MPU);
    if (gyro->dev == NULL) {
        return false;
    }
    
    if (!mpu6000DeviceDetect(gyro->dev)) {
        busDeviceDeInit(gyro->dev);
        return false;
    }

    gyro->dev->scratchpad = 0xFFFF6000;     // Magic number for ACC detection to indicate that we have detected gyro
    gyro->mpuConfiguration.gyroReadXRegister = MPU_RA_GYRO_XOUT_H;
    gyro->initFn = mpu6000AccAndGyroInit;
    gyro->readFn = mpuGyroRead;
    gyro->intStatusFn = mpuCheckDataReady;
    gyro->scale = 1.0f / 16.4f;     // 16.4 dps/lsb scalefactor

    return true;
}

#endif
