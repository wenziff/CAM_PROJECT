/**
 * @copyright (C) 2017 Melexis N.V.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "MLX90640_I2C_Driver.h"
#include "i2c.h"
#include <stddef.h>
#include <stdint.h>

#define MLX90640_I2C_TIMEOUT_MS 1000U

void MLX90640_I2CInit(void)
{
    /* MX_I2C3_Init() is called once from main. */
}

void MLX90640_I2CFreqSet(int freq)
{
    (void)freq;
    /* STM32CubeMX configures this bus for 400 kHz. */
}

int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress,
                     uint16_t nMemAddressRead, uint16_t *data)
{
    uint8_t *bytes = (uint8_t *)data;
    uint32_t byte_count = (uint32_t)nMemAddressRead * 2U;
    uint32_t i;

    if ((data == NULL) || (byte_count > UINT16_MAX)) {
        return -1;
    }
    if (HAL_I2C_Mem_Read(&hi2c3, (uint16_t)slaveAddr << 1,
                         startAddress, I2C_MEMADD_SIZE_16BIT,
                         bytes, (uint16_t)byte_count,
                         MLX90640_I2C_TIMEOUT_MS) != HAL_OK) {
        return -1;
    }
    for (i = 0U; i < byte_count; i += 2U) {
        uint8_t msb = bytes[i];
        bytes[i] = bytes[i + 1U];
        bytes[i + 1U] = msb;
    }
    return 0;
}

int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data)
{
    uint8_t bytes[2];
    uint16_t verify = 0U;

    bytes[0] = (uint8_t)(data >> 8);
    bytes[1] = (uint8_t)data;
    if (HAL_I2C_Mem_Write(&hi2c3, (uint16_t)slaveAddr << 1,
                          writeAddress, I2C_MEMADD_SIZE_16BIT,
                          bytes, sizeof(bytes),
                          MLX90640_I2C_TIMEOUT_MS) != HAL_OK) {
        return -1;
    }
    if (MLX90640_I2CRead(slaveAddr, writeAddress, 1U, &verify) != 0) {
        return -1;
    }
    return (verify == data) ? 0 : -2;
}
