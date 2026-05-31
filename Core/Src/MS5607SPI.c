/*
   MS5607-02 SPI library for ARM STM32 Microcontrollers - Main source file
   05/01/2020 by Joao Pedro Vilas <joaopedrovbs@gmail.com>
   Changelog:
     2012-05-23 - initial release.
*/
/* ============================================================================================
 MS5607-02 device SPI library code is placed under the MIT license
Copyright (c) 2020 João Pedro Vilas Boas

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 ================================================================================================
 */

#include "MS5607SPI.h"
#include <math.h>

/* --- Module-level state -------------------------------------------------- */

static SPI_HandleTypeDef *_hspi;
static GPIO_TypeDef      *_csPort;
static uint16_t           _csPin;

static struct promData                  _prom;
static struct MS5607UncompensatedValues _raw;
static struct MS5607Readings            _readings;

static MS5607OSRFactors _tempOSR     = OSR_4096;
static MS5607OSRFactors _pressureOSR = OSR_4096;

/* --- OSR to conversion delay (ms) ---------------------------------------- */

static uint32_t getConversionDelay(MS5607OSRFactors osr)
{
    switch (osr) {
        case OSR_256:  return 1;
        case OSR_512:  return 2;
        case OSR_1024: return 3;
        case OSR_2048: return 10;
        case OSR_4096: return 20;
        default:       return 20;
    }
}

/* --- Chip-select helpers -------------------------------------------------- */

void enableCSB(void)
{
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
}

void disableCSB(void)
{
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);
}

/* --- Internal SPI helpers ------------------------------------------------- */

static void sendCommand(uint8_t cmd)
{
    uint8_t rxDummy;
    enableCSB();
    HAL_SPI_TransmitReceive(_hspi, &cmd, &rxDummy, 1, HAL_MAX_DELAY);
    disableCSB();
}

static uint8_t MS5607_CRC4(struct promData *prom)
{
    uint16_t words[8];
    uint16_t *promWords = (uint16_t *)prom;
    for (int i = 0; i < 8; i++) {
        words[i] = promWords[i];
    }

    unsigned int n_rem = 0;
    words[7] = words[7] & 0xFF00; // zero out the CRC byte for calculation

    for (int cnt = 0; cnt < 16; cnt++) {
        if (cnt % 2 == 1) {
            n_rem ^= (unsigned short)(words[cnt >> 1] & 0x00FF);
        } else {
            n_rem ^= (unsigned short)(words[cnt >> 1] >> 8);
        }

        for (int n_bit = 8; n_bit > 0; n_bit--) {
            if (n_rem & 0x8000) {
                n_rem = (n_rem << 1) ^ 0x3000;
            } else {
                n_rem = (n_rem << 1);
            }
        }
    }

    n_rem = 0x000F & (n_rem >> 12);
    return (uint8_t)n_rem;
}

/* --- Public API ----------------------------------------------------------- */

MS5607StateTypeDef MS5607_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin)
{
    _hspi   = hspi;
    _csPort = csPort;
    _csPin  = csPin;

    disableCSB();
    HAL_Delay(10);

    /* Reset the sensor */
    sendCommand(RESET_COMMAND);
    HAL_Delay(3);

    /* Read calibration PROM */
    MS5607PromRead(&_prom);

    /* Sanity-check: at least one calibration word must be non-zero */
    if (_prom.sens == 0 && _prom.off == 0 && _prom.tcs == 0) {
        return MS5607_STATE_FAILED;
    }

    /* CRC check */
    uint8_t read_crc = _prom.crc & 0x000F;
    uint8_t calc_crc = MS5607_CRC4(&_prom);
    if (read_crc != calc_crc) {
        return MS5607_STATE_FAILED;
    }

    return MS5607_STATE_READY;
}

void MS5607PromRead(struct promData *prom)
{
    uint8_t txBuf[3];
    uint8_t rxBuf[3];

    uint16_t *words = (uint16_t *)prom;
    for (uint8_t i = 0; i < 8; i++) {
        txBuf[0] = PROM_READ(i);
        txBuf[1] = 0xFF;
        txBuf[2] = 0xFF;
        enableCSB();
        HAL_SPI_TransmitReceive(_hspi, txBuf, rxBuf, 3, HAL_MAX_DELAY);
        disableCSB();
        words[i] = ((uint16_t)rxBuf[1] << 8) | rxBuf[2];
    }
}

void MS5607UncompensatedRead(struct MS5607UncompensatedValues *values)
{
    uint8_t cmd;
    uint8_t rxDummy;
    uint8_t txBuf[4] = {READ_ADC_COMMAND, 0xFF, 0xFF, 0xFF};
    uint8_t adcRx[4];

    /* Convert D1 – pressure. CS stays low through the conversion delay. */
    cmd = CONVERT_D1_COMMAND | (uint8_t)_pressureOSR;
    enableCSB();
    HAL_SPI_TransmitReceive(_hspi, &cmd, &rxDummy, 1, HAL_MAX_DELAY);
    HAL_Delay(getConversionDelay(_pressureOSR));
    disableCSB();

    enableCSB();
    HAL_SPI_TransmitReceive(_hspi, txBuf, adcRx, 4, HAL_MAX_DELAY);
    disableCSB();
    values->pressure = ((uint32_t)adcRx[1] << 16) | ((uint32_t)adcRx[2] << 8) | adcRx[3];

    /* Convert D2 – temperature. CS stays low through the conversion delay. */
    cmd = CONVERT_D2_COMMAND | (uint8_t)_tempOSR;
    enableCSB();
    HAL_SPI_TransmitReceive(_hspi, &cmd, &rxDummy, 1, HAL_MAX_DELAY);
    HAL_Delay(getConversionDelay(_tempOSR));
    disableCSB();

    enableCSB();
    HAL_SPI_TransmitReceive(_hspi, txBuf, adcRx, 4, HAL_MAX_DELAY);
    disableCSB();
    values->temperature = ((uint32_t)adcRx[1] << 16) | ((uint32_t)adcRx[2] << 8) | adcRx[3];
}

/*
 * Compensation algorithm straight from the MS5607-02BA03 datasheet.
 * Results: readings->temperature in units of 0.01 °C (e.g. 2000 = 20.00 °C)
 *          readings->pressure    in Pa
 */
void MS5607Convert(struct MS5607UncompensatedValues *raw, struct MS5607Readings *readings)
{
    uint32_t D1 = raw->pressure;
    uint32_t D2 = raw->temperature;

    /* Pull calibration values from PROM (named per datasheet) */
    int64_t C1 = _prom.sens;
    int64_t C2 = _prom.off;
    int64_t C3 = _prom.tcs;
    int64_t C4 = _prom.tco;
    int64_t C5 = _prom.tref;
    int64_t C6 = _prom.tempsens;

    /* First-order compensation */
    int32_t dT   = (int32_t)D2 - (int32_t)(C5 * 256LL);
    int32_t TEMP = 2000 + (int32_t)(((int64_t)dT * C6) / 8388608LL); /* 2^23 */

    int64_t OFF  = C2 * 131072LL + (C4 * (int64_t)dT) / 64LL;        /* C2*2^17 + C4*dT/2^6  */
    int64_t SENS = C1 * 65536LL  + (C3 * (int64_t)dT) / 128LL;       /* C1*2^16 + C3*dT/2^7  */

    /* Second-order temperature compensation */
    int64_t T2 = 0, OFF2 = 0, SENS2 = 0;
    if (TEMP < 2000) {
        T2    = ((int64_t)dT * (int64_t)dT) / 2147483648LL;           /* dT^2 / 2^31          */
        OFF2  = 61LL * (int64_t)(TEMP - 2000) * (int64_t)(TEMP - 2000) / 16LL;
        SENS2 = 2LL  * (int64_t)(TEMP - 2000) * (int64_t)(TEMP - 2000);

        if (TEMP < -1500) {
            OFF2  += 15LL * (int64_t)(TEMP + 1500) * (int64_t)(TEMP + 1500);
            SENS2 +=  8LL * (int64_t)(TEMP + 1500) * (int64_t)(TEMP + 1500);
        }
    }

    TEMP -= (int32_t)T2;
    OFF  -= OFF2;
    SENS -= SENS2;

    int64_t P = ((int64_t)D1 * SENS / 2097152LL - OFF) / 32768LL;    /* (D1*SENS/2^21-OFF)/2^15 */

    readings->temperature = TEMP;
    readings->pressure    = (int32_t)P;
}

void MS5607Update(void)
{
    MS5607UncompensatedRead(&_raw);
    MS5607Convert(&_raw, &_readings);
}

double MS5607GetTemperatureC(void)
{
    return (double)_readings.temperature / 100.0;
}

int32_t MS5607GetPressurePa(void)
{
    return _readings.pressure;
}

void MS5607SetTemperatureOSR(MS5607OSRFactors osr)
{
    _tempOSR = osr;
}

double MS5607GetAltitudeM(void) {
	   /* ISA standard atmosphere constants */
	    const double T0       = 288.15;    /* sea-level standard temperature (K)     */
	    const double L        = 0.0065;    /* temperature lapse rate (K/m)           */
	    const double P0       = 101325.0;  /* sea-level standard pressure (Pa)       */
	    const double EXPONENT = 5.2558;    /* g*M / (R*L) — dimensionless            */

	    double pressure_pa = (double)_readings.pressure;  /* use internal sensor state */
	    if (pressure_pa < 0.0) {
	        pressure_pa = 0.0;
	    }
	    double altitude    = (T0 / L) * (1.0 - pow((pressure_pa / P0), (1.0 / EXPONENT)));
	    return altitude;
}
void MS5607SetPressureOSR(MS5607OSRFactors osr)
{
    _pressureOSR = osr;
}
