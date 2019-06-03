// ac_bmp3.c

#include "ac_bmp3.h"

#include <stdlib.h>
#include <unistd.h>

#define I2C_ADDR 0x77
#define CHIPID 0x50
#define RESET 0xB6
#define MEASURE 0x13
#define STATUS 0x60

#define REG_CHIPID 0x00
#define REG_CAL_DATA 0x31
#define REG_CMD 0x7E
#define REG_CONTROL 0x1B
#define REG_STATUS 0x03
#define REG_PRESS 0x04

#define AC_CONCAT_BYTES(msb, lsb) (((uint16_t)msb << 8) | (uint16_t)lsb)

struct calibration {
  uint16_t T1, T2, P5, P6;
  int16_t P1, P2, P9;
  int8_t T3, P3, P4, P7, P8, P10, P11;
  double T1f, T2f, T3f, P1f, P2f, P3f, P4f, P5f, P6f, P7f, P8f, P9f, P10f, P11f;
};
static struct calibration _c;
static ac_bmp3_rw_func _read_func;
static ac_bmp3_rw_func _write_func;

/* Pow helper */
static double ac_pow(double base, uint8_t power)
{
  double pow_output = 1;
  while (power != 0) {
    pow_output = base * pow_output;
    power--;
  }
  return pow_output;
}

/* BMP3 Chip Identification */
static int read_chip()
{
  uint8_t v = 0;
  _read_func(I2C_ADDR, REG_CHIPID, &v, 1);
  return v == CHIPID ? 0 : -1;
}

/* BMP3 Calibration Data */
static int read_coefficients()
{
  const int size = 21;
  uint8_t *b = calloc(size, sizeof(uint8_t));
  int r = _read_func(I2C_ADDR, REG_CAL_DATA, b, size);

  _c.T1  = (uint16_t)AC_CONCAT_BYTES(b[1], b[0]);
  _c.T2  = (uint16_t)AC_CONCAT_BYTES(b[3], b[2]);
  _c.T3  = (int8_t)b[4];
  _c.P1  = (int16_t)AC_CONCAT_BYTES(b[6], b[5]);
  _c.P2  = (int16_t)AC_CONCAT_BYTES(b[8], b[7]);
  _c.P3  = (int8_t)b[9];
  _c.P4  = (int8_t)b[10];
  _c.P5  = (uint16_t)AC_CONCAT_BYTES(b[12], b[11]);
  _c.P6  = (uint16_t)AC_CONCAT_BYTES(b[14], b[13]);
  _c.P7  = (int8_t)b[15];
  _c.P8  = (int8_t)b[16];
  _c.P9  = (int16_t)AC_CONCAT_BYTES(b[18], b[17]);
  _c.P10 = (int8_t)b[19];
  _c.P11 = (int8_t)b[20];

  _c.T1f = (double)_c.T1 / 0.00390625f;
  _c.T2f = (double)_c.T2 / 1073741824.0f;
  _c.T3f = (double)_c.T3 / 281474976710656.0f;
  _c.P1f = ((double)_c.P1 - 16384) / 1048576.0f;
  _c.P2f = ((double)_c.P2 - 16384) / 536870912.0f;
  _c.P3f = (double)_c.P3 / 4294967296.0f;
  _c.P4f = (double)_c.P4 / 137438953472.0f;
  _c.P5f = (double)_c.P5 / 0.125f;
  _c.P6f = (double)_c.P6 / 64.0f;
  _c.P7f = (double)_c.P7 / 256.0f;
  _c.P8f = (double)_c.P8 / 32768.0f;
  _c.P9f = (double)_c.P9 / 281474976710656.0f;
  _c.P10f = (double)_c.P10 / 281474976710656.0f;
  _c.P11f = (double)_c.P11 / 36893488147419103232.0f;

  free(b);
  return r;
}

/* BMP3 Soft Reset */
static int reset()
{
  uint8_t v = RESET;
  return _write_func(I2C_ADDR, REG_CMD, &v, 1);
}

/* Calibrate temperature reading */
static double comp_temperature(uint32_t T, struct calibration C)
{
  const double TP1 = (double)(T - C.T1f);
	const double TP2 = (double)(TP1 * C.T2f);
	return TP2 + (TP1 * TP1) * C.T3f;
}

/* Calibrate pressure reading */
static double comp_pressure(uint32_t P, double T, struct calibration C)
{
	double partial_data1;
	double partial_data2;
	double partial_data3;
	double partial_data4;
	double partial_out1;
	double partial_out2;

  partial_data1 = C.P6f * T;
	partial_data2 = C.P7f * ac_pow(T, 2);
	partial_data3 = C.P8f * ac_pow(T, 3);
	partial_out1 = C.P5f + partial_data1 + partial_data2 + partial_data3;

	partial_data1 = C.P2f * T;
	partial_data2 = C.P3f * ac_pow(T, 2);
	partial_data3 = C.P4f * ac_pow(T, 3);
	partial_out2 = P * (C.P1f + partial_data1 + partial_data2 + partial_data3);

	partial_data1 = ac_pow((double)P, 2);
	partial_data2 = C.P9f + C.P10f * T;
	partial_data3 = partial_data1 * partial_data2;
	partial_data4 = partial_data3 + ac_pow((double)P, 3) * C.P11f;

	return partial_out1 + partial_out2 + partial_data4;
}

/* Perform once time temperature and pressure measurement */
int ac_bmp3_measure(double *temp, double *press)
{
  uint8_t v = MEASURE;
  if (_write_func(I2C_ADDR, REG_CONTROL, &v, 1) < 0) {
    return -1;
  }

  int loop_count = 0;
  while (1) {
    loop_count += 1;
    usleep(200000);
    uint8_t v = 0;
    if (_read_func(I2C_ADDR, REG_STATUS, &v, 1) < 0) {
      return -1;
    }
    if (v == 0x70) {
      break;
    }
    if (loop_count > 10) {
      return -1;
    }
  }

  const size_t p_size = 6;
  uint8_t *p = calloc(p_size, sizeof(uint8_t));
  if (_read_func(I2C_ADDR, REG_PRESS, p, p_size) < 0) {
    free(p);
    return -1;
  }

  uint32_t P = (p[2] << 16) | (p[1] << 8) | p[0];
  uint32_t T = (p[5] << 16) | (p[4] << 8) | p[3];

  free(p);

  double temperature = comp_temperature(T, _c);
  double pressure = comp_pressure(P, temperature, _c);

	*temp = temperature;
	*press = pressure;

	return 0;
}

/* Initalize the BMP3 sensor */
extern int ac_bmp3_initalize(ac_bmp3_rw_func r_func,
                             ac_bmp3_rw_func w_func)
{
  _read_func = r_func;
  _write_func = w_func;

  if (read_chip() < 0) {
    return -1;
  }

  if (reset() < 0) {
    return -1;
  }

  if (read_coefficients() < 0) {
    return -1;
  }

  return 0;
}

