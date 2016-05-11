#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <cmath>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <math.h>

#include "aos/common/logging/queue_logging.h"
#include "aos/common/util/phased_loop.h"
#include "aos/linux_code/init.h"
#include "third_party/MPU6050/MPU6050.h"
#include "third_party/ADXL345/ADXL345.h"
#include "third_party/HMC5883L/HMC5883L.h"
#include "y2016_balloon/input/sensor_reader.h"
#include "y2016_balloon/input/sensors.q.h"
#include <gps.h>

#define sleepms(ms) usleep((ms)*1000)

#define I2CBus "/dev/i2c-1"
float RAD_TO_DEG = 57.29578;
#define M_PI 3.14159265358979323846

namespace y2016_balloon {
namespace input {
namespace bmp180 {
// Returns a file id for the port/bus
int i2c_Open(char *I2CBusName) {
  int fd;
  // Open port for reading and writing
  if ((fd = open(I2CBusName, O_RDWR)) < 0) {
    printf("\n");
    printf("%s : Failed to open the i2c bus, error : %d\n", __func__, errno);
    printf("Check to see if you have a bus: %s\n", I2CBusName);
    printf(
        "This is not a slave device problem, I can not find the bus/port with "
        "which to talk to the device\n");
    printf("\n");
    // Only one of the following lines should be used
    // the second line allows you to retry another bus, PS disable all the
    // printf 's
    exit(1);  // Use this line if the function must terminate on failure
              // return fd;  //Use this line if it must return to the caller for
              // processing
  } else {
    return fd;
  }
}

// BMP085 & BMP180 Specific code
int bmp_ReadInt(int fd, uint8_t *devValues, uint8_t startReg,
                uint8_t bytesToRead) {
  int rc;
  struct i2c_rdwr_ioctl_data messagebuffer;

  // Build a register read command
  // Requires a one complete message containing a command
  // and anaother complete message for the reply
  struct i2c_msg read_reg[2] = {
      {BMPx8x_I2CADDR, 0, 1, &startReg},
      {BMPx8x_I2CADDR, I2C_M_RD, bytesToRead, devValues}};

  messagebuffer.nmsgs = 2;        // Two message/action
  messagebuffer.msgs = read_reg;  // load the 'read__reg' message into the
                                  // buffer
  rc = ioctl(
      fd, I2C_RDWR,
      &messagebuffer);  // Send the buffer to the bus and returns a send status
  if (rc < 0) {
    printf("\n");
    printf("%s :Reg Read command failed with error :%d\n", __func__, errno);
    printf(
        "This means that device with address :0x%0x failed to receive this "
        "command\n",
        BMPx8x_I2CADDR);
    printf("This command was preceded by a reset if that worked\n");
    printf(
        "and this failed, then possible causes are Delay timing to short "
        "(overclock stuffing timing up)\n");
    printf(
        "or bus unstable ,wire length,power supply unstable, terminating "
        "resistors.\n");
    printf("\n");
    // Only one of the following lines should be used
    // exit(1);       //Use this line if the function must terminate on failure
    return rc;  // Use this line if it must return to the caller for processing
  }
  // note that the return data is contained in the array pointed to by devValues
  // (passed by-ref)
  return 0;
}

int bmp_WriteCmd(int fd, uint8_t devAction) {
  int rc;
  struct i2c_rdwr_ioctl_data messagebuffer;
  uint8_t datatosend[2];

  datatosend[0] = BMPx8x_CtrlMeas;
  datatosend[1] = devAction;
  // Build a register write command
  // Requires one complete message containing a reg address and command
  struct i2c_msg write_reg[1] = {{BMPx8x_I2CADDR, 0, 2, datatosend}};

  messagebuffer.nmsgs = 1;  // One message/action
  messagebuffer.msgs =
      write_reg;  // load the 'write__reg' message into the buffer
  rc = ioctl(
      fd, I2C_RDWR,
      &messagebuffer);  // Send the buffer to the bus and returns a send status
  if (rc < 0) {
    printf("\n");
    printf("%s :Write reg command failed with error :%d\n", __func__, errno);
    printf(
        "This means that device with address :0x%0x failed to receive this "
        "command\n",
        BMPx8x_I2CADDR);
    printf("This command was preceded by a reset if that worked\n");
    printf(
        "and this failed, then possible causes are Delay timing to short "
        "(overclock stuffing timing up)\n");
    printf(
        "or bus unstable ,wire length,power supply unstable, terminating "
        "resistors.\n");
    printf("\n");
    // Only one of the following lines should be used
    // exit(1);       //Use this line if the function must terminate on failure
    return rc;  // Use this line if it must return to the caller for processing
  }
  return 0;
}

int bmp_Calibration(int fd) {
  uint8_t rValue[21];
  // printf("Entering Calibration\n");
  if (bmp_ReadInt(fd, rValue, 0xAA, 22) == 0) {
    bmp_ac1 = ((rValue[0] << 8) | rValue[1]);
    bmp_ac2 = ((rValue[2] << 8) | rValue[3]);
    bmp_ac3 = ((rValue[4] << 8) | rValue[5]);
    bmp_ac4 = ((rValue[6] << 8) | rValue[7]);
    bmp_ac5 = ((rValue[8] << 8) | rValue[9]);
    bmp_ac6 = ((rValue[10] << 8) | rValue[11]);
    bmp_b1 = ((rValue[12] << 8) | rValue[13]);
    bmp_b2 = ((rValue[14] << 8) | rValue[15]);
    bmp_mb = ((rValue[16] << 8) | rValue[17]);
    bmp_mc = ((rValue[18] << 8) | rValue[19]);
    bmp_md = ((rValue[20] << 8) | rValue[21]);

    return 0;
  }

  return -1;
}

int WaitForConversion(int fd) {
  uint8_t rValues[3];
  int counter = 0;

  // Delay can now be reduced by checking that bit 5 of Ctrl_Meas(0xF4) == 0
  do {
    sleepms(BMPx8x_RetryDelay);
    if (bmp_ReadInt(fd, rValues, BMPx8x_CtrlMeas, 1) != 0) return -1;
    counter++;
  } while (((rValues[0] & 0x20) != 0) && counter < 20);
  return 0;
}

// Calculate calibrated pressure
// Value returned will be in hPa
int bmp_GetPressure(int fd, double *Pres) {
  unsigned int up;
  uint8_t rValues[3];

  // Pressure conversion with oversampling 0x34+ BMPx8x_OverSampling 'bit
  // shifted'
  if (bmp_WriteCmd(fd, (BMPx8x_PresConversion0 + (BMPx8x_OverSampling << 6))) !=
      0)
    return -1;

  // Delay gets longer the higher the oversampling must be at least 26 ms plus a
  // bit for turbo
  // clock error ie 26 * 1000/700 or 38 ms
  // sleepms (BMPx8x_minDelay + (4<<BMPx8x_OverSampling));  //39ms at oversample
  // = 3

  // Code is now 'turbo' overclock independent
  sleepms(BMPx8x_minDelay);
  if (WaitForConversion(fd) != 0) return -1;

  if (bmp_ReadInt(fd, rValues, BMPx8x_Results, 3) != 0) return -1;
  up = (((unsigned int)rValues[0] << 16) | ((unsigned int)rValues[1] << 8) |
        (unsigned int)rValues[2]) >>
       (8 - BMPx8x_OverSampling);

  int x1, x2, x3, b3, b6, p;
  unsigned int b4, b7;

  b6 = bmp_b5 - 4000;
  x1 = (bmp_b2 * (b6 * b6) >> 12) >> 11;
  x2 = (bmp_ac2 * b6) >> 11;
  x3 = x1 + x2;
  b3 = (((((int)bmp_ac1) * 4 + x3) << BMPx8x_OverSampling) + 2) >> 2;

  x1 = (bmp_ac3 * b6) >> 13;
  x2 = (bmp_b1 * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (bmp_ac4 * (unsigned int)(x3 + 32768)) >> 15;

  b7 = ((unsigned int)(up - b3) * (50000 >> BMPx8x_OverSampling));
  if (b7 < 0x80000000)
    p = (b7 << 1) / b4;
  else
    p = (b7 / b4) << 1;

  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  p += (x1 + x2 + 3791) >> 4;
  *Pres = ((double)p / 100);
  return 0;
}

// Calculate calibrated temperature
// Value returned will be in units of 0.1 deg C
int bmp_GetTemperature(int fd, double *Temp) {
  unsigned int ut;
  uint8_t rValues[2];

  if (bmp_WriteCmd(fd, BMPx8x_TempConversion) != 0) return -1;
  // Code is now 'turbo' overclock independent
  sleepms(BMPx8x_minDelay);
  if (WaitForConversion(fd) != 0) return -1;

  if (bmp_ReadInt(fd, rValues, BMPx8x_Results, 2) != 0) return -1;
  ut = ((rValues[0] << 8) | rValues[1]);

  int x1, x2;
  x1 = (((int)ut - (int)bmp_ac6) * (int)bmp_ac5) >> 15;
  x2 = ((int)bmp_mc << 11) / (x1 + bmp_md);
  bmp_b5 = x1 + x2;

  double result = ((bmp_b5 + 8) >> 4);
  *Temp = result / 10;
  return 0;
}

double bmp_altitude(double p) {
  //return 145437.86 * (1 - pow((p / 1013.25), 0.190294496));  // return feet
  return 44330*(1- pow((p/1013.25),0.190294496)); //return meters
}

double bmp_qnh(double p, double StationAlt) {
  //return p / pow((1 - (StationAlt / 145437.86)),
  //               5.255);  // return hPa based on feet
  return p / pow((1-(StationAlt/44330)),5.255) ; //return hPa based on meters
}

double ppl_DensityAlt(double PAlt, double Temp) {
  double ISA = 15 - (1.98 * (PAlt / 1000));
  return PAlt + (120 * (Temp - ISA));  // So,So density altitude
}
}  // namespace bmp180

namespace ms5803 {
/*
 * libms5803 - MS5803 pressure sensor library
 *
 * Copyright (C) 2014-2016 by Artur Wroblewski <wrobell@pld-linux.org>
 *
 * Also contains code from
 *
 *      http://www.john.geek.nz/2013/02/update-bosch-ms5803-source-raspberry-pi/
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include <stdint.h>

#ifdef MS5803_DEBUG
#include <stdio.h>
#define DEBUG_LOG(fmt, ...) fprintf(stderr, fmt, ##__VA_ARGS__)
#else
#define DEBUG_LOG(fmt, ...)
#endif

#define BSWAP16(v) (v << 8) & 0xFF00 | (v >> 8) & 0xFF

/* i2c file descriptor */
static int i2c_fd;

/* MS5803 calibration data */
static uint16_t calib_coeff[8] = {0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00};

/*!
 * Perform ADC read from the sensor.
 *
 * Command is 0x48 or 0x58 - read pressure or temperature with highest
 * accuracy.
 *
 * \param command ADC command.
 */
static uint32_t read_value(uint8_t command) {
  int r;
  uint8_t data[3];

  // r = write(i2c_fd, (unsigned char[]) {command}, 1);
  unsigned char array[] = {command};
  r = write(i2c_fd, array, 1);
  if (r != 1) return -1;
  usleep(8250);

  array[0] = 0x00;
  r = write(i2c_fd, array, 1);
  if (r != 1) return -1;

  r = read(i2c_fd, data, 3);
  if (r != 3) return -1;

  DEBUG_LOG("MS5803 DEBUG: value for command %x: %d\n", command,
            (data[0] << 16) | (data[1] << 8) | data[2]);
  return (data[0] << 16) | (data[1] << 8) | data[2];
}

/*!
 * Calculate pressure and temperature using D1, D2 values read from sensor
 * and the calibration coefficients.
 */
static void calculate(uint32_t d1, uint32_t d2, int32_t *pressure,
                      int32_t *temperature) {
  int32_t d_t, temp, p;
  int64_t off, sens, off_2, sens_2, t_2;
  int64_t c1, c2, c3, c4, c5, c6;

  c1 = calib_coeff[1];
  c2 = calib_coeff[2];
  c3 = calib_coeff[3];
  c4 = calib_coeff[4];
  c5 = calib_coeff[5];
  c6 = calib_coeff[6];

  d_t = d2 - (c5 << 8);
  temp = 2000 + ((d_t * c6) >> 23);
  off = (c2 << 16) + ((c4 * d_t) >> 7);
  sens = (c1 << 15) + ((c3 * d_t) >> 8);
  p = (((d1 * sens) >> 21) - off) >> 15;

  DEBUG_LOG("MS5803 DEBUG: d_t: %d\n", d_t);
  DEBUG_LOG("MS5803 DEBUG: temp: %d\n", temp);
  DEBUG_LOG("MS5803 DEBUG: off: %d\n", off);
  DEBUG_LOG("MS5803 DEBUG: sens: %d\n", sens);
  DEBUG_LOG("MS5803 DEBUG: p: %d\n", p);

  if (temp >= 2000) {
    DEBUG_LOG("MS5803 DEBUG: temp >= 20\n");
    t_2 = (7 * ((int64_t)d_t * d_t)) >> 37;
    off_2 = ((temp - 2000) * (temp - 2000)) >> 4;
    sens_2 = 0;
  } else {
    DEBUG_LOG("MS5803 DEBUG: temp < 20\n");
    t_2 = (3 * (int64_t)d_t * d_t) >> 33;
    off_2 = (3 * (temp - 2000) * (temp - 2000)) >> 2;
    sens_2 = (5 * (temp - 2000) * (temp - 2000)) >> 3;

    if (temp < 1500) {
      DEBUG_LOG("MS5803 DEBUG: temp < 15\n");
      t_2 = (3 * (int64_t)d_t * d_t) >> 33;
      off_2 = off_2 + 7 * (temp + 1500) * (temp + 1500);
      sens_2 = sens_2 + 4 * (temp + 1500) * (temp + 1500);
    }
  }

  temp = temp - t_2;
  off = off - off_2;
  sens = sens - sens_2;

  *pressure = p;
  *temperature = temp;
}

/*
 * Public API implementation.
 */

int ms5803_init(const char *f_dev, unsigned char address) {
  int r, i;

  if ((i2c_fd = open(f_dev, O_RDWR)) < 0) return -1;

  /* set the port options and set the address of the device */
  if (ioctl(i2c_fd, I2C_SLAVE, address) < 0) return -1;

  /* reset the sensor */
  unsigned char array[] = {0x1e};
  r = write(i2c_fd, array, 1);
  if (r != 1) return -1;
  usleep(3000);

  /* read the calibration coefficients and store them in `calib_coeff`
     array */
  for (i = 0; i < 8; i++) {
    array[0] = 0xa0 + i * 2;
    r = write(i2c_fd, array, 1);
    if (r != 1) return -1;

    r = read(i2c_fd, &calib_coeff[i], 2);
    calib_coeff[i] = BSWAP16(calib_coeff[i]);
    DEBUG_LOG("MS5803 DEBUG: calibration coefficient %d: %d\n", i,
              calib_coeff[i]);
    if (r != 2) return -1;
  }

  return 0;
}

int ms5803_read(int32_t *pressure, int32_t *temperature) {
  uint32_t d1;
  uint32_t d2;

  d1 = read_value(0x48);
  if (d1 == -1) return -1;

  d2 = read_value(0x58);
  if (d2 == -1) return -1;

  calculate(d1, d2, pressure, temperature);
  return 0;
}

int ms5803_close() { return close(i2c_fd); }

}  // namespace ms5803
}  // namespace input
}  // namespace y2016_balloon

int main(int argc, char **argv) {
  ::aos::Init();
  int fd, fd_adxl;

  double bmp180_temp, bmp180_pressure;

  short ax, ay, az;
  int16_t cx, cy, cz;
  //float AccYangle(0.0), AccXangle(0.0);

  int32_t ms5803_temp, ms5803_pressure;

  HMC5883L hmc5883l;

  // double PAlt;
  fd = ::y2016_balloon::input::bmp180::i2c_Open(I2CBus);

  if(init(fd_adxl)){
    printf("Could not init ADXL345\n");
  }

  printf("\nCalibration:%i (0= worked)\n",
         ::y2016_balloon::input::bmp180::bmp_Calibration(fd));

  hmc5883l.initialize();

  ::aos::time::PhasedLoop phased_loop(::aos::time::Time::InMS(5),
                                      ::aos::time::Time::InMS(0));
  ::y2016_balloon::input::ms5803::ms5803_init(I2CBus, 0x76);
  while (true) {
    phased_loop.SleepUntilNext();
    ::y2016_balloon::input::bmp180::bmp_GetTemperature(fd, &bmp180_temp);
    ::y2016_balloon::input::bmp180::bmp_GetPressure(fd, &bmp180_pressure);
    readADXL345(fd_adxl, ax, ay, az);
    hmc5883l.getHeading(&cx, &cy, &cz);
    //TODO(comran): Check the following...
    /*
    AccXangle = (float)(atan2(ay, az) + M_PI) * RAD_TO_DEG;
    AccYangle = (float)(atan2(ax, az) + M_PI) * RAD_TO_DEG;
    // Change the rotation value of the accelerometer to -/+ 180
    if (AccXangle > 180) {
      AccXangle -= (float)360.0;
    }
    if (AccYangle > 180) {
      AccYangle -= (float)360.0;
    }*/

    ::y2016_balloon::input::ms5803::ms5803_read(&ms5803_pressure, &ms5803_temp);

    ::y2016_balloon::sensors::sensors.MakeWithBuilder()
        .bmp180_temp(bmp180_temp)
        .bmp180_pressure(bmp180_pressure)
        .bmp180_altitude(::y2016_balloon::input::bmp180::bmp_altitude(bmp180_pressure))
        .ax(ax)
        .ay(ay)
        .az(az)
        .cx(cx)
        .cy(cy)
        .cz(cz)
        .ms5803_temp(ms5803_temp)
        .ms5803_pressure(ms5803_pressure)
        .Send();
    // TODO(comran): bmp_qnh, ppl_DensityAlt.
    // TODO(comran): Use all SI units.
  }

  close(fd);

  ::aos::Cleanup();
  return 0;
}
