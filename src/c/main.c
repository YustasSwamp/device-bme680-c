/*
 * Copyright (c) 2018
 * IoTech Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */
#include <stdio.h>
#include <stdbool.h>
#include <assert.h>
#include <signal.h>
#include <math.h>
#include <pthread.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>
#include <edgex/devsdk.h>
#include "bme680.h"

#define BME680_ERR_CHECK(x) if (x.code) { fprintf (stderr, "Error: %d: %s\n", x.code, x.reason); return x.code; }
#define BME680_SVC "Device-BME680"

typedef struct
{
  iot_logging_client *lc;
  edgex_device_service *svc;
  pthread_mutex_t mutex;
  struct bme680_dev *dev;
  uint16_t meas_period;
} bme680_driver_t;

static bool stop = false;

static void bme680_inthandler (int i)
{
  stop = true;
}

int fd;

static int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  write(fd, &reg_addr,1);
  read(fd, data, len);
  return 0;
}

static void user_delay_ms(uint32_t period)
{
  usleep(period*1000);
}

static int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  int8_t *buf;
  buf = malloc(len +1);
  buf[0] = reg_addr;
  memcpy(buf +1, data, len);
  write(fd, buf, len +1);
  free(buf);
  return 0;
}

static void usage (void)
{
  printf ("Options: \n");
  printf ("   -h, --help           : Show this text\n");
  printf ("   -r, --registry       : Use the registry service\n");
  printf ("   -p, --profile <name> : Set the profile name\n");
  printf ("   -c, --confdir <dir>  : Set the configuration directory\n");
}

static bool bme680_initialize (void *impl, struct iot_logging_client *lc, const edgex_nvpairs *config)
{
  int8_t rslt = BME680_OK;
  uint8_t set_required_settings;

  bme680_driver_t *driver = (bme680_driver_t *) impl;
  driver->lc = lc;
  pthread_mutex_init (&driver->mutex, NULL);

  struct bme680_dev *dev = malloc (sizeof (struct bme680_dev));
  memset (dev, 0, sizeof (struct bme680_dev));
  driver->dev = dev;
  iot_log_debug (lc, "driver initialization");

  if ((fd = open("/dev/i2c-0", O_RDWR)) < 0) {
    iot_log_error (lc, "Failed to open the i2c bus /dev/i2c-0");
    return false;
  }
  if (ioctl(fd, I2C_SLAVE, 0x77) < 0) {
    iot_log_error (lc, "Failed to acquire bus access and/or talk to slave");
    return false;
  }

  dev->dev_id = BME680_I2C_ADDR_SECONDARY;
  dev->intf = BME680_I2C_INTF;
  dev->read = user_i2c_read;
  dev->write = user_i2c_write;
  dev->delay_ms = user_delay_ms;
  dev->amb_temp = 25;

  rslt = bme680_init(dev);
  if (rslt == BME680_OK) {
	  /* Sensor chip ID will be printed if initialization was successful */
	  iot_log_debug (lc, "Device found with chip id 0x%x\n", dev->chip_id);
  } else {
	  iot_log_error (lc, "bme680_init() failed: %d\n", rslt);
	  return false;
  }

  /* Set the temperature, pressure and humidity settings */
  dev->tph_sett.os_hum = BME680_OS_2X;
  dev->tph_sett.os_pres = BME680_OS_4X;
  dev->tph_sett.os_temp = BME680_OS_8X;
  dev->tph_sett.filter = BME680_FILTER_SIZE_3;

  /* Set the remaining gas sensor settings and link the heating profile */
  dev->gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
  /* Create a ramp heat waveform in 3 steps */
  dev->gas_sett.heatr_temp = 320; /* degree Celsius */
  dev->gas_sett.heatr_dur = 150; /* milliseconds */

  /* Select the power mode */
  /* Must be set before writing the sensor configuration */
  dev->power_mode = BME680_FORCED_MODE;

  /* Set the required sensor settings needed */
  set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL
	  | BME680_GAS_SENSOR_SEL;

  /* Set the desired sensor configuration */
  rslt = bme680_set_sensor_settings(set_required_settings, dev);

  /* Set the power mode */
  rslt = bme680_set_sensor_mode(dev);

  /* Get the total measurement duration so as to sleep or wait till the
   * measurement is complete */
  bme680_get_profile_dur(&driver->meas_period, dev);
  iot_log_debug(driver->lc, "meas period:: %d",driver->meas_period);

  return true;
}

static bool bme680_gethandler
(
  void *impl,
  const edgex_addressable *devaddr,
  uint32_t nresults,
  const edgex_device_commandrequest *requests,
  edgex_device_commandresult *readings
)
{
  struct bme680_field_data data;
  bme680_driver_t *driver = (bme680_driver_t *) impl;
  /* Access the address of the device to be accessed and log it */
  iot_log_debug(driver->lc, "GET on address: %s",devaddr->address);

  assert (nresults == 1);
  pthread_mutex_lock (&driver->mutex);

  user_delay_ms(driver->meas_period); /* Delay till the measurement is ready */

  bme680_get_sensor_data(&data, driver->dev);

  /* Trigger the next measurement if you would like to read data out continuously */
  if (driver->dev->power_mode == BME680_FORCED_MODE) {
	  bme680_set_sensor_mode(driver->dev);
  }
  pthread_mutex_unlock (&driver->mutex);

  readings[0].type = Float32;
  if (strcmp (requests[0].devobj->name, "Temperature") ==0 )
            readings[0].value.f32_result = (float) data.temperature / 100.0f;
  else if (strcmp (requests[0].devobj->name, "Humidity") ==0 )
            readings[0].value.f32_result = (float) data.humidity / 1000.0f;
  else if (strcmp (requests[0].devobj->name, "Pressure") ==0 )
            readings[0].value.f32_result = (float) data.pressure / 100.0f;

  return true;
}

static bool bme680_puthandler
(
  void *impl,
  const edgex_addressable *devaddr,
  uint32_t nrequests,
  const edgex_device_commandrequest *requests,
  const edgex_device_commandresult *readings
)
{
  return true;
 }

static bool bme680_disconnect (void *impl, edgex_addressable *device)
{
  return true;
}

static void bme680_stop (void *impl, bool force)
{
  bme680_driver_t *driver = (bme680_driver_t *) impl;

  iot_log_debug (driver->lc, "bme680 stop call");

  close (fd);
  free (driver->dev);
  pthread_mutex_destroy (&driver->mutex);
  free (driver);
}

int main (int argc, char *argv[])
{
  const char *profile = "";
  char *confdir = "";
  edgex_error err;
  bool useRegistry = false;

  bme680_driver_t *driver = malloc (sizeof (bme680_driver_t));
  memset (driver, 0, sizeof (bme680_driver_t));
  driver->lc = iot_log_default;

  int n = 1;
  while (n < argc)
  {
    if (strcmp (argv[n], "-h") == 0 || strcmp (argv[n], "--help") == 0)
    {
      usage ();
      return 0;
    }
    if (strcmp (argv[n], "-r") == 0 || strcmp (argv[n], "--registry") == 0)
    {
      useRegistry = true;
      n++;
      continue;
    }
    if (strcmp (argv[n], "-p") == 0 || strcmp (argv[n], "--profile") == 0)
    {
      profile = argv[n + 1];
      n += 2;
      continue;
    }
    if (strcmp (argv[n], "-c") == 0 || strcmp (argv[n], "--confdir") == 0)
    {
      confdir = argv[n + 1];
      n = n + 2;
      continue;
    }

    printf ("Unknown option %s\n", argv[n]);
    usage ();
    return 0;
  }

  err.code = 0;

  edgex_device_callbacks myImpls =
  {
    bme680_initialize,
    NULL,
    bme680_gethandler,
    bme680_puthandler,
    bme680_disconnect,
    bme680_stop
  };

  edgex_device_service *bme680_service = edgex_device_service_new (BME680_SVC, "1.0", driver, myImpls, &err);
  BME680_ERR_CHECK (err);

  driver->svc = bme680_service;
  edgex_device_service_start (bme680_service, useRegistry, NULL, 0, profile, confdir, &err);
  BME680_ERR_CHECK (err);

  printf ("\nRunning - press ctrl-c to exit\n");
  signal (SIGTERM, bme680_inthandler);
  signal (SIGINT, bme680_inthandler);

  while (!stop)
  {
    sleep (1);
  }

  edgex_error e;
  e.code = 0;
  edgex_device_service_stop (bme680_service, true, &e);
  BME680_ERR_CHECK (e);
  return 0;
}
