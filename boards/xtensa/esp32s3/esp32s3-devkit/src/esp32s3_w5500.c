/****************************************************************************
 * boards/xtensa/esp32/esp32-devkitc/src/esp32_w5500.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <syslog.h>
#include <assert.h>
#include <debug.h>

#include "xtensa.h"
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>
#include <arch/chip/irq.h>
#include <nuttx/spi/spi.h>
#include <nuttx/net/w5500.h>
#include <nuttx/ioexpander/gpio.h>
#include "esp32s3-devkit.h"
#include "esp32s3_spi.h"
#include "esp32s3_gpio.h"
#include "hardware/esp32s3_gpio_sigmap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* W5500 GPIO pins for device 1 */
#define GPIO_W5500_1_INTR   6
#define GPIO_W5500_1_RESET  18
#define GPIO_W5500_1_CS     10

/* W5500 GPIO pins for device 2 */
#define GPIO_W5500_2_INTR   40
#define GPIO_W5500_2_RESET  41
#define GPIO_W5500_2_CS     39

/* W5500 is on SPI2 */

/* SPI Assumptions **********************************************************/

#define W5500_SPI_PORTNO 2   /* On SPI2 */
#define W5500_DEVNO_1    0   /* W5500 device 1 */
#define W5500_DEVNO_2    1   /* W5500 device 2 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp32s3_lower_s
{
  struct w5500_lower_s lower;    /* Low-level MCU interface */
  xcpt_t               handler;  /* W5500 interrupt handler */
  void                *arg;      /* Argument that accompanies IRQ */
  uint8_t              cs_pin;   /* Chip Select pin */
  uint8_t              reset_pin;/* Reset pin */
  uint8_t              intr_pin; /* Interrupt pin */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  up_attach(const struct w5500_lower_s *lower, xcpt_t handler,
                      void *arg);
static void up_enable(const struct w5500_lower_s *lower, bool enable);
static void up_reset(const struct w5500_lower_s *lower, bool reset);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct esp32s3_lower_s g_enclower1 =
{
  .lower =
  {
    .frequency = 40000000,
    .spidevid  = 0,
    .mode      = SPIDEV_MODE0,
    .attach    = up_attach,
    .enable    = up_enable,
    .reset     = up_reset,
  },
  .handler = NULL,
  .arg     = NULL,
  .cs_pin  = GPIO_W5500_1_CS,
  .reset_pin = GPIO_W5500_1_RESET,
  .intr_pin = GPIO_W5500_1_INTR
};

static struct esp32s3_lower_s g_enclower2 =
{
  .lower =
  {
    .frequency = 40000000,
    .spidevid  = 1,
    .mode      = SPIDEV_MODE0,
    .attach    = up_attach,
    .enable    = up_enable,
    .reset     = up_reset,
  },
  .handler = NULL,
  .arg     = NULL,
  .cs_pin  = GPIO_W5500_2_CS,
  .reset_pin = GPIO_W5500_2_RESET,
  .intr_pin = GPIO_W5500_2_INTR
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int up_attach(const struct w5500_lower_s *lower, xcpt_t handler,
                     void *arg)
{
  struct esp32s3_lower_s *priv = (struct esp32s3_lower_s *)lower;

  /* Just save the handler for use when the interrupt is enabled */

  priv->handler = handler;
  priv->arg     = arg;
  return OK;
}

static void up_enable(const struct w5500_lower_s *lower, bool enable)
{
  struct esp32s3_lower_s *priv = (struct esp32s3_lower_s *)lower;

  int irq = ESP32S3_PIN2IRQ(priv->intr_pin);
  int ret;

  /* Make sure the interrupt is disabled */

  esp32s3_gpioirqdisable(irq);

  DEBUGASSERT(priv->handler);
  if (enable)
    {
      ret = irq_attach(irq, priv->handler, priv->arg);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: irq_attach() failed: %d\n", ret);
        }

      /* IRQ on falling edge */

      esp32s3_gpioirqenable(irq, FALLING);
    }
  else
    {
      /* Just keep interrupt disabled is enough */
    }
}

static void up_reset(const struct w5500_lower_s *lower, bool reset)
{
  struct esp32s3_lower_s *priv = (struct esp32s3_lower_s *)lower;
  
  /* Take W5500 out of reset (active low) */

  esp32s3_gpiowrite(priv->reset_pin, !reset);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void esp32s3_spi2_select(struct spi_dev_s *dev, uint32_t devid,
                         bool selected)
{
  switch(devid)
  {
    case W5500_DEVNO_1:
      esp32s3_gpiowrite(g_enclower1.cs_pin, !selected);
      break;
    case W5500_DEVNO_2:
      esp32s3_gpiowrite(g_enclower2.cs_pin, !selected);
      break;
    default:
      break;
  }
  
}

void esp_gw_eth_init(void)
{
  struct spi_dev_s *spi;
  int ret;

  /* Configure the interrupt pin for device 1 */

  esp32s3_configgpio(GPIO_W5500_1_INTR, INPUT_FUNCTION_1 | PULLUP);

  /* Configure the reset pin as output for device 1 */

  esp32s3_gpio_matrix_out(GPIO_W5500_1_RESET, SIG_GPIO_OUT_IDX, 0, 0);
  esp32s3_configgpio(GPIO_W5500_1_RESET, OUTPUT_FUNCTION_1 | INPUT_FUNCTION_1);

  /* Configure the CS pin as output for device 1 */

  esp32s3_configgpio(GPIO_W5500_1_CS, OUTPUT_FUNCTION_1 | INPUT_FUNCTION_1);
  esp32s3_gpio_matrix_out(GPIO_W5500_1_CS, SIG_GPIO_OUT_IDX, 0, 0);

  /* Configure the interrupt pin for device 2 */

  esp32s3_configgpio(GPIO_W5500_2_INTR, INPUT_FUNCTION_1 | PULLUP);

  /* Configure the reset pin as output for device 2 */

  esp32s3_gpio_matrix_out(GPIO_W5500_2_RESET, SIG_GPIO_OUT_IDX, 0, 0);
  esp32s3_configgpio(GPIO_W5500_2_RESET, OUTPUT_FUNCTION_1 | INPUT_FUNCTION_1);

  /* Configure the CS pin as output for device 2 */

  esp32s3_configgpio(GPIO_W5500_2_CS, OUTPUT_FUNCTION_1 | INPUT_FUNCTION_1);
  esp32s3_gpio_matrix_out(GPIO_W5500_2_CS, SIG_GPIO_OUT_IDX, 0, 0);

  /* Assumptions:
   * 1) W5500 pins were configured in up_spi.c early in the boot-up phase.
   * 2) Clocking for the SPI1 peripheral was also provided earlier in
   *    boot-up.
   */

  spi = esp32s3_spibus_initialize(W5500_SPI_PORTNO);
  if (!spi)
  {
    nerr("ERROR: Failed to initialize SPI port %d\n", W5500_SPI_PORTNO);
    return;
  }

  /* Bind the SPI port to the first W5500 driver */

  ret = w5500_initialize(spi, &g_enclower1.lower, W5500_DEVNO_1);
  if (ret < 0)
    {
      nerr("ERROR: Failed to bind SPI port %d W5500 device %d: %d\n",
           W5500_SPI_PORTNO, W5500_DEVNO_1, ret);
      return;
    }

  ninfo("Bound SPI port %d to W5500 device %d\n",
        W5500_SPI_PORTNO, W5500_DEVNO_1);

  /* Bind the SPI port to the second W5500 driver */
  //spi
  ret = w5500_initialize(spi, &g_enclower2.lower, W5500_DEVNO_2);
  if (ret < 0)
    {
      nerr("ERROR: Failed to bind SPI port %d W5500 device %d: %d\n",
           W5500_SPI_PORTNO, W5500_DEVNO_2, ret);
      return;
    }

  ninfo("Bound SPI port %d to W5500 device %d\n",
        W5500_SPI_PORTNO, W5500_DEVNO_2);
}
