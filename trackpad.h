#ifndef ZEPHYR_DRIVERS_SENSOR_BBTRACKPAD_H_
#define ZEPHYR_DRIVERS_SENSOR_BBTRACKPAD_H_

#include <zephyr.h>
#include <drivers/spi.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>

/* The BlackBerry trackpad appears to follow the ADNS3530 register
 * definitions. One difference appears to be that burst mode is not supported.
 */

#define BBTRACKPAD_REG_PRODUCT_ID             0x00
#define BBTRACKPAD_REG_REVISION_ID            0x01
#define BBTRACKPAD_REG_MOTION                 0x02
#define BBTRACKPAD_REG_DELTA_X                0x03
#define BBTRACKPAD_REG_DELTA_Y                0x04

#define BBTRACKPAD_REG_CONFIGURATION          0x11

#define BBTRACKPAD_REG_OBSERVATION            0x2E

#define BBTRACKPAD_REG_INV_REVISION_ID        0x3E
#define BBTRACKPAD_REG_INV_PRODUCT_ID         0x3F

#define BBTRACKPAD_REG_MOTION_BURST           0x42

/* Register bitfields */

#define BBTRACKPAD_REG_MOTION_MOT             (1 << 7)
#define BBTRACKPAD_REG_MOTION_OVF             (1 << 4)

#define BBTRACKPAD_REG_CONFIGURATION_RES      (1 << 7)

#define BBTRACKPAD_REG_OBSERVATION_MODE_RUN   (0 << 6)
#define BBTRACKPAD_REG_OBSERVATION_MODE_REST1 (1 << 6)
#define BBTRACKPAD_REG_OBSERVATION_MODE_REST2 (2 << 6)
#define BBTRACKPAD_REG_OBSERVATION_MODE_REST3 (3 << 6)

/* Internal constants */

#define BB_TRACKPAD_PRODUCT_ID     0x0d    /* BB8520 */
#define BB_TRACKPAD_REVISION       0x02

#define BBTRACKPAD_INT_PORT    DT_GPIO_LABEL(TRACKPAD_DEV, irq_gpios)
#define BBTRACKPAD_INT_PIN     DT_GPIO_PIN(TRACKPAD_DEV, irq_gpios)
#define BBTRACKPAD_INT_FLAGS   DT_GPIO_FLAGS(TRACKPAD_DEV, irq_gpios)

/* Device configuration */

struct bbtrackpad_dev_config
{
  int motion_pin;
  int reset_pin;
  int power_pin;
};

struct bbtrackpad_motion_data
{
  int16_t dx;
  int16_t dy;

};

struct bbtrackpad_data
{
  const struct device *bus;
  struct spi_config spi_cfg;
  struct spi_cs_control cs_ctrl;
  struct bbtrackpad_motion_data sample;
  //struct bbtrackpad_fifo_config fifo_config;

  const struct device *gpio;
  struct gpio_callback gpio_cb;

  sensor_trigger_handler_t trigger_handler;
  struct sensor_trigger trigger;

  const struct device *dev;

  struct k_sem excl_sem;
  struct k_work work;

#if 0
  #if defined(CONFIG_ADXL372_TRIGGER_OWN_THREAD)
    K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_ADXL372_THREAD_STACK_SIZE);
    struct k_sem gpio_sem;
    struct k_thread thread;
  #elif defined(CONFIG_ADXL372_TRIGGER_GLOBAL_THREAD)
    struct k_work work;
  #endif
#endif
};

#endif
