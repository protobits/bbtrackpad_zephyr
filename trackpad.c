#include <zephyr.h>
#include <kernel.h>
#include <init.h>
#include <logging/log.h>
#include "trackpad.h"

#define DT_DRV_COMPAT blackberry_trackpad_spi
LOG_MODULE_REGISTER(bbtrackpad, 3);

static void bbtrackpad_on_motion(const struct device *port,
                                 struct gpio_callback *cb,
                                 gpio_port_pins_t pins)
{
  struct bbtrackpad_data* data = CONTAINER_OF(cb, struct bbtrackpad_data, gpio_cb);
  k_work_submit(&data->work);
}

static void bbtrackpad_worker(struct k_work* work)
{
  struct bbtrackpad_data* data = CONTAINER_OF(work, struct bbtrackpad_data, work);

  if (data->trigger_handler)
  {
    data->trigger_handler(data->dev, &data->trigger);
  }
}

int bbtrackpad_trigger_set(const struct device* dev,
                           const struct sensor_trigger *trig,
                           sensor_trigger_handler_t handler)
{
  struct bbtrackpad_data* data = dev->data;

  if (trig->type != SENSOR_TRIG_DATA_READY)
    {
      return -EINVAL;
    }

  data->trigger_handler = handler;
  data->trigger = *trig;

  return 0;
}

void bbtrackpad_write_register(const struct device* dev, uint8_t addr, uint8_t data)
{
  const struct bbtrackpad_data* bb_data = dev->data;

  uint8_t write_addr = (addr | (1 << 7));

  struct spi_buf tx_buf[2] =
  {
    { .buf = &write_addr, .len = 1 },
    { .buf = &data, .len = 1 }
  };

  struct spi_buf_set tx_bufset = { .buffers = tx_buf, .count = 2 };

  spi_write(bb_data->bus, &bb_data->spi_cfg, &tx_bufset);
}

void bbtrackpad_read_register(const struct device* dev, uint8_t addr, uint8_t* data, size_t size)
{
  struct bbtrackpad_data* bb_data = dev->data;

  uint8_t buf[2] = { addr, 0 };
  struct spi_buf tx_buf;
  tx_buf.buf = buf;
  tx_buf.len = 2;

  struct spi_buf rx_buf[2];
  rx_buf[0].buf = NULL;
  rx_buf[0].len = 1;
  rx_buf[1].buf = data;
  rx_buf[1].len = size;

  struct spi_buf_set tx_bufset = { .buffers = &tx_buf, .count = 1 };
  struct spi_buf_set rx_bufset = { .buffers = rx_buf, .count = 2 };

  spi_transceive(bb_data->bus, &bb_data->spi_cfg, &tx_bufset, &rx_bufset);
}

int bbtrackpad_fetch(const struct device* dev, enum sensor_channel chan)
{
  struct bbtrackpad_data* data = dev->data;

  int i = 0;
  uint8_t values[3];
  int16_t dx, dy;

  values[1] = values[2] = 0;

  /* Read motion registers in burst mode */

  bbtrackpad_read_register(dev, BBTRACKPAD_REG_MOTION_BURST, values, 3);

  dx = (int8_t)values[2];
  dy = (int8_t)values[1];

  /* Since internal data may have overflowed, we read in a loop
   * until all data is read. Just in case we don't end up reading
   * indefinitely if motion continues to occur, we read the maximum
   * possible according to datasheet (32 cycles at full resolution)
   */

  while ((values[0] & BBTRACKPAD_REG_MOTION_MOT) && (i < 32))
    {
      bbtrackpad_read_register(dev, BBTRACKPAD_REG_MOTION_BURST, values, 3);

      dx += values[2];
      dy += values[1];

      i++;
    }

  /* Add up current results to sample so that data is not lost
   * if get() is not called immediately */

  k_sem_take(&data->excl_sem, K_FOREVER);
  data->sample.dx += dx;
  data->sample.dy += dy;
  k_sem_give(&data->excl_sem);

  return (i > 0);
}

int bbtrackpad_channel_get(const struct device* dev,
                           enum sensor_channel chan,
                           struct sensor_value *val)
{
  struct bbtrackpad_data* data = dev->data;
  struct bbtrackpad_motion_data* motion = (struct bbtrackpad_motion_data*)val;

  k_sem_take(&data->excl_sem, K_FOREVER);
  *motion = data->sample;
  k_sem_give(&data->excl_sem);

  return 0;
}

void bbtrackpad_reset(const struct device* dev)
{
  struct bbtrackpad_data* data = dev->data;
  const struct bbtrackpad_dev_config* cfg = dev->config;

  gpio_pin_set(data->gpio, cfg->reset_pin, true);

  k_msleep(10);

  gpio_pin_set(data->gpio, cfg->reset_pin, false);
}

void bbtrackpad_power(const struct device* dev, bool on)
{
  const struct bbtrackpad_data* data = dev->data;
  const struct bbtrackpad_dev_config* cfg = dev->config;

  gpio_pin_set(data->gpio, cfg->power_pin, on);

  if (on)
    {
      k_msleep(100);

      bbtrackpad_reset(dev);
    }
}

int bbtrackpad_verify(const struct device* dev)
{
  uint8_t regval, regval2;

  /* Check product ID */

  bbtrackpad_read_register(dev, BBTRACKPAD_REG_PRODUCT_ID, &regval, 1);

  if (regval != BB_TRACKPAD_PRODUCT_ID)
    {
      printk("Unexpected product ID: %i\n", regval);
      return false;
    }

  /* Verify correct communication */

  bbtrackpad_read_register(dev, BBTRACKPAD_REG_INV_PRODUCT_ID, &regval2, 1);

  if (!(regval ^ regval2))
    {
      printk("Read test failed\n");
      return false;
    }

  /* Check revision */

  bbtrackpad_read_register(dev, BBTRACKPAD_REG_REVISION_ID, &regval, 1);

  if (regval != BB_TRACKPAD_REVISION)
    {
      printk("Unexpected revision: %i\n", regval);
      return false;
    }

  bbtrackpad_read_register(dev, BBTRACKPAD_REG_INV_REVISION_ID, &regval2, 1);

  if (!(regval ^ regval2))
    {
      printk("Read test failed\n");
      return false;
    }

  printk("Detected valid trackpad\n");

  return true;
}

void bbtrackpad_configure(const struct device* dev)
{
  uint8_t regval;

  bbtrackpad_read_register(dev, BBTRACKPAD_REG_CONFIGURATION, &regval, 1);
  bbtrackpad_write_register(dev, BBTRACKPAD_REG_CONFIGURATION, regval | BBTRACKPAD_REG_CONFIGURATION_RES);
  bbtrackpad_read_register(dev, BBTRACKPAD_REG_CONFIGURATION, &regval, 1);

  if (!(regval & BBTRACKPAD_REG_CONFIGURATION_RES))
    {
      printk("failed to set high resolution mode\n");
    }
}

static int bbtrackpad_init(const struct device *dev)
{
	struct bbtrackpad_data *data = dev->data;
	const struct bbtrackpad_dev_config *cfg = dev->config;

  k_sem_init(&data->excl_sem, 1, 1);

  /* configure SPI */

  data->spi_cfg.cs = &data->cs_ctrl;
  data->spi_cfg.frequency = DT_INST_PROP(0, spi_max_frequency);
  data->spi_cfg.operation = SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_LINES_SINGLE;
  data->spi_cfg.slave = 0;

  /* get SPI instance */

  data->bus = device_get_binding(DT_INST_BUS_LABEL(0));
  if (!data->bus)
    {
      LOG_ERR("spi device not found");
      return -ENODEV;
    }

  /* CS line */

  data->cs_ctrl.gpio_dev = device_get_binding(DT_INST_SPI_DEV_CS_GPIOS_LABEL(0));
  data->cs_ctrl.gpio_pin = DT_INST_SPI_DEV_CS_GPIOS_PIN(0);
  data->cs_ctrl.gpio_dt_flags = DT_INST_SPI_DEV_CS_GPIOS_FLAGS(0);
  data->cs_ctrl.delay = 0;

  /* power & reset pins */

  gpio_pin_configure(data->cs_ctrl.gpio_dev, cfg->power_pin, GPIO_OUTPUT_LOW);
  gpio_pin_configure(data->cs_ctrl.gpio_dev, cfg->reset_pin, GPIO_OUTPUT_LOW);

  /* motion pin handling */

  gpio_pin_configure(data->cs_ctrl.gpio_dev, cfg->motion_pin, GPIO_INPUT | GPIO_INT_ENABLE | GPIO_INT_EDGE | GPIO_INT_EDGE_FALLING);

  k_work_init(&data->work, bbtrackpad_worker);
  gpio_init_callback(&data->gpio_cb, bbtrackpad_on_motion, BIT(cfg->motion_pin));
  gpio_add_callback(data->cs_ctrl.gpio_dev, &data->gpio_cb);

  bbtrackpad_power(dev, false);
  k_msleep(10);
  bbtrackpad_power(dev, true);

	if (bbtrackpad_verify(dev) < 0) {
		return -ENODEV;
	}

  bbtrackpad_configure(dev);

  /* discard any previous data */

  bbtrackpad_fetch(dev, SENSOR_CHAN_ALL);
  data->sample.dx = data->sample.dy = 0;

	return 0;
}

/* Sensor API */

static const struct sensor_driver_api bbtrackpad_api_funcs =
{
	.sample_fetch = bbtrackpad_fetch,
	.channel_get  = bbtrackpad_channel_get,
	.trigger_set  = bbtrackpad_trigger_set,
};

static struct bbtrackpad_data bbtrackpad_data = { };

static const struct bbtrackpad_dev_config bbtrackpad_config =
{
  .reset_pin = DT_INST_PROP(0, reset_pin),
  .power_pin = DT_INST_PROP(0, power_pin),
  .motion_pin = DT_INST_PROP(0, motion_pin)
};

DEVICE_DT_INST_DEFINE(0, bbtrackpad_init, NULL,
		    &bbtrackpad_data, &bbtrackpad_config, POST_KERNEL,
		    CONFIG_SENSOR_INIT_PRIORITY, &bbtrackpad_api_funcs);


