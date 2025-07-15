#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include "icm20948.h"


LOG_MODULE_REGISTER(ICM20948_TRIG, CONFIG_SENSOR_LOG_LEVEL);



static void icm20948_handle_interrupt(const struct device *dev)
{
    struct icm20948_data *data = dev->data;

#if defined(CONFIG_ICM20948_TRIGGER_OWN_THREAD)
    k_sem_give(&data->gpio_sem);
#elif defined(CONFIG_ICM20948_TRIGGER_GLOBAL_THREAD)
    k_work_submit(&data->work);
#endif
}

static void icm20948_gpio_callback(const struct device *port,
                                   struct gpio_callback *cb, uint32_t pins)
{
    struct icm20948_data *data = CONTAINER_OF(cb, struct icm20948_data, gpio_cb);
    const struct device *dev = data->dev;

    ARG_UNUSED(port);
    ARG_UNUSED(pins);

    icm20948_handle_interrupt(dev);
}

static void icm20948_thread_cb(const struct device *dev)
{
    struct icm20948_data *data = dev->data;
    uint8_t int_status;
    uint8_t int_status_1;
    uint8_t int_status_2;
    uint8_t int_status_3;
    int ret;

    /* Read all interrupt status registers to determine interrupt source */
    ret = icm20948_read_reg(dev, ICM20948_BANK_0, ICM20948_INT_STATUS, &int_status);
    if (ret < 0) {
        LOG_ERR("Failed to read INT_STATUS");
        return;
    }

    ret = icm20948_read_reg(dev, ICM20948_BANK_0, ICM20948_INT_STATUS_1, &int_status_1);
    if (ret < 0) {
        LOG_ERR("Failed to read INT_STATUS_1");
        return;
    }

    ret = icm20948_read_reg(dev, ICM20948_BANK_0, ICM20948_INT_STATUS_2, &int_status_2);
    if (ret < 0) {
        LOG_ERR("Failed to read INT_STATUS_2");
        return;
    }

    ret = icm20948_read_reg(dev, ICM20948_BANK_0, ICM20948_INT_STATUS_3, &int_status_3);
    if (ret < 0) {
        LOG_ERR("Failed to read INT_STATUS_3");
        return;
    }

    /* Handle data ready interrupt */
    if (data->data_ready_handler != NULL && (int_status_1 & BIT(0))) {
        data->data_ready_handler(dev, data->data_ready_trigger);
    }

    /* Handle Wake-on-Motion interrupt */
    if (int_status & BIT(3)) {
        LOG_DBG("Wake-on-Motion interrupt detected");
        /* Could trigger a WOM-specific callback here if implemented */
    }

    /* Handle FIFO overflow interrupts */
    if (int_status_2 & 0x1F) {
        LOG_WRN("FIFO overflow detected: 0x%02x", int_status_2 & 0x1F);
        /* Could trigger FIFO overflow callback here if implemented */
    }

    /* Handle FIFO watermark interrupts */
    if (int_status_3 & 0x1F) {
        LOG_DBG("FIFO watermark reached: 0x%02x", int_status_3 & 0x1F);
        /* Could trigger FIFO watermark callback here if implemented */
    }

    /* Handle I2C Master interrupt */
    if (int_status & BIT(6)) {
        LOG_DBG("I2C Master interrupt detected");
        /* Could handle I2C master status here if needed */
    }

    /* Handle PLL ready interrupt */
    if (int_status & BIT(2)) {
        LOG_DBG("PLL ready interrupt detected");
    }
}

#ifdef CONFIG_ICM20948_TRIGGER_OWN_THREAD
static void icm20948_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    struct icm20948_data *data = p1;
    const struct device *dev = data->dev;

    while (1) {
        k_sem_take(&data->gpio_sem, K_FOREVER);
        icm20948_thread_cb(dev);
    }
}
#endif

#ifdef CONFIG_ICM20948_TRIGGER_GLOBAL_THREAD
static void icm20948_work_callback(struct k_work *work)
{
    struct icm20948_data *data = CONTAINER_OF(work, struct icm20948_data, work);
    const struct device *dev = data->dev;

    icm20948_thread_cb(dev);
}
#endif

int icm20948_trigger_set(const struct device *dev,
                        const struct sensor_trigger *trig,
                        sensor_trigger_handler_t handler)
{
    struct icm20948_data *data = dev->data;
    int ret;

    if (trig->type != SENSOR_TRIG_DATA_READY) {
        LOG_ERR("Unsupported trigger type: %d", trig->type);
        return -ENOTSUP;
    }

    /* Store the handler and trigger */
    data->data_ready_handler = handler;
    data->data_ready_trigger = trig;

    /* Enable/disable data ready interrupt */
    ret = icm20948_int_enable_raw_data_ready(dev, (handler != NULL));
    if (ret < 0) {
        LOG_ERR("Failed to configure data ready interrupt");
        return ret;
    }

    LOG_DBG("Data ready trigger %s", handler ? "enabled" : "disabled");

    return 0;
}

int icm20948_init_interrupt(const struct device *dev)
{
    struct icm20948_data *data = dev->data;
    const struct icm20948_config *cfg = dev->config;
    int ret;

    /* Store device reference */
    data->dev = dev;

    /* Check if GPIO is ready */
    if (!gpio_is_ready_dt(&cfg->int_gpio)) {
        LOG_ERR("Interrupt GPIO port not ready");
        return -ENODEV;
    }

    /* Configure interrupt pin */
    ret = gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT | cfg->int_gpio.dt_flags);
    if (ret < 0) {
        LOG_ERR("Could not configure interrupt GPIO pin: %d", ret);
        return ret;
    }

    /* Initialize GPIO callback */
    gpio_init_callback(&data->gpio_cb, icm20948_gpio_callback, BIT(cfg->int_gpio.pin));

    /* Add GPIO callback */
    ret = gpio_add_callback(cfg->int_gpio.port, &data->gpio_cb);
    if (ret < 0) {
        LOG_ERR("Could not add GPIO callback: %d", ret);
        return ret;
    }

    /* Configure interrupt edge */
    ret = gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) {
        LOG_ERR("Could not configure interrupt on GPIO pin: %d", ret);
        return ret;
    }

#if defined(CONFIG_ICM20948_TRIGGER_OWN_THREAD)
    /* Initialize semaphore */
    k_sem_init(&data->gpio_sem, 0, K_SEM_MAX_LIMIT);

    /* Create thread */
    k_thread_create(&data->thread, data->thread_stack,
                    CONFIG_ICM20948_THREAD_STACK_SIZE,
                    icm20948_thread, data, NULL, NULL,
                    K_PRIO_COOP(CONFIG_ICM20948_THREAD_PRIORITY), 0, K_NO_WAIT);
#elif defined(CONFIG_ICM20948_TRIGGER_GLOBAL_THREAD)
    /* Initialize work */
    k_work_init(&data->work, icm20948_work_callback);
#endif

    /* Configure interrupt pin characteristics */
    ret = icm20948_cfg_int_active_low(dev, false); /* Active high */
    if (ret < 0) {
        LOG_ERR("Failed to configure interrupt polarity: %d", ret);
        return ret;
    }

    ret = icm20948_cfg_int_open_drain(dev, false); /* Push-pull */
    if (ret < 0) {
        LOG_ERR("Failed to configure interrupt output type: %d", ret);
        return ret;
    }

    ret = icm20948_cfg_int_latch(dev, false); /* Pulse mode */
    if (ret < 0) {
        LOG_ERR("Failed to configure interrupt latch mode: %d", ret);
        return ret;
    }

    LOG_INF("ICM20948 interrupt initialized successfully");

    return 0;
}

/* Additional trigger-related helper functions */
int icm20948_trigger_enable_wom(const struct device *dev, 
                               const struct sensor_trigger *trig,
                               sensor_trigger_handler_t handler)
{
    int ret;
    
    /* This would be called to enable Wake-on-Motion triggers */
    /* For now, just enable the WOM interrupt */
    ret = icm20948_int_enable_wom(dev, (handler != NULL));
    if (ret < 0) {
        return ret;
    }
    
    /* Configure WOM logic if enabling */
    if (handler != NULL) {
        ret = icm20948_wom_logic(dev, 1, 1); /* Enable WOM with interrupt mode */
        if (ret < 0) {
            return ret;
        }
        
        /* Set a default threshold - this could be made configurable */
        ret = icm20948_wom_threshold(dev, 50); /* Threshold value */
        if (ret < 0) {
            return ret;
        }
    }
    
    return 0;
}

int icm20948_trigger_enable_fifo(const struct device *dev,
                                const struct sensor_trigger *trig,
                                sensor_trigger_handler_t handler)
{
    int ret;
    
    /* Enable FIFO overflow or watermark interrupts */
    if (trig->type == SENSOR_TRIG_FIFO_FULL) {
        ret = icm20948_int_enable_fifo_overflow(dev, handler ? 0x1F : 0x00);
    } else if (trig->type == SENSOR_TRIG_FIFO_WATERMARK) {
        ret = icm20948_int_enable_fifo_watermark(dev, handler ? 0x1F : 0x00);
    } else {
        return -ENOTSUP;
    }
    
    return ret;
}

