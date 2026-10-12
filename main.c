/*
 * Robot Sensor Monitor
 * --------------------
 * ESP32 + FreeRTOS (ESP-IDF) project, simulated on Wokwi.
 *
 * Tasks
 *   sensor_task  - reads the MPU6050 IMU over I2C every 500 ms,
 *                  detects tilt alarms and sensor (I2C) faults
 *   uart_task    - receives readings from a queue and prints them over UART
 *   led_task     - shows system status on an LED
 *   button_task  - woken by a GPIO interrupt (via semaphore), toggles pause
 *
 * Status LED
 *   slow blink (1 Hz)  -> normal
 *   fast blink (5 Hz)  -> tilt alarm (robot tilted more than TILT_LIMIT_DEG)
 *   solid ON           -> sensor fault (I2C reads failing)
 *   OFF                -> paused by the button
 */

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_log.h"

/* ---------- Pin mapping ---------- */
#define I2C_SDA_PIN        GPIO_NUM_21
#define I2C_SCL_PIN        GPIO_NUM_22
#define LED_PIN            GPIO_NUM_2
#define BUTTON_PIN         GPIO_NUM_4
#define UART_PORT          UART_NUM_0

/* ---------- MPU6050 registers ---------- */
#define MPU6050_ADDR       0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT 0x3B   /* first of 14 data bytes: accel, temp, gyro */
#define ACCEL_LSB_PER_G    16384.0f  /* +/-2 g full-scale range */

/* ---------- Settings ---------- */
#define SAMPLE_PERIOD_MS   500
#define TILT_LIMIT_DEG     30.0f
#define MAX_I2C_FAILURES   3
#define I2C_TIMEOUT_MS     100
#define QUEUE_LENGTH       10
#define RAD_TO_DEG         (180.0f / 3.14159265f)

/* ---------- Status bits (event group) ---------- */
#define BIT_PAUSED         BIT0
#define BIT_ALARM          BIT1
#define BIT_FAULT          BIT2

typedef struct {
    uint32_t sample_no;
    float ax, ay, az;     /* acceleration in g */
    float tilt_deg;       /* angle between Z axis and gravity */
    float temp_c;
} reading_t;

static const char *TAG = "monitor";

static i2c_master_dev_handle_t mpu_dev;
static QueueHandle_t      reading_queue;   /* sensor_task -> uart_task */
static SemaphoreHandle_t  button_sem;      /* button ISR  -> button_task */
static EventGroupHandle_t status_bits;     /* shared system state */

/* =====================================================================
 * MPU6050 driver
 * ===================================================================== */

static esp_err_t mpu6050_wake(void)
{
    /* The MPU6050 starts in sleep mode; writing 0 to PWR_MGMT_1 wakes it. */
    uint8_t cmd[2] = { MPU6050_PWR_MGMT_1, 0x00 };
    return i2c_master_transmit(mpu_dev, cmd, sizeof(cmd), I2C_TIMEOUT_MS);
}

static esp_err_t mpu6050_init(void)
{
    i2c_master_bus_handle_t bus;

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &mpu_dev));

    return mpu6050_wake();
}

static esp_err_t mpu6050_read(reading_t *r)
{
    uint8_t reg = MPU6050_ACCEL_XOUT;
    uint8_t d[14];

    esp_err_t err = i2c_master_transmit_receive(mpu_dev, &reg, 1,
                                                d, sizeof(d), I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }

    /* Data is big-endian: high byte first. */
    int16_t raw_ax = (int16_t)((d[0] << 8) | d[1]);
    int16_t raw_ay = (int16_t)((d[2] << 8) | d[3]);
    int16_t raw_az = (int16_t)((d[4] << 8) | d[5]);
    int16_t raw_t  = (int16_t)((d[6] << 8) | d[7]);

    r->ax = raw_ax / ACCEL_LSB_PER_G;
    r->ay = raw_ay / ACCEL_LSB_PER_G;
    r->az = raw_az / ACCEL_LSB_PER_G;
    r->temp_c = raw_t / 340.0f + 36.53f;   /* formula from the datasheet */

    /* Tilt = angle between the sensor's Z axis and the gravity vector. */
    float mag = sqrtf(r->ax * r->ax + r->ay * r->ay + r->az * r->az);
    if (mag < 0.01f) {
        r->tilt_deg = 0.0f;                /* avoid divide-by-zero */
    } else {
        float c = r->az / mag;
        if (c > 1.0f)  c = 1.0f;           /* guard against rounding */
        if (c < -1.0f) c = -1.0f;
        r->tilt_deg = acosf(c) * RAD_TO_DEG;
    }
    return ESP_OK;
}

/* =====================================================================
 * Tasks
 * ===================================================================== */

static void sensor_task(void *arg)
{
    reading_t r = { 0 };
    uint32_t sample_no = 0;
    int failures = 0;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        /* Fixed sampling period, independent of how long the read takes. */
        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));

        if (xEventGroupGetBits(status_bits) & BIT_PAUSED) {
            continue;
        }

        if (mpu6050_read(&r) != ESP_OK) {
            failures++;
            ESP_LOGW(TAG, "I2C read failed (%d/%d)", failures, MAX_I2C_FAILURES);
            if (failures >= MAX_I2C_FAILURES) {
                if (!(xEventGroupGetBits(status_bits) & BIT_FAULT)) {
                    ESP_LOGE(TAG, "SENSOR FAULT: MPU6050 not responding");
                }
                xEventGroupSetBits(status_bits, BIT_FAULT);
                mpu6050_wake();            /* try to recover the sensor */
            }
            continue;
        }

        /* Successful read: clear any earlier fault. */
        if (failures >= MAX_I2C_FAILURES) {
            ESP_LOGI(TAG, "Sensor recovered");
        }
        failures = 0;
        xEventGroupClearBits(status_bits, BIT_FAULT);

        if (r.tilt_deg > TILT_LIMIT_DEG) {
            xEventGroupSetBits(status_bits, BIT_ALARM);
        } else {
            xEventGroupClearBits(status_bits, BIT_ALARM);
        }

        r.sample_no = ++sample_no;
        if (xQueueSend(reading_queue, &r, 0) != pdTRUE) {
            ESP_LOGW(TAG, "Queue full, reading dropped");
        }
    }
}

static void uart_task(void *arg)
{
    reading_t r;
    char line[160];

    for (;;) {
        /* Blocks until sensor_task sends a reading. */
        if (xQueueReceive(reading_queue, &r, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        const char *state = (r.tilt_deg > TILT_LIMIT_DEG) ? "TILT ALARM" : "OK";
        int len = snprintf(line, sizeof(line),
                           "#%-4lu ax=%5.2f ay=%5.2f az=%5.2f g | tilt=%5.1f deg | temp=%5.1f C | %s\r\n",
                           (unsigned long)r.sample_no, r.ax, r.ay, r.az,
                           r.tilt_deg, r.temp_c, state);
        if (len > (int)sizeof(line) - 1) {
            len = sizeof(line) - 1;
        }
        uart_write_bytes(UART_PORT, line, len);
    }
}

static void led_task(void *arg)
{
    int level = 0;

    for (;;) {
        EventBits_t bits = xEventGroupGetBits(status_bits);

        if (bits & BIT_PAUSED) {
            gpio_set_level(LED_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(200));
        } else if (bits & BIT_FAULT) {
            gpio_set_level(LED_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(200));
        } else {
            level = !level;
            gpio_set_level(LED_PIN, level);
            vTaskDelay(pdMS_TO_TICKS((bits & BIT_ALARM) ? 100 : 500));
        }
    }
}

/* Keep the ISR short: just wake the button task. */
static void IRAM_ATTR button_isr(void *arg)
{
    BaseType_t higher_prio_woken = pdFALSE;
    xSemaphoreGiveFromISR(button_sem, &higher_prio_woken);
    portYIELD_FROM_ISR(higher_prio_woken);
}

static void button_task(void *arg)
{
    for (;;) {
        xSemaphoreTake(button_sem, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(50));           /* debounce */
        if (gpio_get_level(BUTTON_PIN) == 0) {
            if (xEventGroupGetBits(status_bits) & BIT_PAUSED) {
                xEventGroupClearBits(status_bits, BIT_PAUSED);
                ESP_LOGI(TAG, "Monitoring RESUMED");
            } else {
                xEventGroupSetBits(status_bits, BIT_PAUSED);
                ESP_LOGI(TAG, "Monitoring PAUSED");
            }
            while (gpio_get_level(BUTTON_PIN) == 0) {  /* wait for release */
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        xSemaphoreTake(button_sem, 0);           /* discard bounce interrupts */
    }
}

/* =====================================================================
 * Setup
 * ===================================================================== */

static void gpio_setup(void)
{
    gpio_config_t led_cfg = {
        .pin_bit_mask = 1ULL << LED_PIN,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&led_cfg));

    gpio_config_t btn_cfg = {
        .pin_bit_mask = 1ULL << BUTTON_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,       /* button connects pin to GND */
        .intr_type = GPIO_INTR_NEGEDGE,         /* interrupt on press */
    };
    ESP_ERROR_CHECK(gpio_config(&btn_cfg));

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BUTTON_PIN, button_isr, NULL));
}

void app_main(void)
{
    /* RTOS objects */
    reading_queue = xQueueCreate(QUEUE_LENGTH, sizeof(reading_t));
    button_sem    = xSemaphoreCreateBinary();
    status_bits   = xEventGroupCreate();
    if (!reading_queue || !button_sem || !status_bits) {
        ESP_LOGE(TAG, "Failed to create RTOS objects");
        return;
    }

    /* Peripherals */
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, 1024, 0, 0, NULL, 0));
    gpio_setup();

    if (mpu6050_init() != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 not found at startup");
        xEventGroupSetBits(status_bits, BIT_FAULT);
    }

    const char *banner = "\r\n=== Robot Sensor Monitor (ESP32 + FreeRTOS) ===\r\n";
    uart_write_bytes(UART_PORT, banner, strlen(banner));

    /* Tasks: name, stack size (bytes), priority */
    xTaskCreate(sensor_task, "sensor", 4096, NULL, 5, NULL);
    xTaskCreate(uart_task,   "uart",   4096, NULL, 4, NULL);
    xTaskCreate(button_task, "button", 2048, NULL, 6, NULL);
    xTaskCreate(led_task,    "led",    2048, NULL, 3, NULL);
}
