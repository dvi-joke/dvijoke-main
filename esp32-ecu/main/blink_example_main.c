#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "string.h"
#include "esp_log.h"
#include "cJSON.h"

// === Настройки UART ===
#define UART_PORT UART_NUM_0
#define UART_TX_PIN 5
#define UART_RX_PIN 6
#define BUF_SIZE (1024)

// === Настройки датчика оборотов ===
#define BUTTON_GPIO GPIO_NUM_0     // Входной сигнал (например, датчик Холла)
#define OUTPUT_PIN GPIO_NUM_3      // Пин управления транзистором

// === Настройки таблицы зажигания ===
#define MAX_RPM 6000               // Максимальные обороты
#define RPM_STEP 100               // Шаг по RPM
#define TABLE_SIZE (MAX_RPM / RPM_STEP + 1) // Размер массива углов

#define COIL_CHARGE_TIME 2000

static const char *TAG = "IGNITION_TABLE";

// Переменные для расчёта RPM
static int64_t last_press_time = 0;
static volatile int64_t rev_time = 0;
static volatile uint8_t changed = 0;
static volatile uint64_t last_rev_time = 0;

// Таблица зажигания: angle_table[i] = угол для диапазона i*RPM_STEP - (i+1)*RPM_STEP
static int angle_table[TABLE_SIZE] = {0};

// Таймер для управления транзистором
esp_timer_handle_t transistor_on_timer = NULL;
esp_timer_handle_t transistor_off_timer = NULL;

// === Включение транзистора ===
void IRAM_ATTR transistor_on_callback(void* arg) {
    gpio_set_level(OUTPUT_PIN, 1); // Включаем транзистор
}

// === Выключение транзистора через 2 мс ===
void IRAM_ATTR transistor_off_callback(void* arg) {
    gpio_set_level(OUTPUT_PIN, 0); // Выключаем транзистор
}

// === Обработчик прерываний от датчика ===
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint64_t current_time = esp_timer_get_time();
    uint64_t curr_pin_diff = current_time - last_press_time;

    if (curr_pin_diff < 700) return; // антидребезг

    if (curr_pin_diff > (rev_time * 2)) {
        rev_time = current_time - last_rev_time;
        changed = 1;
        last_rev_time = current_time;

        // === Расчёт угла и задержки ===
        uint64_t rpm = 60000000 / rev_time;
        int index = rpm / RPM_STEP;
        if (index >= TABLE_SIZE) index = TABLE_SIZE - 1;

        int angle = angle_table[index];
        uint64_t delay_us = (rev_time / 360) * angle;

        // === Запуск таймеров ===
        esp_timer_stop(transistor_on_timer);
        esp_timer_start_once(transistor_on_timer, delay_us);

        esp_timer_stop(transistor_off_timer);
        esp_timer_start_once(transistor_off_timer, delay_us + COIL_CHARGE_TIME); // через 2 мс выключить
    }

    last_press_time = current_time;
}


// === Парсинг JSON с таблицей зажигания ===
void parse_angle_table(const char *json_str) {
    cJSON *root = cJSON_Parse(json_str);
    if (!root || !cJSON_IsArray(root)) {
        ESP_LOGE(TAG, "Ошибка парсинга JSON или неверный формат");
        return;
    }

    int count = cJSON_GetArraySize(root);
    for (int i = 0; i < count && i < TABLE_SIZE; i++) {
        cJSON *item = cJSON_GetArrayItem(root, i);
        if (cJSON_IsNumber(item)) {
            angle_table[i] = item->valueint;
            ESP_LOGI(TAG, "[%d]: %d", i, angle_table[i]);
        }
    }

    ESP_LOGI(TAG, "Таблица зажигания загружена");
    cJSON_Delete(root);
}

// === Задача чтения UART ===
void uart_read_task(void *pvParameters) {
    uint8_t data[BUF_SIZE];
    while (1) {
        int len = uart_read_bytes(UART_PORT, data, BUF_SIZE - 1, pdMS_TO_TICKS(100));
        if (len > 0) {
            data[len] = '\0';
            ESP_LOGI(TAG, "Получено: %s", data);
            parse_angle_table((const char *)data);
        }
    }
}

// === Основная функция ===
void app_main(void) {
    // === Инициализация UART ===
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0);

    // === Инициализация GPIO ===
    gpio_reset_pin(BUTTON_GPIO);
    gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_GPIO, GPIO_PULLDOWN_ONLY);
    gpio_set_intr_type(BUTTON_GPIO, GPIO_INTR_POSEDGE);

    gpio_reset_pin(OUTPUT_PIN);
    gpio_set_direction(OUTPUT_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(OUTPUT_PIN, 0);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, gpio_isr_handler, (void*)BUTTON_GPIO);

    // === Создание одноразового таймера ===
    const esp_timer_create_args_t on_timer_args = {
        .callback = &transistor_on_callback,
        .name = "on_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&on_timer_args, &transistor_on_timer));

    const esp_timer_create_args_t off_timer_args = {
        .callback = &transistor_off_callback,
        .name = "off_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&off_timer_args, &transistor_off_timer));


    // === Запуск задачи чтения UART ===
    xTaskCreate(uart_read_task, "uart_read_task", 2048, NULL, 10, NULL);

    // === Основной цикл ===
    char json[128];
    while (1) {
        if (changed && rev_time > 0) {
            uint64_t rpm = 60000000 / rev_time;
            snprintf(json, sizeof(json), "{\"rpm\": %lld}", rpm);
            uart_write_bytes(UART_PORT, json, strlen(json));
            uart_write_bytes(UART_PORT, "\r\n", 2);
            changed = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}