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
#include "nvs_flash.h"
#include "nvs.h"


#define NVS_NAMESPACE "ignition"
#define NVS_TABLE_KEY "angle_table"

// === Настройки UART ===
#define UART_PORT UART_NUM_1
#define UART_TX_PIN 5
#define UART_RX_PIN 6
#define BUF_SIZE (1024)

// === Настройки датчика оборотов ===
#define BUTTON_GPIO GPIO_NUM_0     // Входной сигнал (например, датчик Холла)
#define OUTPUT_PIN GPIO_NUM_7      // Пин управления транзистором
#define OUTPUT_PIN2 GPIO_NUM_8     // транзистор второй катушки

// === Настройки таблицы зажигания ===
#define MAX_RPM 6000               // Максимальные обороты
#define RPM_STEP 100               // Шаг по RPM
#define TABLE_SIZE ((MAX_RPM / RPM_STEP) + 1) // Размер массива углов

#define COIL_CHARGE_TIME_US 1000

#define SAMPLE_COUNT 3

static const char *TAG = "IGNITION_TABLE";

// Переменные для расчёта RPM
static int64_t last_interrupt_time = 0;
static volatile int64_t rev_period_us = 0;
static volatile uint8_t rpm_updated = 0;
static volatile uint64_t last_rev_time = 0;
static volatile uint64_t rpm = 0;

static volatile int64_t current_avg = 0;
static volatile uint8_t sample_count = 0;
// Таблица зажигания: angle_table[i] = угол для диапазона i*RPM_STEP - (i+1)*RPM_STEP
static int angle_table[TABLE_SIZE] = {30, 30, };

// Таймер для управления транзистором
esp_timer_handle_t transistor_on_timer14 = NULL;
esp_timer_handle_t transistor_off_timer14 = NULL;
esp_timer_handle_t transistor_on_timer23 = NULL;
esp_timer_handle_t transistor_off_timer23 = NULL;

// === Включение транзистора ===
void IRAM_ATTR transistor_on_callback14(void* arg) {
    gpio_set_level(OUTPUT_PIN, 1); // Включаем транзистор
}

// === Выключение транзистора через 1 мс ===
void IRAM_ATTR transistor_off_callback14(void* arg) {
    gpio_set_level(OUTPUT_PIN, 0); // Выключаем транзистор
}

// === Включение транзистора ===
void IRAM_ATTR transistor_on_callback23(void* arg) {
    gpio_set_level(OUTPUT_PIN2, 1); // Включаем транзистор
}

// === Выключение транзистора через 1 мс ===
void IRAM_ATTR transistor_off_callback23(void* arg) {
    gpio_set_level(OUTPUT_PIN2, 0); // Выключаем транзистор
}

void save_angle_table_to_nvs(int *table, size_t length) {
    nvs_handle_t my_handle;
    esp_err_t err;

    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка открытия NVS: %s", esp_err_to_name(err));
        return;
    }

    err = nvs_set_blob(my_handle, NVS_TABLE_KEY, table, length * sizeof(int));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Ошибка записи в NVS: %s", esp_err_to_name(err));
    } else {
        nvs_commit(my_handle);
        ESP_LOGI(TAG, "Таблица сохранена в NVS");
    }

    nvs_close(my_handle);
}

void load_angle_table_from_nvs(int *table, size_t length) {
    nvs_handle_t my_handle;
    esp_err_t err;

    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS не содержит таблицу зажигания");
        return;
    }

    size_t size = length * sizeof(int);
    err = nvs_get_blob(my_handle, NVS_TABLE_KEY, table, &size);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Ошибка чтения таблицы из NVS: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Таблица загружена из NVS");
    }

    nvs_close(my_handle);
}


// === Обработчик прерываний от датчика ===
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    // ESP_EARLY_LOGI(TAG, "GPIO ISR fired");

    int64_t current_time = esp_timer_get_time();
    int64_t diff = current_time - last_interrupt_time;

    // last_interrupt_time = current_time;
    
    // Антидребезг: игнорируем слишком маленькие/большие периоды
    if (diff < 1000) {
        return;
    }
    last_interrupt_time = current_time;
    uint64_t new_rpm = 60000000ULL / diff;

    if (new_rpm > rpm * 49 / 40 && rpm > 700) {
        return;
    }

    

    // Обновляем время последнего прерывания
    // last_interrupt_time = current_time;

    // Сохраняем текущий период
    rev_period_us = diff;
    rpm_updated = 1;

    // Рассчитываем RPM
    rpm = 60000000ULL / rev_period_us;

    // === Расчёт угла зажигания и управление катушкой ===
    int index = rpm / RPM_STEP;
    if (index >= TABLE_SIZE) index = TABLE_SIZE - 1;
    int angle_deg = angle_table[index] - 10;
    uint64_t delay_us = (rev_period_us * (angle_deg)) / 360;

    if (delay_us < COIL_CHARGE_TIME_US) {
        delay_us = COIL_CHARGE_TIME_US;
    }

    uint64_t delay_us23 = delay_us + (rev_period_us / 2);

    // Запуск таймеров
    esp_timer_stop(transistor_on_timer14);
    esp_timer_start_once(transistor_on_timer14, delay_us - COIL_CHARGE_TIME_US);

    esp_timer_stop(transistor_off_timer14);
    esp_timer_start_once(transistor_off_timer14, delay_us);

    esp_timer_stop(transistor_on_timer23);
    esp_timer_start_once(transistor_on_timer23, delay_us23 - COIL_CHARGE_TIME_US);

    esp_timer_stop(transistor_off_timer23);
    esp_timer_start_once(transistor_off_timer23, delay_us23);
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

    // Сохраняем новую таблицу в NVS
    save_angle_table_to_nvs(angle_table, TABLE_SIZE);

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
    // === Инициализация NVS ===
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    load_angle_table_from_nvs(angle_table, TABLE_SIZE);


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
    uart_driver_install(UART_PORT, BUF_SIZE, BUF_SIZE, 0, NULL, 0);

    // === Инициализация GPIO ===
    gpio_reset_pin(BUTTON_GPIO);
    gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);
    // gpio_set_pull_mode(BUTTON_GPIO, GPIO_PULLDOWN_ONLY);
    gpio_set_intr_type(BUTTON_GPIO, GPIO_INTR_POSEDGE);

    gpio_reset_pin(OUTPUT_PIN);
    gpio_set_direction(OUTPUT_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(OUTPUT_PIN, 0);
    gpio_reset_pin(OUTPUT_PIN2);
    gpio_set_direction(OUTPUT_PIN2, GPIO_MODE_OUTPUT);
    gpio_set_level(OUTPUT_PIN2, 0);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, gpio_isr_handler, (void*)BUTTON_GPIO);

    // === Создание одноразового таймера ===
    const esp_timer_create_args_t on_timer_args14 = {
        .callback = &transistor_on_callback14,
        .name = "on_timer14"
    };
    ESP_ERROR_CHECK(esp_timer_create(&on_timer_args14, &transistor_on_timer14));

    const esp_timer_create_args_t off_timer_args = {
        .callback = &transistor_off_callback14,
        .name = "off_timer14"
    };
    ESP_ERROR_CHECK(esp_timer_create(&off_timer_args, &transistor_off_timer14));

    // === Создание одноразового таймера ===
    const esp_timer_create_args_t on_timer_args23 = {
        .callback = &transistor_on_callback23,
        .name = "on_timer23"
    };
    ESP_ERROR_CHECK(esp_timer_create(&on_timer_args23, &transistor_on_timer23));

    const esp_timer_create_args_t off_timer_args23 = {
        .callback = &transistor_off_callback23,
        .name = "off_timer23"
    };
    ESP_ERROR_CHECK(esp_timer_create(&off_timer_args23, &transistor_off_timer23));


    // === Запуск задачи чтения UART ===
    xTaskCreate(uart_read_task, "uart_read_task", 4096, NULL, 10, NULL);

    // === Основной цикл ===
    char json[128];
    int angle;
    while (1) {
        if (rpm_updated) {
            uint64_t rpm = 60000000 / rev_period_us;
            int index = rpm / RPM_STEP;
            if (index >= TABLE_SIZE) index = TABLE_SIZE - 1;

            angle = angle_table[index];
            
        } else {
            rpm = 0;
            angle = angle_table[0];
        }
        snprintf(json, sizeof(json), "{\"rpm\": %lld, \"angle\": %d}", rpm, angle);
        uart_write_bytes(UART_PORT, json, strlen(json));
        uart_write_bytes(UART_PORT, "\r\n", 2);
        ESP_LOGI(TAG, "{\"rpm\": %lld, \"angle\": %d}", rpm, angle);
        rpm_updated = 0;
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}