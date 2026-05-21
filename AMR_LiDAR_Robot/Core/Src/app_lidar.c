#include "app_lidar.h"

#include "bringup_log.h"
#include "cmsis_os.h"
#include "usart.h"

#include <stdbool.h>
#include <stdint.h>

#define APP_LIDAR_BAUD_RATE 460800U
#define APP_LIDAR_LOG_INTERVAL_MS 1000U
#define APP_LIDAR_SAMPLE_SIZE 16U
#define APP_LIDAR_SAMPLE_TEXT_SIZE ((APP_LIDAR_SAMPLE_SIZE * 3U) + 1U)
#define APP_LIDAR_CMD_DELAY_MS 100U
#define APP_LIDAR_CMD_TIMEOUT_MS 100U

static HAL_StatusTypeDef App_Lidar_StartRx(void);
static void App_Lidar_SendStartupCommands(void);
static HAL_StatusTypeDef App_Lidar_SendCommand(const char *name, const uint8_t *command, uint16_t length);
static void App_Lidar_DelayMs(uint32_t delay_ms);
static void App_Lidar_CopySample(uint32_t *rx_bytes, uint8_t *sample, uint8_t *sample_count);
static void App_Lidar_FormatSample(const uint8_t *sample, uint8_t sample_count, char *text, uint32_t text_size);

static uint8_t app_lidar_rx_byte;
static volatile uint32_t app_lidar_rx_bytes;
static volatile uint8_t app_lidar_sample[APP_LIDAR_SAMPLE_SIZE];
static volatile uint8_t app_lidar_sample_write_index;
static volatile uint8_t app_lidar_sample_count;
static bool app_lidar_initialized;

void App_Lidar_Init(void)
{
    app_lidar_rx_bytes = 0U;
    app_lidar_sample_write_index = 0U;
    app_lidar_sample_count = 0U;
    app_lidar_initialized = true;

    HAL_StatusTypeDef status = App_Lidar_StartRx();
    if ((status == HAL_OK) || (status == HAL_BUSY))
    {
        APP_LOG("APP LiDAR: uart4 rx started, baud=%lu", (unsigned long)APP_LIDAR_BAUD_RATE);
        App_Lidar_SendStartupCommands();
        return;
    }

    APP_LOG("APP LiDAR: uart4 rx start failed, status=%u", (unsigned int)status);
}

void App_Lidar_Task(void)
{
    static uint32_t last_log_ms = 0U;
    uint32_t now_ms = HAL_GetTick();

    if (app_lidar_initialized && (huart4.RxState != HAL_UART_STATE_BUSY_RX))
    {
        (void)App_Lidar_StartRx();
    }

    if ((now_ms - last_log_ms) < APP_LIDAR_LOG_INTERVAL_MS)
    {
        return;
    }

    last_log_ms = now_ms;

    uint32_t rx_bytes = 0U;
    uint8_t sample[APP_LIDAR_SAMPLE_SIZE];
    uint8_t sample_count = 0U;
    char sample_text[APP_LIDAR_SAMPLE_TEXT_SIZE];

    App_Lidar_CopySample(&rx_bytes, sample, &sample_count);
    App_Lidar_FormatSample(sample, sample_count, sample_text, (uint32_t)sizeof(sample_text));

    APP_LOG("APP LiDAR: rx_bytes=%lu sample=%s", (unsigned long)rx_bytes, sample_text);
}

void App_Lidar_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != UART4))
    {
        return;
    }

    app_lidar_rx_bytes++;
    app_lidar_sample[app_lidar_sample_write_index] = app_lidar_rx_byte;
    app_lidar_sample_write_index = (uint8_t)((app_lidar_sample_write_index + 1U) % APP_LIDAR_SAMPLE_SIZE);

    if (app_lidar_sample_count < APP_LIDAR_SAMPLE_SIZE)
    {
        app_lidar_sample_count++;
    }

    (void)App_Lidar_StartRx();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    App_Lidar_RxCpltCallback(huart);
}

static HAL_StatusTypeDef App_Lidar_StartRx(void)
{
    return HAL_UART_Receive_IT(&huart4, &app_lidar_rx_byte, 1U);
}

static void App_Lidar_SendStartupCommands(void)
{
    static const uint8_t stop_cmd[] = {0xA5U, 0x25U};
    static const uint8_t scan_cmd[] = {0xA5U, 0x20U};

    (void)App_Lidar_SendCommand("STOP", stop_cmd, (uint16_t)sizeof(stop_cmd));
    App_Lidar_DelayMs(APP_LIDAR_CMD_DELAY_MS);
    (void)App_Lidar_SendCommand("SCAN", scan_cmd, (uint16_t)sizeof(scan_cmd));
}

static HAL_StatusTypeDef App_Lidar_SendCommand(const char *name, const uint8_t *command, uint16_t length)
{
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart4,
                                                 (uint8_t *)command,
                                                 length,
                                                 APP_LIDAR_CMD_TIMEOUT_MS);

    if (status == HAL_OK)
    {
        APP_LOG("APP LiDAR: send %s cmd", name);
    }
    else
    {
        APP_LOG("APP LiDAR: send %s cmd failed, status=%u", name, (unsigned int)status);
    }

    return status;
}

static void App_Lidar_DelayMs(uint32_t delay_ms)
{
    if (osKernelGetState() == osKernelRunning)
    {
        (void)osDelay(delay_ms);
        return;
    }

    HAL_Delay(delay_ms);
}

static void App_Lidar_CopySample(uint32_t *rx_bytes, uint8_t *sample, uint8_t *sample_count)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (rx_bytes != NULL)
    {
        *rx_bytes = app_lidar_rx_bytes;
    }

    if ((sample != NULL) && (sample_count != NULL))
    {
        uint8_t count = app_lidar_sample_count;
        uint8_t start = (uint8_t)((app_lidar_sample_write_index + APP_LIDAR_SAMPLE_SIZE - count) % APP_LIDAR_SAMPLE_SIZE);

        *sample_count = count;
        for (uint8_t i = 0U; i < count; i++)
        {
            sample[i] = app_lidar_sample[(start + i) % APP_LIDAR_SAMPLE_SIZE];
        }
    }

    if (primask == 0U)
    {
        __enable_irq();
    }
}

static void App_Lidar_FormatSample(const uint8_t *sample, uint8_t sample_count, char *text, uint32_t text_size)
{
    static const char hex[] = "0123456789ABCDEF";
    uint32_t pos = 0U;

    if ((text == NULL) || (text_size == 0U))
    {
        return;
    }

    if ((sample == NULL) || (sample_count == 0U))
    {
        if (text_size >= 3U)
        {
            text[0] = '-';
            text[1] = '-';
            text[2] = '\0';
        }
        else
        {
            text[0] = '\0';
        }
        return;
    }

    for (uint8_t i = 0U; i < sample_count; i++)
    {
        if ((i > 0U) && ((pos + 1U) < text_size))
        {
            text[pos++] = ' ';
        }

        if ((pos + 2U) >= text_size)
        {
            break;
        }

        text[pos++] = hex[(sample[i] >> 4) & 0x0FU];
        text[pos++] = hex[sample[i] & 0x0FU];
    }

    text[pos] = '\0';
}
