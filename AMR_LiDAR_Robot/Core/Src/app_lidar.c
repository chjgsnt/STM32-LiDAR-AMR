#include "app_lidar.h"

#include "app_serial_command.h"
#include "bringup_log.h"
#include "cmsis_os.h"
#include "usart.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define APP_LIDAR_BAUD_RATE 460800U
#define APP_LIDAR_LOG_INTERVAL_MS 2000U
#define APP_LIDAR_STATUS_INTERVAL_MS 100U
#define APP_LIDAR_SAMPLE_SIZE 8U
#define APP_LIDAR_DESCRIPTOR_SIZE 7U
#define APP_LIDAR_SCAN_NODE_SIZE 5U
#define APP_LIDAR_SAMPLE_TEXT_SIZE ((APP_LIDAR_SAMPLE_SIZE * 3U) + 1U)
#define APP_LIDAR_DESCRIPTOR_TEXT_SIZE ((APP_LIDAR_DESCRIPTOR_SIZE * 3U) + 1U)
#define APP_LIDAR_CMD_DELAY_MS 100U
#define APP_LIDAR_CMD_TIMEOUT_MS 100U
#define APP_LIDAR_MIN_DISTANCE_MM 50U
#define APP_LIDAR_ZONE_MIN_DISTANCE_MM 120U
#define APP_LIDAR_MAX_DISTANCE_MM 12000U
#define APP_LIDAR_MIN_DISTANCE_NONE 0xFFFFFFFFUL
#define APP_LIDAR_STATUS_DISTANCE_INVALID 0xFFFFU
#define APP_LIDAR_ZONE_MIN_QUALITY 8U
#define APP_LIDAR_RX_DEBUG_INTERVAL_BYTES 50000U
#define APP_LIDAR_FRONT_WIDE_MIN_TENTH_DEG 100U
#define APP_LIDAR_FRONT_WIDE_MAX_TENTH_DEG 800U
#define APP_LIDAR_FRONT_MIN_TENTH_DEG 200U
#define APP_LIDAR_FRONT_MAX_TENTH_DEG 650U
#define APP_LIDAR_LEFT_MIN_TENTH_DEG 850U
#define APP_LIDAR_LEFT_MAX_TENTH_DEG 1300U
#define APP_LIDAR_RIGHT_MIN_TENTH_DEG 2450U
#define APP_LIDAR_RIGHT_MAX_TENTH_DEG 3050U

#define APP_LIDAR_VERBOSE_LOGS (APP_DEBUG_VERBOSE || APP_DEBUG_LIDAR_VERBOSE)

#define APP_LIDAR_DESCRIPTOR_SEARCH_A5 0U
#define APP_LIDAR_DESCRIPTOR_SEARCH_5A 1U
#define APP_LIDAR_DESCRIPTOR_COLLECT 2U

typedef struct
{
    uint32_t rx_bytes;
    uint32_t parsed_points;
    uint32_t valid_points;
    uint32_t invalid_header;
    uint32_t invalid_range;
    uint32_t zero_quality;
    uint32_t min_distance_mm;
    uint32_t front_min_mm;
    uint32_t front_wide_min_mm;
    uint32_t left_min_mm;
    uint32_t right_min_mm;
    uint32_t nearest_distance_mm;
    uint32_t nearest_angle_tenth_deg;
    uint32_t last_angle_tenth_deg;
    uint32_t last_rx_tick_ms;
    uint32_t last_valid_update_ms;
    uint32_t uart4_error_count;
    uint32_t uart4_error_code;
    uint32_t scan_start_count;
    uint32_t descriptor_payload_len;
    uint8_t ready;
    uint8_t descriptor_seen;
    uint8_t descriptor_pending;
    uint8_t descriptor[APP_LIDAR_DESCRIPTOR_SIZE];
    uint8_t descriptor_data_type;
    uint8_t descriptor_send_mode;
    uint8_t has_valid_distance;
    uint8_t has_front_distance;
    uint8_t has_front_wide_distance;
    uint8_t has_left_distance;
    uint8_t has_right_distance;
    uint8_t has_nearest;
    uint8_t nearest_quality;
    uint8_t has_angle;
    uint8_t sample_count;
    uint8_t sample[APP_LIDAR_SAMPLE_SIZE];
} App_Lidar_Snapshot_t;

typedef struct
{
    bool header_valid;
    uint8_t start_flag;
    uint8_t quality;
    uint32_t angle_q6;
    uint32_t distance_q2;
    uint32_t angle_tenth_deg;
    uint32_t distance_mm;
} App_Lidar_ScanNode_t;

static HAL_StatusTypeDef App_Lidar_StartRx(void);
static void App_Lidar_LogRxDebug(void);
static void App_Lidar_SendStartupCommands(void);
static HAL_StatusTypeDef App_Lidar_SendCommand(const char *name, const uint8_t *command, uint16_t length);
static void App_Lidar_DelayMs(uint32_t delay_ms);
static void App_Lidar_ResetParser(void);
static void App_Lidar_FeedParser(uint8_t byte);
static bool App_Lidar_FeedDescriptorParser(uint8_t byte);
static void App_Lidar_OnDescriptorComplete(void);
static void App_Lidar_ProcessScanNodeCandidate(void);
static App_Lidar_ScanNode_t App_Lidar_DecodeScanNode(const uint8_t *node);
static void App_Lidar_UpdateZoneMins(const App_Lidar_ScanNode_t *node);
static void App_Lidar_ShiftNodeBuffer(void);
static void App_Lidar_CopySnapshot(App_Lidar_Snapshot_t *snapshot);
static void App_Lidar_UpdatePublishedStatus(const App_Lidar_Snapshot_t *snapshot);
static void App_Lidar_MarkDescriptorPrinted(void);
#if APP_LIDAR_VERBOSE_LOGS
static void App_Lidar_FormatBytes(const uint8_t *bytes, uint8_t count, char *text, uint32_t text_size);
static uint32_t App_Lidar_ElapsedMs(uint32_t now_ms, uint32_t then_ms);
#endif

static uint8_t app_lidar_rx_byte;
static volatile uint32_t app_lidar_rx_bytes;
static volatile uint8_t app_lidar_sample[APP_LIDAR_SAMPLE_SIZE];
static volatile uint8_t app_lidar_sample_write_index;
static volatile uint8_t app_lidar_sample_count;

static uint8_t app_lidar_descriptor_state;
static uint8_t app_lidar_descriptor_index;
static uint8_t app_lidar_descriptor_buffer[APP_LIDAR_DESCRIPTOR_SIZE];
static uint8_t app_lidar_node_buffer[APP_LIDAR_SCAN_NODE_SIZE];
static uint8_t app_lidar_node_count;

static volatile uint8_t app_lidar_descriptor_seen;
static volatile uint8_t app_lidar_descriptor_pending;
static volatile uint8_t app_lidar_descriptor_latest[APP_LIDAR_DESCRIPTOR_SIZE];
static volatile uint8_t app_lidar_descriptor_data_type;
static volatile uint8_t app_lidar_descriptor_send_mode;
static volatile uint32_t app_lidar_descriptor_payload_len;

static volatile uint32_t app_lidar_parsed_points;
static volatile uint32_t app_lidar_valid_points;
static volatile uint32_t app_lidar_invalid_header;
static volatile uint32_t app_lidar_invalid_range;
static volatile uint32_t app_lidar_zero_quality;
static volatile uint32_t app_lidar_min_distance_mm;
static volatile uint32_t app_lidar_front_min_mm;
static volatile uint32_t app_lidar_front_wide_min_mm;
static volatile uint32_t app_lidar_left_min_mm;
static volatile uint32_t app_lidar_right_min_mm;
static volatile uint32_t app_lidar_nearest_distance_mm;
static volatile uint32_t app_lidar_nearest_angle_tenth_deg;
static volatile uint32_t app_lidar_last_angle_tenth_deg;
static volatile uint32_t app_lidar_last_rx_tick;
static volatile uint32_t app_lidar_last_valid_update_tick;
static volatile uint32_t app_lidar_uart4_error_count;
static volatile uint32_t app_lidar_uart4_error_code;
static volatile uint32_t app_lidar_scan_start_count;
static volatile uint8_t app_lidar_has_valid_distance;
static volatile uint8_t app_lidar_has_front_distance;
static volatile uint8_t app_lidar_has_front_wide_distance;
static volatile uint8_t app_lidar_has_left_distance;
static volatile uint8_t app_lidar_has_right_distance;
static volatile uint8_t app_lidar_has_nearest;
static volatile uint8_t app_lidar_nearest_quality;
static volatile uint8_t app_lidar_has_angle;
static volatile uint8_t app_lidar_callback_entered_logged;
static volatile uint8_t app_lidar_callback_entered_log_pending;
static volatile uint8_t app_lidar_first_rx_log_pending;
static volatile uint32_t app_lidar_rx_count_log_pending;
static volatile uint32_t app_lidar_uart4_error_log_pending;
static volatile uint32_t app_lidar_uart4_error_code_log_pending;

static AppLidarStatus app_lidar_status;
static bool app_lidar_initialized;

const AppLidarStatus *App_Lidar_GetStatus(void)
{
    return &app_lidar_status;
}

void App_Lidar_Init(void)
{
    app_lidar_rx_bytes = 0U;
    app_lidar_sample_write_index = 0U;
    app_lidar_sample_count = 0U;
    app_lidar_status.ready = 0U;
    app_lidar_status.rx_bytes = 0U;
    app_lidar_status.valid_points = 0U;
    app_lidar_status.front_min_mm = APP_LIDAR_STATUS_DISTANCE_INVALID;
    app_lidar_status.front_wide_min_mm = APP_LIDAR_STATUS_DISTANCE_INVALID;
    app_lidar_status.left_min_mm = APP_LIDAR_STATUS_DISTANCE_INVALID;
    app_lidar_status.right_min_mm = APP_LIDAR_STATUS_DISTANCE_INVALID;
    app_lidar_status.front_valid = 0U;
    app_lidar_status.front_wide_valid = 0U;
    app_lidar_status.left_valid = 0U;
    app_lidar_status.right_valid = 0U;
    app_lidar_status.nearest_angle_deg = 0.0f;
    app_lidar_status.nearest_distance_mm = APP_LIDAR_STATUS_DISTANCE_INVALID;
    app_lidar_status.last_rx_tick_ms = 0U;
    app_lidar_status.last_valid_update_ms = 0U;
    app_lidar_status.last_update_ms = 0U;
    app_lidar_status.uart4_error_count = 0U;
    app_lidar_status.uart4_error_code = 0U;
    app_lidar_last_rx_tick = 0U;
    app_lidar_last_valid_update_tick = 0U;
    app_lidar_uart4_error_count = 0U;
    app_lidar_uart4_error_code = 0U;
    app_lidar_callback_entered_logged = 0U;
    app_lidar_callback_entered_log_pending = 0U;
    app_lidar_first_rx_log_pending = 0U;
    app_lidar_rx_count_log_pending = 0U;
    app_lidar_uart4_error_log_pending = 0U;
    app_lidar_uart4_error_code_log_pending = 0U;
    App_Lidar_ResetParser();
    app_lidar_initialized = true;

    HAL_StatusTypeDef status = App_Lidar_StartRx();
    APP_LOG("[LIDAR_RX] arm ret=%u state=0x%02X error=0x%08lX",
            (unsigned int)status,
            (unsigned int)huart4.RxState,
            (unsigned long)huart4.ErrorCode);
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
    static uint32_t last_status_ms = 0U;
    uint32_t now_ms = HAL_GetTick();
    uint8_t log_due = 0U;
    uint8_t status_due = 0U;

    if (app_lidar_initialized && (huart4.RxState != HAL_UART_STATE_BUSY_RX))
    {
        (void)App_Lidar_StartRx();
    }

    App_Lidar_LogRxDebug();

    if ((now_ms - last_log_ms) >= APP_LIDAR_LOG_INTERVAL_MS)
    {
        log_due = 1U;
    }

    if ((now_ms - last_status_ms) >= APP_LIDAR_STATUS_INTERVAL_MS)
    {
        status_due = 1U;
    }

    if ((log_due == 0U) && (status_due == 0U))
    {
        return;
    }

    App_Lidar_Snapshot_t snapshot;
#if APP_LIDAR_VERBOSE_LOGS
    char sample_text[APP_LIDAR_SAMPLE_TEXT_SIZE];
    char descriptor_text[APP_LIDAR_DESCRIPTOR_TEXT_SIZE];
    char type_text[8];
    char debug_min_text[16];
    char front_text[16];
    char front_wide_text[16];
    char left_text[16];
    char right_text[16];
    char nearest_text[32];
    char angle_text[16];
    uint32_t rx_age_ms;
    uint32_t valid_age_ms;
#endif

    App_Lidar_CopySnapshot(&snapshot);
    App_Lidar_UpdatePublishedStatus(&snapshot);
    last_status_ms = now_ms;

    if (log_due == 0U)
    {
        return;
    }

    last_log_ms = now_ms;

#if APP_LIDAR_VERBOSE_LOGS
    App_Lidar_FormatBytes(snapshot.sample,
                          snapshot.sample_count,
                          sample_text,
                          (uint32_t)sizeof(sample_text));

    if (snapshot.descriptor_pending != 0U)
    {
        App_Lidar_FormatBytes(snapshot.descriptor,
                              APP_LIDAR_DESCRIPTOR_SIZE,
                              descriptor_text,
                              (uint32_t)sizeof(descriptor_text));

        APP_LOG("APP LiDAR: descriptor=%s data_type=0x%02X len=%lu send_mode=0x%02X",
                descriptor_text,
                (unsigned int)snapshot.descriptor_data_type,
                (unsigned long)snapshot.descriptor_payload_len,
                (unsigned int)snapshot.descriptor_send_mode);

        if (snapshot.descriptor_data_type == 0x81U)
        {
            APP_LOG("APP LiDAR: scan response type 0x81 detected");
        }

        App_Lidar_MarkDescriptorPrinted();
    }

    if (snapshot.descriptor_seen != 0U)
    {
        (void)snprintf(type_text, sizeof(type_text), "0x%02X", (unsigned int)snapshot.descriptor_data_type);
    }
    else
    {
        (void)snprintf(type_text, sizeof(type_text), "--");
    }

    if (snapshot.has_valid_distance != 0U)
    {
        (void)snprintf(debug_min_text, sizeof(debug_min_text), "%lumm", (unsigned long)snapshot.min_distance_mm);
    }
    else
    {
        (void)snprintf(debug_min_text, sizeof(debug_min_text), "--");
    }

    if (snapshot.has_front_distance != 0U)
    {
        (void)snprintf(front_text, sizeof(front_text), "%lumm", (unsigned long)snapshot.front_min_mm);
    }
    else
    {
        (void)snprintf(front_text, sizeof(front_text), "--");
    }

    if (snapshot.has_front_wide_distance != 0U)
    {
        (void)snprintf(front_wide_text, sizeof(front_wide_text), "%lumm", (unsigned long)snapshot.front_wide_min_mm);
    }
    else
    {
        (void)snprintf(front_wide_text, sizeof(front_wide_text), "--");
    }

    if (snapshot.has_left_distance != 0U)
    {
        (void)snprintf(left_text, sizeof(left_text), "%lumm", (unsigned long)snapshot.left_min_mm);
    }
    else
    {
        (void)snprintf(left_text, sizeof(left_text), "--");
    }

    if (snapshot.has_right_distance != 0U)
    {
        (void)snprintf(right_text, sizeof(right_text), "%lumm", (unsigned long)snapshot.right_min_mm);
    }
    else
    {
        (void)snprintf(right_text, sizeof(right_text), "--");
    }

    if (snapshot.has_nearest != 0U)
    {
        (void)snprintf(nearest_text,
                       sizeof(nearest_text),
                       "%lumm@%lu.%01ludeg q=%u",
                       (unsigned long)snapshot.nearest_distance_mm,
                       (unsigned long)(snapshot.nearest_angle_tenth_deg / 10U),
                       (unsigned long)(snapshot.nearest_angle_tenth_deg % 10U),
                       (unsigned int)snapshot.nearest_quality);
    }
    else
    {
        (void)snprintf(nearest_text, sizeof(nearest_text), "--");
    }

    if (snapshot.has_angle != 0U)
    {
        (void)snprintf(angle_text,
                       sizeof(angle_text),
                       "%lu.%01lu",
                       (unsigned long)(snapshot.last_angle_tenth_deg / 10U),
                       (unsigned long)(snapshot.last_angle_tenth_deg % 10U));
    }
    else
    {
        (void)snprintf(angle_text, sizeof(angle_text), "--");
    }

    rx_age_ms = App_Lidar_ElapsedMs(now_ms, snapshot.last_rx_tick_ms);
    valid_age_ms = App_Lidar_ElapsedMs(now_ms, snapshot.last_valid_update_ms);

    APP_LOG("APP LiDAR: ready=%u rx=%lu rx_age=%lu valid=%lu valid_age=%lu uart4_error_count=%lu type=%s nearest=%s front=%s front_wide=%s left=%s right=%s scan_start=%lu debug_min=%s parsed=%lu invalid_header=%lu invalid_range=%lu zero_quality=%lu angle=%s sample=%s",
            (unsigned int)snapshot.ready,
            (unsigned long)snapshot.rx_bytes,
            (unsigned long)rx_age_ms,
            (unsigned long)snapshot.valid_points,
            (unsigned long)valid_age_ms,
            (unsigned long)snapshot.uart4_error_count,
            type_text,
            nearest_text,
            front_text,
            front_wide_text,
            left_text,
            right_text,
            (unsigned long)snapshot.scan_start_count,
            debug_min_text,
            (unsigned long)snapshot.parsed_points,
            (unsigned long)snapshot.invalid_header,
            (unsigned long)snapshot.invalid_range,
            (unsigned long)snapshot.zero_quality,
            angle_text,
            sample_text);
#else
    if (snapshot.descriptor_pending != 0U)
    {
        if (snapshot.descriptor_data_type == 0x81U)
        {
            APP_LOG("APP LiDAR: scan response type 0x81 detected");
        }

        App_Lidar_MarkDescriptorPrinted();
    }
#endif
}

void App_Lidar_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != UART4))
    {
        return;
    }

    if (app_lidar_callback_entered_logged == 0U)
    {
        app_lidar_callback_entered_logged = 1U;
        app_lidar_callback_entered_log_pending = 1U;
    }

    uint8_t byte = app_lidar_rx_byte;

    app_lidar_rx_bytes++;
    app_lidar_last_rx_tick = HAL_GetTick();
    if (app_lidar_rx_bytes == 1U)
    {
        app_lidar_first_rx_log_pending = 1U;
    }
    else if ((app_lidar_rx_bytes % APP_LIDAR_RX_DEBUG_INTERVAL_BYTES) == 0U)
    {
        app_lidar_rx_count_log_pending = app_lidar_rx_bytes;
    }

    app_lidar_sample[app_lidar_sample_write_index] = byte;
    app_lidar_sample_write_index = (uint8_t)((app_lidar_sample_write_index + 1U) % APP_LIDAR_SAMPLE_SIZE);

    if (app_lidar_sample_count < APP_LIDAR_SAMPLE_SIZE)
    {
        app_lidar_sample_count++;
    }

    App_Lidar_FeedParser(byte);
    (void)App_Lidar_StartRx();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return;
    }

    if (huart->Instance == UART4)
    {
        App_Lidar_RxCpltCallback(huart);
    }

    if (huart->Instance == USART2)
    {
        App_SerialCommand_RxCpltCallback(huart);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uint32_t error_code;
    uint32_t count;

    if ((huart == NULL) || (huart->Instance != UART4))
    {
        return;
    }

    error_code = huart->ErrorCode;
    count = app_lidar_uart4_error_count + 1U;
    app_lidar_uart4_error_count = count;
    app_lidar_uart4_error_code = error_code;

    if ((count == 1U) || ((count % 20U) == 0U))
    {
        app_lidar_uart4_error_log_pending = count;
        app_lidar_uart4_error_code_log_pending = error_code;
    }

    (void)App_Lidar_StartRx();
}

static HAL_StatusTypeDef App_Lidar_StartRx(void)
{
    return HAL_UART_Receive_IT(&huart4, &app_lidar_rx_byte, 1U);
}

static void App_Lidar_LogRxDebug(void)
{
    uint8_t first_log;
    uint8_t callback_entered_log;
    uint32_t count_log;
    uint32_t error_count_log;
    uint32_t error_code_log;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    first_log = app_lidar_first_rx_log_pending;
    app_lidar_first_rx_log_pending = 0U;
    callback_entered_log = app_lidar_callback_entered_log_pending;
    app_lidar_callback_entered_log_pending = 0U;
    count_log = app_lidar_rx_count_log_pending;
    app_lidar_rx_count_log_pending = 0U;
    error_count_log = app_lidar_uart4_error_log_pending;
    app_lidar_uart4_error_log_pending = 0U;
    error_code_log = app_lidar_uart4_error_code_log_pending;
    app_lidar_uart4_error_code_log_pending = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }

#if APP_LIDAR_VERBOSE_LOGS
    if (callback_entered_log != 0U)
    {
        APP_LOG("[LIDAR_RX] callback entered");
    }

    if (first_log != 0U)
    {
        APP_LOG("[LIDAR_RX] first byte");
    }

    if (count_log != 0U)
    {
        APP_LOG("[LIDAR_RX] rx_count=%lu", (unsigned long)count_log);
    }
#else
    (void)callback_entered_log;
    (void)first_log;
    (void)count_log;
#endif

    if (error_count_log != 0U)
    {
        APP_LOG("[LIDAR_RX] uart4_error count=%lu code=0x%08lX state=0x%02X",
                (unsigned long)error_count_log,
                (unsigned long)error_code_log,
                (unsigned int)huart4.RxState);
    }
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

static void App_Lidar_ResetParser(void)
{
    app_lidar_descriptor_state = APP_LIDAR_DESCRIPTOR_SEARCH_A5;
    app_lidar_descriptor_index = 0U;
    app_lidar_node_count = 0U;
    app_lidar_descriptor_seen = 0U;
    app_lidar_descriptor_pending = 0U;
    app_lidar_descriptor_data_type = 0U;
    app_lidar_descriptor_send_mode = 0U;
    app_lidar_descriptor_payload_len = 0U;
    app_lidar_parsed_points = 0U;
    app_lidar_valid_points = 0U;
    app_lidar_invalid_header = 0U;
    app_lidar_invalid_range = 0U;
    app_lidar_zero_quality = 0U;
    app_lidar_min_distance_mm = APP_LIDAR_MIN_DISTANCE_NONE;
    app_lidar_front_min_mm = APP_LIDAR_MIN_DISTANCE_NONE;
    app_lidar_front_wide_min_mm = APP_LIDAR_MIN_DISTANCE_NONE;
    app_lidar_left_min_mm = APP_LIDAR_MIN_DISTANCE_NONE;
    app_lidar_right_min_mm = APP_LIDAR_MIN_DISTANCE_NONE;
    app_lidar_nearest_distance_mm = APP_LIDAR_MIN_DISTANCE_NONE;
    app_lidar_nearest_angle_tenth_deg = 0U;
    app_lidar_last_angle_tenth_deg = 0U;
    app_lidar_scan_start_count = 0U;
    app_lidar_has_valid_distance = 0U;
    app_lidar_has_front_distance = 0U;
    app_lidar_has_front_wide_distance = 0U;
    app_lidar_has_left_distance = 0U;
    app_lidar_has_right_distance = 0U;
    app_lidar_has_nearest = 0U;
    app_lidar_nearest_quality = 0U;
    app_lidar_has_angle = 0U;

    for (uint8_t i = 0U; i < APP_LIDAR_DESCRIPTOR_SIZE; i++)
    {
        app_lidar_descriptor_buffer[i] = 0U;
        app_lidar_descriptor_latest[i] = 0U;
    }
}

static void App_Lidar_FeedParser(uint8_t byte)
{
    if (app_lidar_descriptor_seen == 0U)
    {
        (void)App_Lidar_FeedDescriptorParser(byte);
        return;
    }

    app_lidar_node_buffer[app_lidar_node_count] = byte;
    app_lidar_node_count++;

    if (app_lidar_node_count >= APP_LIDAR_SCAN_NODE_SIZE)
    {
        App_Lidar_ProcessScanNodeCandidate();
    }
}

static bool App_Lidar_FeedDescriptorParser(uint8_t byte)
{
    if (app_lidar_descriptor_state == APP_LIDAR_DESCRIPTOR_SEARCH_A5)
    {
        if (byte == 0xA5U)
        {
            app_lidar_descriptor_buffer[0] = byte;
            app_lidar_descriptor_index = 1U;
            app_lidar_descriptor_state = APP_LIDAR_DESCRIPTOR_SEARCH_5A;
            return true;
        }

        return false;
    }

    if (app_lidar_descriptor_state == APP_LIDAR_DESCRIPTOR_SEARCH_5A)
    {
        if (byte == 0x5AU)
        {
            app_lidar_descriptor_buffer[1] = byte;
            app_lidar_descriptor_index = 2U;
            app_lidar_descriptor_state = APP_LIDAR_DESCRIPTOR_COLLECT;
            return true;
        }

        if (byte == 0xA5U)
        {
            app_lidar_descriptor_buffer[0] = byte;
            app_lidar_descriptor_index = 1U;
            return true;
        }

        app_lidar_descriptor_state = APP_LIDAR_DESCRIPTOR_SEARCH_A5;
        app_lidar_descriptor_index = 0U;
        return false;
    }

    app_lidar_descriptor_buffer[app_lidar_descriptor_index] = byte;
    app_lidar_descriptor_index++;

    if (app_lidar_descriptor_index >= APP_LIDAR_DESCRIPTOR_SIZE)
    {
        App_Lidar_OnDescriptorComplete();
        app_lidar_descriptor_state = APP_LIDAR_DESCRIPTOR_SEARCH_A5;
        app_lidar_descriptor_index = 0U;
    }

    return true;
}

static void App_Lidar_OnDescriptorComplete(void)
{
    for (uint8_t i = 0U; i < APP_LIDAR_DESCRIPTOR_SIZE; i++)
    {
        app_lidar_descriptor_latest[i] = app_lidar_descriptor_buffer[i];
    }

    app_lidar_descriptor_data_type = app_lidar_descriptor_buffer[6];
    app_lidar_descriptor_payload_len =
        (uint32_t)app_lidar_descriptor_buffer[2] |
        ((uint32_t)app_lidar_descriptor_buffer[3] << 8) |
        ((uint32_t)app_lidar_descriptor_buffer[4] << 16) |
        (((uint32_t)app_lidar_descriptor_buffer[5] & 0x3FU) << 24);
    app_lidar_descriptor_send_mode = app_lidar_descriptor_buffer[5] & 0xC0U;
    app_lidar_descriptor_seen = 1U;
    app_lidar_descriptor_pending = 1U;
    app_lidar_node_count = 0U;
}

static void App_Lidar_ProcessScanNodeCandidate(void)
{
    App_Lidar_ScanNode_t node = App_Lidar_DecodeScanNode(app_lidar_node_buffer);

    app_lidar_parsed_points++;

    if (!node.header_valid)
    {
        app_lidar_invalid_header++;
        App_Lidar_ShiftNodeBuffer();
        return;
    }

    if (node.start_flag != 0U)
    {
        app_lidar_scan_start_count++;
    }

    if (node.quality == 0U)
    {
        app_lidar_zero_quality++;
        app_lidar_node_count = 0U;
        return;
    }

    if ((node.angle_q6 >= (360U * 64U)) ||
        (node.distance_q2 < (APP_LIDAR_MIN_DISTANCE_MM * 4U)) ||
        (node.distance_q2 > (APP_LIDAR_MAX_DISTANCE_MM * 4U)))
    {
        app_lidar_invalid_range++;
        app_lidar_node_count = 0U;
        return;
    }

    app_lidar_valid_points++;
    app_lidar_has_angle = 1U;
    app_lidar_last_angle_tenth_deg = node.angle_tenth_deg;
    app_lidar_has_valid_distance = 1U;
    app_lidar_last_valid_update_tick = HAL_GetTick();

    if (node.distance_mm < app_lidar_min_distance_mm)
    {
        app_lidar_min_distance_mm = node.distance_mm;
    }

    App_Lidar_UpdateZoneMins(&node);

    app_lidar_node_count = 0U;
}

static App_Lidar_ScanNode_t App_Lidar_DecodeScanNode(const uint8_t *node)
{
    App_Lidar_ScanNode_t decoded;
    uint8_t s = node[0] & 0x01U;
    uint8_t inv_s = (node[0] >> 1) & 0x01U;
    uint16_t angle_q6 = ((uint16_t)node[2] << 7) | ((uint16_t)node[1] >> 1);
    uint16_t distance_q2 = ((uint16_t)node[4] << 8) | (uint16_t)node[3];

    decoded.start_flag = s;
    decoded.quality = node[0] >> 2;
    decoded.header_valid = (((s ^ inv_s) == 1U) && ((node[1] & 0x01U) == 1U));
    decoded.angle_q6 = angle_q6;
    decoded.distance_q2 = distance_q2;
    decoded.angle_tenth_deg = (((uint32_t)angle_q6 * 10U) + 32U) / 64U;
    decoded.distance_mm = (uint32_t)distance_q2 / 4U;

    return decoded;
}

static void App_Lidar_UpdateZoneMins(const App_Lidar_ScanNode_t *node)
{
    if ((node == NULL) ||
        (node->quality < APP_LIDAR_ZONE_MIN_QUALITY) ||
        (node->angle_q6 >= (360U * 64U)) ||
        (node->distance_q2 < (APP_LIDAR_ZONE_MIN_DISTANCE_MM * 4U)) ||
        (node->distance_q2 > (APP_LIDAR_MAX_DISTANCE_MM * 4U)))
    {
        return;
    }

    uint32_t angle = node->angle_tenth_deg;
    uint32_t distance = node->distance_mm;

    if ((app_lidar_has_nearest == 0U) || (distance < app_lidar_nearest_distance_mm))
    {
        app_lidar_nearest_distance_mm = distance;
        app_lidar_nearest_angle_tenth_deg = angle;
        app_lidar_nearest_quality = node->quality;
        app_lidar_has_nearest = 1U;
    }

    if ((angle >= APP_LIDAR_FRONT_WIDE_MIN_TENTH_DEG) &&
        (angle <= APP_LIDAR_FRONT_WIDE_MAX_TENTH_DEG))
    {
        if ((app_lidar_has_front_wide_distance == 0U) || (distance < app_lidar_front_wide_min_mm))
        {
            app_lidar_front_wide_min_mm = distance;
            app_lidar_has_front_wide_distance = 1U;
        }
    }

    if ((angle >= APP_LIDAR_FRONT_MIN_TENTH_DEG) &&
        (angle <= APP_LIDAR_FRONT_MAX_TENTH_DEG))
    {
        if ((app_lidar_has_front_distance == 0U) || (distance < app_lidar_front_min_mm))
        {
            app_lidar_front_min_mm = distance;
            app_lidar_has_front_distance = 1U;
        }
    }

    if ((angle >= APP_LIDAR_LEFT_MIN_TENTH_DEG) &&
        (angle <= APP_LIDAR_LEFT_MAX_TENTH_DEG))
    {
        if ((app_lidar_has_left_distance == 0U) || (distance < app_lidar_left_min_mm))
        {
            app_lidar_left_min_mm = distance;
            app_lidar_has_left_distance = 1U;
        }
    }

    if ((angle >= APP_LIDAR_RIGHT_MIN_TENTH_DEG) &&
        (angle <= APP_LIDAR_RIGHT_MAX_TENTH_DEG))
    {
        if ((app_lidar_has_right_distance == 0U) || (distance < app_lidar_right_min_mm))
        {
            app_lidar_right_min_mm = distance;
            app_lidar_has_right_distance = 1U;
        }
    }
}

static void App_Lidar_ShiftNodeBuffer(void)
{
    for (uint8_t i = 0U; i < (APP_LIDAR_SCAN_NODE_SIZE - 1U); i++)
    {
        app_lidar_node_buffer[i] = app_lidar_node_buffer[i + 1U];
    }

    app_lidar_node_count = APP_LIDAR_SCAN_NODE_SIZE - 1U;
}

static void App_Lidar_CopySnapshot(App_Lidar_Snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    snapshot->rx_bytes = app_lidar_rx_bytes;
    snapshot->parsed_points = app_lidar_parsed_points;
    snapshot->valid_points = app_lidar_valid_points;
    snapshot->invalid_header = app_lidar_invalid_header;
    snapshot->invalid_range = app_lidar_invalid_range;
    snapshot->zero_quality = app_lidar_zero_quality;
    snapshot->min_distance_mm = app_lidar_min_distance_mm;
    snapshot->front_min_mm = app_lidar_front_min_mm;
    snapshot->front_wide_min_mm = app_lidar_front_wide_min_mm;
    snapshot->left_min_mm = app_lidar_left_min_mm;
    snapshot->right_min_mm = app_lidar_right_min_mm;
    snapshot->nearest_distance_mm = app_lidar_nearest_distance_mm;
    snapshot->nearest_angle_tenth_deg = app_lidar_nearest_angle_tenth_deg;
    snapshot->last_angle_tenth_deg = app_lidar_last_angle_tenth_deg;
    snapshot->last_rx_tick_ms = app_lidar_last_rx_tick;
    snapshot->last_valid_update_ms = app_lidar_last_valid_update_tick;
    snapshot->uart4_error_count = app_lidar_uart4_error_count;
    snapshot->uart4_error_code = app_lidar_uart4_error_code;
    snapshot->scan_start_count = app_lidar_scan_start_count;
    snapshot->descriptor_payload_len = app_lidar_descriptor_payload_len;
    snapshot->ready = app_lidar_descriptor_seen;
    snapshot->descriptor_seen = app_lidar_descriptor_seen;
    snapshot->descriptor_pending = app_lidar_descriptor_pending;
    snapshot->descriptor_data_type = app_lidar_descriptor_data_type;
    snapshot->descriptor_send_mode = app_lidar_descriptor_send_mode;
    snapshot->has_valid_distance = app_lidar_has_valid_distance;
    snapshot->has_front_distance = app_lidar_has_front_distance;
    snapshot->has_front_wide_distance = app_lidar_has_front_wide_distance;
    snapshot->has_left_distance = app_lidar_has_left_distance;
    snapshot->has_right_distance = app_lidar_has_right_distance;
    snapshot->has_nearest = app_lidar_has_nearest;
    snapshot->nearest_quality = app_lidar_nearest_quality;
    snapshot->has_angle = app_lidar_has_angle;

    /* Direction and nearest stats are per-log-window to avoid stale readings. */
    app_lidar_front_min_mm = APP_LIDAR_MIN_DISTANCE_NONE;
    app_lidar_front_wide_min_mm = APP_LIDAR_MIN_DISTANCE_NONE;
    app_lidar_left_min_mm = APP_LIDAR_MIN_DISTANCE_NONE;
    app_lidar_right_min_mm = APP_LIDAR_MIN_DISTANCE_NONE;
    app_lidar_nearest_distance_mm = APP_LIDAR_MIN_DISTANCE_NONE;
    app_lidar_nearest_angle_tenth_deg = 0U;
    app_lidar_has_front_distance = 0U;
    app_lidar_has_front_wide_distance = 0U;
    app_lidar_has_left_distance = 0U;
    app_lidar_has_right_distance = 0U;
    app_lidar_has_nearest = 0U;
    app_lidar_nearest_quality = 0U;

    for (uint8_t i = 0U; i < APP_LIDAR_DESCRIPTOR_SIZE; i++)
    {
        snapshot->descriptor[i] = app_lidar_descriptor_latest[i];
    }

    uint8_t count = app_lidar_sample_count;
    uint8_t start = (uint8_t)((app_lidar_sample_write_index + APP_LIDAR_SAMPLE_SIZE - count) % APP_LIDAR_SAMPLE_SIZE);

    snapshot->sample_count = count;
    for (uint8_t i = 0U; i < count; i++)
    {
        snapshot->sample[i] = app_lidar_sample[(start + i) % APP_LIDAR_SAMPLE_SIZE];
    }

    if (primask == 0U)
    {
        __enable_irq();
    }
}

static void App_Lidar_UpdatePublishedStatus(const App_Lidar_Snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    app_lidar_status.ready = snapshot->ready;
    app_lidar_status.rx_bytes = snapshot->rx_bytes;
    app_lidar_status.valid_points = snapshot->valid_points;
    app_lidar_status.last_rx_tick_ms = snapshot->last_rx_tick_ms;
    app_lidar_status.last_valid_update_ms = snapshot->last_valid_update_ms;
    app_lidar_status.last_update_ms = snapshot->last_valid_update_ms;
    app_lidar_status.uart4_error_count = snapshot->uart4_error_count;
    app_lidar_status.uart4_error_code = snapshot->uart4_error_code;
    app_lidar_status.front_valid = snapshot->has_front_distance;
    app_lidar_status.front_wide_valid = snapshot->has_front_wide_distance;
    app_lidar_status.left_valid = snapshot->has_left_distance;
    app_lidar_status.right_valid = snapshot->has_right_distance;
    app_lidar_status.front_min_mm = (snapshot->has_front_distance != 0U)
        ? (uint16_t)snapshot->front_min_mm
        : (uint16_t)APP_LIDAR_STATUS_DISTANCE_INVALID;
    app_lidar_status.front_wide_min_mm = (snapshot->has_front_wide_distance != 0U)
        ? (uint16_t)snapshot->front_wide_min_mm
        : (uint16_t)APP_LIDAR_STATUS_DISTANCE_INVALID;
    app_lidar_status.left_min_mm = (snapshot->has_left_distance != 0U)
        ? (uint16_t)snapshot->left_min_mm
        : (uint16_t)APP_LIDAR_STATUS_DISTANCE_INVALID;
    app_lidar_status.right_min_mm = (snapshot->has_right_distance != 0U)
        ? (uint16_t)snapshot->right_min_mm
        : (uint16_t)APP_LIDAR_STATUS_DISTANCE_INVALID;

    if (snapshot->has_nearest != 0U)
    {
        app_lidar_status.nearest_distance_mm = (uint16_t)snapshot->nearest_distance_mm;
        app_lidar_status.nearest_angle_deg = (float)snapshot->nearest_angle_tenth_deg / 10.0f;
    }
    else
    {
        app_lidar_status.nearest_distance_mm = (uint16_t)APP_LIDAR_STATUS_DISTANCE_INVALID;
        app_lidar_status.nearest_angle_deg = 0.0f;
    }

    if (primask == 0U)
    {
        __enable_irq();
    }
}

static void App_Lidar_MarkDescriptorPrinted(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    app_lidar_descriptor_pending = 0U;

    if (primask == 0U)
    {
        __enable_irq();
    }
}

#if APP_LIDAR_VERBOSE_LOGS
static void App_Lidar_FormatBytes(const uint8_t *bytes, uint8_t count, char *text, uint32_t text_size)
{
    static const char hex[] = "0123456789ABCDEF";
    uint32_t pos = 0U;

    if ((text == NULL) || (text_size == 0U))
    {
        return;
    }

    if ((bytes == NULL) || (count == 0U))
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

    for (uint8_t i = 0U; i < count; i++)
    {
        if ((i > 0U) && ((pos + 1U) < text_size))
        {
            text[pos++] = ' ';
        }

        if ((pos + 2U) >= text_size)
        {
            break;
        }

        text[pos++] = hex[(bytes[i] >> 4) & 0x0FU];
        text[pos++] = hex[bytes[i] & 0x0FU];
    }

    text[pos] = '\0';
}

static uint32_t App_Lidar_ElapsedMs(uint32_t now_ms, uint32_t then_ms)
{
    if (then_ms == 0U)
    {
        return UINT32_MAX;
    }

    if (now_ms >= then_ms)
    {
        return now_ms - then_ms;
    }

    return 0U;
}
#endif
