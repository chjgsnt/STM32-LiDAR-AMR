#include "app_lidar.h"

#include "bringup_log.h"
#include "cmsis_os.h"
#include "usart.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define APP_LIDAR_BAUD_RATE 460800U
#define APP_LIDAR_LOG_INTERVAL_MS 1000U
#define APP_LIDAR_SAMPLE_SIZE 16U
#define APP_LIDAR_DESCRIPTOR_SIZE 7U
#define APP_LIDAR_SCAN_NODE_SIZE 5U
#define APP_LIDAR_SAMPLE_TEXT_SIZE ((APP_LIDAR_SAMPLE_SIZE * 3U) + 1U)
#define APP_LIDAR_DESCRIPTOR_TEXT_SIZE ((APP_LIDAR_DESCRIPTOR_SIZE * 3U) + 1U)
#define APP_LIDAR_CMD_DELAY_MS 100U
#define APP_LIDAR_CMD_TIMEOUT_MS 100U
#define APP_LIDAR_MIN_DISTANCE_MM 50U
#define APP_LIDAR_MAX_DISTANCE_MM 12000U
#define APP_LIDAR_MIN_DISTANCE_NONE 0xFFFFFFFFUL

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
    uint32_t last_angle_tenth_deg;
    uint32_t scan_start_count;
    uint32_t descriptor_payload_len;
    uint8_t descriptor_seen;
    uint8_t descriptor_pending;
    uint8_t descriptor[APP_LIDAR_DESCRIPTOR_SIZE];
    uint8_t descriptor_data_type;
    uint8_t descriptor_send_mode;
    uint8_t has_valid_distance;
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
static void App_Lidar_SendStartupCommands(void);
static HAL_StatusTypeDef App_Lidar_SendCommand(const char *name, const uint8_t *command, uint16_t length);
static void App_Lidar_DelayMs(uint32_t delay_ms);
static void App_Lidar_ResetParser(void);
static void App_Lidar_FeedParser(uint8_t byte);
static bool App_Lidar_FeedDescriptorParser(uint8_t byte);
static void App_Lidar_OnDescriptorComplete(void);
static void App_Lidar_ProcessScanNodeCandidate(void);
static App_Lidar_ScanNode_t App_Lidar_DecodeScanNode(const uint8_t *node);
static void App_Lidar_ShiftNodeBuffer(void);
static void App_Lidar_CopySnapshot(App_Lidar_Snapshot_t *snapshot);
static void App_Lidar_MarkDescriptorPrinted(void);
static void App_Lidar_FormatBytes(const uint8_t *bytes, uint8_t count, char *text, uint32_t text_size);

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
static volatile uint32_t app_lidar_last_angle_tenth_deg;
static volatile uint32_t app_lidar_scan_start_count;
static volatile uint8_t app_lidar_has_valid_distance;
static volatile uint8_t app_lidar_has_angle;

static bool app_lidar_initialized;

void App_Lidar_Init(void)
{
    app_lidar_rx_bytes = 0U;
    app_lidar_sample_write_index = 0U;
    app_lidar_sample_count = 0U;
    App_Lidar_ResetParser();
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

    App_Lidar_Snapshot_t snapshot;
    char sample_text[APP_LIDAR_SAMPLE_TEXT_SIZE];
    char descriptor_text[APP_LIDAR_DESCRIPTOR_TEXT_SIZE];
    char type_text[8];
    char min_text[16];
    char angle_text[16];

    App_Lidar_CopySnapshot(&snapshot);
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
        (void)snprintf(min_text, sizeof(min_text), "%lumm", (unsigned long)snapshot.min_distance_mm);
    }
    else
    {
        (void)snprintf(min_text, sizeof(min_text), "--");
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

    APP_LOG("APP LiDAR: rx_bytes=%lu type=%s parsed=%lu valid=%lu invalid_header=%lu invalid_range=%lu zero_quality=%lu min=%s angle=%s scan_start=%lu sample=%s",
            (unsigned long)snapshot.rx_bytes,
            type_text,
            (unsigned long)snapshot.parsed_points,
            (unsigned long)snapshot.valid_points,
            (unsigned long)snapshot.invalid_header,
            (unsigned long)snapshot.invalid_range,
            (unsigned long)snapshot.zero_quality,
            min_text,
            angle_text,
            (unsigned long)snapshot.scan_start_count,
            sample_text);
}

void App_Lidar_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != UART4))
    {
        return;
    }

    uint8_t byte = app_lidar_rx_byte;

    app_lidar_rx_bytes++;
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
    app_lidar_last_angle_tenth_deg = 0U;
    app_lidar_scan_start_count = 0U;
    app_lidar_has_valid_distance = 0U;
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

    if (node.distance_mm < app_lidar_min_distance_mm)
    {
        app_lidar_min_distance_mm = node.distance_mm;
    }

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
    snapshot->last_angle_tenth_deg = app_lidar_last_angle_tenth_deg;
    snapshot->scan_start_count = app_lidar_scan_start_count;
    snapshot->descriptor_payload_len = app_lidar_descriptor_payload_len;
    snapshot->descriptor_seen = app_lidar_descriptor_seen;
    snapshot->descriptor_pending = app_lidar_descriptor_pending;
    snapshot->descriptor_data_type = app_lidar_descriptor_data_type;
    snapshot->descriptor_send_mode = app_lidar_descriptor_send_mode;
    snapshot->has_valid_distance = app_lidar_has_valid_distance;
    snapshot->has_angle = app_lidar_has_angle;

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
