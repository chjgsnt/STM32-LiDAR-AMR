#include "ssd1306.h"

#include <stddef.h>

#define SSD1306_WIDTH 128U
#define SSD1306_HEIGHT 64U
#define SSD1306_PAGE_COUNT (SSD1306_HEIGHT / 8U)
#define SSD1306_BUFFER_SIZE (SSD1306_WIDTH * SSD1306_PAGE_COUNT)
#define SSD1306_COMMAND_CONTROL 0x00U
#define SSD1306_DATA_CONTROL 0x40U
#define SSD1306_I2C_TIMEOUT_MS 100U

static I2C_HandleTypeDef *ssd1306_i2c;
static uint8_t ssd1306_addr_7bit;
static uint8_t ssd1306_buffer[SSD1306_BUFFER_SIZE];

static bool SSD1306_WriteControl(uint8_t control, const uint8_t *data, uint16_t size)
{
    if ((ssd1306_i2c == NULL) || (data == NULL) || (size == 0U))
    {
        return false;
    }

    return HAL_I2C_Mem_Write(ssd1306_i2c,
                             (uint16_t)(ssd1306_addr_7bit << 1),
                             control,
                             I2C_MEMADD_SIZE_8BIT,
                             (uint8_t *)data,
                             size,
                             SSD1306_I2C_TIMEOUT_MS) == HAL_OK;
}

static bool SSD1306_WriteCommand(uint8_t command)
{
    return SSD1306_WriteControl(SSD1306_COMMAND_CONTROL, &command, 1U);
}

static bool SSD1306_WriteCommandList(const uint8_t *commands, uint16_t size)
{
    for (uint16_t i = 0U; i < size; i++)
    {
        if (SSD1306_WriteCommand(commands[i]) == false)
        {
            return false;
        }
    }

    return true;
}

static void SSD1306_GetGlyph(char c, uint8_t glyph[5])
{
    static const uint8_t blank[5] = {0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
    const uint8_t *src = blank;

    switch (c)
    {
    case '0':
    {
        static const uint8_t g[5] = {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU};
        src = g;
        break;
    }
    case '2':
    {
        static const uint8_t g[5] = {0x62U, 0x51U, 0x49U, 0x49U, 0x46U};
        src = g;
        break;
    }
    case '3':
    {
        static const uint8_t g[5] = {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U};
        src = g;
        break;
    }
    case ':':
    {
        static const uint8_t g[5] = {0x00U, 0x36U, 0x36U, 0x00U, 0x00U};
        src = g;
        break;
    }
    case 'C':
    {
        static const uint8_t g[5] = {0x3EU, 0x41U, 0x41U, 0x41U, 0x22U};
        src = g;
        break;
    }
    case 'D':
    {
        static const uint8_t g[5] = {0x7FU, 0x41U, 0x41U, 0x22U, 0x1CU};
        src = g;
        break;
    }
    case 'E':
    {
        static const uint8_t g[5] = {0x7FU, 0x49U, 0x49U, 0x49U, 0x41U};
        src = g;
        break;
    }
    case 'I':
    {
        static const uint8_t g[5] = {0x00U, 0x41U, 0x7FU, 0x41U, 0x00U};
        src = g;
        break;
    }
    case 'K':
    {
        static const uint8_t g[5] = {0x7FU, 0x08U, 0x14U, 0x22U, 0x41U};
        src = g;
        break;
    }
    case 'L':
    {
        static const uint8_t g[5] = {0x7FU, 0x40U, 0x40U, 0x40U, 0x40U};
        src = g;
        break;
    }
    case 'O':
    {
        static const uint8_t g[5] = {0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU};
        src = g;
        break;
    }
    case 'x':
    {
        static const uint8_t g[5] = {0x44U, 0x28U, 0x10U, 0x28U, 0x44U};
        src = g;
        break;
    }
    default:
        break;
    }

    for (uint8_t i = 0U; i < 5U; i++)
    {
        glyph[i] = src[i];
    }
}

bool SSD1306_Init(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit)
{
    static const uint8_t init_commands[] = {
        0xAEU,       /* Display off */
        0x20U, 0x00U, /* Horizontal addressing mode */
        0xB0U,
        0xC8U,
        0x00U,
        0x10U,
        0x40U,
        0x81U, 0x7FU,
        0xA1U,
        0xA6U,
        0xA8U, 0x3FU,
        0xA4U,
        0xD3U, 0x00U,
        0xD5U, 0x80U,
        0xD9U, 0xF1U,
        0xDAU, 0x12U,
        0xDBU, 0x40U,
        0x8DU, 0x14U,
        0xAFU        /* Display on */
    };

    if (hi2c == NULL)
    {
        return false;
    }

    ssd1306_i2c = hi2c;
    ssd1306_addr_7bit = addr_7bit;

    if (SSD1306_WriteCommandList(init_commands, (uint16_t)sizeof(init_commands)) == false)
    {
        return false;
    }

    return SSD1306_Clear();
}

bool SSD1306_Clear(void)
{
    for (uint16_t i = 0U; i < SSD1306_BUFFER_SIZE; i++)
    {
        ssd1306_buffer[i] = 0x00U;
    }

    return true;
}

bool SSD1306_WriteString(uint8_t x, uint8_t page, const char *text)
{
    uint8_t cursor_x = x;

    if ((text == NULL) || (page >= SSD1306_PAGE_COUNT))
    {
        return false;
    }

    while (*text != '\0')
    {
        uint8_t glyph[5];

        if ((uint16_t)cursor_x + 5U >= SSD1306_WIDTH)
        {
            break;
        }

        SSD1306_GetGlyph(*text, glyph);

        for (uint8_t col = 0U; col < 5U; col++)
        {
            ssd1306_buffer[(page * SSD1306_WIDTH) + cursor_x] = glyph[col];
            cursor_x++;
        }

        if (cursor_x < SSD1306_WIDTH)
        {
            ssd1306_buffer[(page * SSD1306_WIDTH) + cursor_x] = 0x00U;
            cursor_x++;
        }

        text++;
    }

    return true;
}

bool SSD1306_UpdateScreen(void)
{
    const uint16_t chunk_size = 16U;

    if (SSD1306_WriteCommand(0x21U) == false)
    {
        return false;
    }

    if ((SSD1306_WriteCommand(0x00U) == false) || (SSD1306_WriteCommand(0x7FU) == false))
    {
        return false;
    }

    if (SSD1306_WriteCommand(0x22U) == false)
    {
        return false;
    }

    if ((SSD1306_WriteCommand(0x00U) == false) || (SSD1306_WriteCommand(0x07U) == false))
    {
        return false;
    }

    for (uint16_t offset = 0U; offset < SSD1306_BUFFER_SIZE; offset += chunk_size)
    {
        uint16_t remaining = SSD1306_BUFFER_SIZE - offset;
        uint16_t size = (remaining > chunk_size) ? chunk_size : remaining;

        if (SSD1306_WriteControl(SSD1306_DATA_CONTROL, &ssd1306_buffer[offset], size) == false)
        {
            return false;
        }
    }

    return true;
}
