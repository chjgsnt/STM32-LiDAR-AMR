#include "lidar_parser.h"

void LidarParser_Reset(void)
{
    /* TODO: Reset parser state machine when packet parsing is implemented. */
}

bool LidarParser_ParseByte(uint8_t byte)
{
    (void)byte;
    /* TODO: Parse RPLidar C1 frames and report complete samples. */
    return false;
}
