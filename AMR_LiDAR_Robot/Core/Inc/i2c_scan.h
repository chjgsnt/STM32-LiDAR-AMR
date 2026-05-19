#ifndef I2C_SCAN_H
#define I2C_SCAN_H

/*
 * Minimal I2C bus scan module.
 *
 * This module is used during bring-up to verify that I2C devices respond on
 * the shared bus. It is intentionally simple and does not implement any full
 * device driver.
 */

void I2C_ScanBus(void);
void I2C_ReadMpu6500WhoAmI(void);

#endif /* I2C_SCAN_H */
