#ifndef BOOT_SYNC_H
#define BOOT_SYNC_H

#include <stdint.h>

/**
 * @brief A magic value to indicate a firmware update is requested.
 */
#define FW_UPDATE_INDICATOR_MAGIC 0xDEADBEEFU

extern volatile uint32_t boot_sync_flag;

#endif // BOOT_SYNC_H