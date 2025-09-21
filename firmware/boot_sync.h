/**
 * @file boot_sync.h
 * @brief Boot Synchronization Interface Between User Application and Bootloader
 *
 * @details
 * This header defines the shared memory interface used for synchronization
 * between the user application and bootloader. It provides a mechanism for
 * the user application to request firmware updates by setting a magic value
 * in a designated memory location that survives software resets.
 *
 * Synchronization Mechanism:
 * - Uses a 32-bit flag variable in a fixed SRAM location
 * - Variable is placed in .bootflag section (uninitialized data)
 * - Survives software resets but not power cycles
 * - Magic value indicates firmware update request
 * - Cleared by bootloader after processing
 *
 * Memory Layout:
 * - Location: 0x08F8 (SRAM)
 * - Size: 4 bytes
 * - Access: Volatile (can be modified by ISR or different execution contexts)
 *
 * Usage Flow:
 * 1. User app sets boot_sync_flag to BOOT_FW_UPDATE_REQUEST
 * 2. User app triggers software reset (e.g., via watchdog)
 * 3. Bootloader checks boot_sync_flag on startup
 * 4. If magic value present, bootloader enters update mode
 * 5. Bootloader clears flag after reading
 * 6. If no magic value, bootloader jumps to user app
 *
 * @note This mechanism only works for software-triggered resets
 * @note Power-on reset will clear the SRAM, losing the magic value
 * @note Both bootloader and application must use the same memory location
 *
 * @warning Incorrect configuration may prevent proper bootloader operation
 * @warning Ensure linker scripts for both app and bootloader reserve this location
 *
 * @author m3y54m
 * @version 1.0.0
 * @date 2025
 * @license MIT License
 */

#ifndef BOOT_SYNC_H
#define BOOT_SYNC_H

// ============================================================================
// Includes
// ============================================================================

#include <stdbool.h>
#include <stdint.h>

// ============================================================================
// Configuration Constants
// ============================================================================

/**
 * @brief Magic value indicating firmware update request
 *
 * @details
 * This 32-bit magic value is written to boot_sync_flag by the user
 * application when it wants to trigger a firmware update. The value
 * is chosen to be highly unlikely to occur randomly in memory.
 *
 * The specific value 0xDEADBEEF is a well-known magic number in
 * embedded systems, making it easy to identify in memory dumps
 * and debugging sessions.
 *
 * @note Must be identical in both bootloader and user application
 */
#define BOOT_FW_UPDATE_REQUEST (0xDEADBEEFUL)

/**
 * @brief Clear value indicating no special boot request
 *
 * @details
 * This value is written to boot_sync_flag after the bootloader
 * has processed the request, ensuring the special boot mode is
 * not re-entered on subsequent resets.
 */
#define BOOT_MAGIC_CLEAR (0x00000000UL)

// ============================================================================
// Global Variable Declaration
// ============================================================================

/**
 * @brief Boot synchronization flag shared between bootloader and application
 *
 * @details
 * This variable is placed in a special section of SRAM that:
 * - Is not initialized by the C runtime startup code
 * - Maintains its value across software resets
 * - Is at a known, fixed memory address (0x08F8)
 *
 * The variable is declared as volatile because:
 * - It can be modified by different execution contexts (app vs bootloader)
 * - It may be checked in loops where optimization could cause issues
 * - It ensures memory barriers for proper synchronization
 *
 * Memory attributes:
 * - section(".bootflag"): Places variable in custom linker section
 * - used: Prevents linker from removing as unused
 * - volatile: Prevents compiler optimizations, ensures memory access
 *
 * @note Both bootloader and application must link this at the same address
 * @note Linker script must define .bootflag section at fixed location
 *
 * @warning Do not access this variable directly; use provided functions
 */
volatile uint32_t boot_sync_flag __attribute__((section(".bootflag"), used));

// ============================================================================
// Function Declarations (Optional - for enhanced functionality)
// ============================================================================

/**
 * @brief Check if firmware update has been requested
 *
 * @details
 * This inline function checks if the boot synchronization flag contains
 * the magic value indicating a firmware update request. It should be
 * called early in the bootloader's initialization sequence.
 *
 * @return true if firmware update requested, false otherwise
 *
 * @note This is typically called by the bootloader, not the user app
 */
static inline bool is_firmware_update_requested(void)
{
    if (boot_sync_flag == BOOT_FW_UPDATE_REQUEST)
    {
        // Clear flag immediately to prevent update loops if interrupted
        boot_sync_flag = BOOT_MAGIC_CLEAR;
        return true;
    }
    return false;
}

/**
 * @brief Request firmware update on next boot
 *
 * @details
 * This inline function sets the boot synchronization flag to request
 * a firmware update. After calling this function, the application
 * should trigger a software reset to enter bootloader mode.
 *
 * @note This is typically called by the user application
 * @note Must be followed by a software reset to take effect
 */
static inline void set_firmware_update_request_flag(void)
{
    boot_sync_flag = BOOT_FW_UPDATE_REQUEST;
}

// ============================================================================
// Safety Checks
// ============================================================================

/* Compile-time assertion to ensure magic values are distinct */
#if (BOOT_FW_UPDATE_REQUEST == BOOT_MAGIC_CLEAR)
#error "Magic values must be unique"
#endif

#endif /* BOOT_SYNC_H */

// ============================================================================
// End of File
// ============================================================================