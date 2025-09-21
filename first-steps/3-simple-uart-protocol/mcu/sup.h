/**
 * @file sup.h
 * @brief Simple UART Protocol (SUP) - Public API and Protocol Definitions
 *
 * @details
 * This header file defines the constants, data structures, and function
 * prototypes for the Simple UART Protocol (SUP). SUP is a lightweight,
 * frame-based protocol designed for reliable communication between a host
 * computer and an embedded microcontroller over a UART interface. It is
 * suitable for tasks like firmware updates, data logging, and command/control.
 *
 * @note This implementation is for educational purposes and not production-ready.
 * It lacks features like byte stuffing, which would be necessary to handle
 * SOF/EOF markers appearing in the payload data.
 *
 * @warning This code is provided for educational purposes only and is not
 * intended for production use. Use at your own risk. No warranty is provided.
 *
 * @author m3y54m
 * @version 1.0.0
 * @date 2025
 * @license MIT License
 */

#ifndef SUP_H
#define SUP_H

#include <stdbool.h>
#include <stdint.h>

// ============================================================================
// Protocol Constants
// ============================================================================

#define SUP_SOF 0xA1 ///< SUP Protocol Start-Of-Frame byte
#define SUP_EOF 0xE9 ///< SUP Protocol End-Of-Frame byte

/**
 * @brief Maximum payload size in bytes.
 * @details This defines the size of the payload buffer and limits the amount of
 * data that can be sent in a single frame. This value can be adjusted based
 * on the available SRAM on the target microcontroller.
 */
#define SUP_MAX_PAYLOAD_SIZE 128

/*
 * ============================================================================
 * Protocol Frame Format
 * ============================================================================
 *
 * +-----+----+------+--------------------+----------+-----+
 * | SOF | ID | SIZE |      PAYLOAD       | CHECKSUM | EOF |
 * +-----+----+------+--------------------+----------+-----+
 * | 1B  | 1B |  1B  |   0 to SIZE bytes  |    1B    | 1B  |
 * +-----+----+------+--------------------+----------+-----+
 *
 * Frame Fields:
 * - SOF: Start-Of-Frame marker (SUP_SOF).
 * - ID: Frame identifier, indicating the frame's purpose (see sup_id_t).
 * - SIZE: The number of bytes in the PAYLOAD field (0 to SUP_MAX_PAYLOAD_SIZE).
 * - PAYLOAD: The variable-length data field.
 * - CHECKSUM: An 8-bit checksum calculated over the ID, SIZE, and PAYLOAD fields.
 * It is a simple arithmetic sum of the bytes, modulo 256.
 * - EOF: End-Of-Frame marker (SUP_EOF).
 */

// ============================================================================
// Frame IDs and Payload Structures
// ============================================================================

/**
 * @enum sup_id_t
 * @brief Defines the set of valid identifiers for SUP frames.
 */
typedef enum
{
    //---- Basic Protocol Frames ----
    /**
     * @brief Acknowledgment (ACK). Sent to confirm successful receipt of a frame.
     * @details Payload: [ack'd_id (1B)] + [optional_info (1B)]
     */
    SUP_ID_ACK = 0x01,

    /**
     * @brief Negative Acknowledgment (NACK). Sent to report an error in a received frame.
     * @details Payload: [nack'd_id (1B)] + [optional_error_code (1B)]
     */
    SUP_ID_NACK = 0x02,

    /**
     * @brief General-purpose data frame.
     * @details Used to transfer chunks of binary data, e.g., during a firmware update.
     */
    SUP_ID_DATA = 0x03,

    //---- Command Frames ----
    /**
     * @brief Command to initiate a firmware update.
     *
     * @details This command begins the firmware update sequence.
     * Transfer sequence (Host -> MCU):
     * 1) Host -> MCU : Send CMD_FW_UPDATE frame.
     * 2) MCU  -> Host : Respond with ACK for CMD_FW_UPDATE.
     * 3) Host -> MCU : Send DATA frame with total firmware size (2 bytes, little-endian).
     * 4) MCU  -> Host : Respond with ACK for the size frame.
     * 5) Host -> MCU : Send firmware in chunks using DATA frames.
     * 6) MCU  -> Host : ACK each DATA chunk.
     * 7) Repeat 5-6 until the entire firmware is transferred.
     */
    SUP_ID_CMD_FW_UPDATE = 0x11,

} sup_id_t;

// ============================================================================
// Type Definitions
// ============================================================================

/**
 * @enum sup_frame_parsing_state_t
 * @brief States for the SUP frame reception state machine.
 */
typedef enum
{
    SUP_STATE_WAIT_SOF = 0,      ///< Waiting for the Start-Of-Frame byte (SUP_SOF).
    SUP_STATE_WAIT_ID,           ///< Waiting for the frame ID byte.
    SUP_STATE_WAIT_PAYLOAD_SIZE, ///< Waiting for the payload size byte.
    SUP_STATE_GET_PAYLOAD,       ///< Actively receiving payload bytes.
    SUP_STATE_WAIT_CHECKSUM,     ///< Waiting for the checksum byte.
    SUP_STATE_WAIT_EOF           ///< Waiting for the End-Of-Frame byte (SUP_EOF).
} sup_frame_parsing_state_t;

/**
 * @enum sup_frame_parsing_result_t
 * @brief Possible outcomes of the frame parsing process.
 */
typedef enum
{
    SUP_RESULT_PENDING = 0,    ///< Frame reception is in progress.
    SUP_RESULT_SUCCESS,        ///< A valid frame has been successfully received.
    SUP_RESULT_ERROR_SIZE,     ///< Error: Payload size exceeds SUP_MAX_PAYLOAD_SIZE.
    SUP_RESULT_ERROR_CHECKSUM, ///< Error: Calculated checksum does not match received checksum.
    SUP_RESULT_ERROR_EOF,      ///< Error: Expected EOF marker was not received.
} sup_frame_parsing_result_t;

/**
 * @struct sup_frame_t
 * @brief Represents the data content of a SUP frame.
 */
typedef struct
{
    uint8_t id;                            ///< Frame identifier (from sup_id_t).
    uint8_t payload_size;                  ///< Number of valid bytes in the payload buffer.
    uint8_t payload[SUP_MAX_PAYLOAD_SIZE]; ///< Buffer to store payload data.
} sup_frame_t;

/**
 * @struct sup_rx_frame_state_t
 * @brief Holds the complete state for the SUP reception state machine.
 * @details An instance of this struct must be provided by the user application
 * to the `sup_init` function.
 */
typedef struct
{
    sup_frame_parsing_state_t  parsing_state;  ///< Current state of the parser.
    sup_frame_parsing_result_t parsing_result; ///< Result of the last parsing attempt.
    sup_frame_t                frame;          ///< Buffer for the incoming frame data.
    uint8_t                    payload_index;  ///< Tracks the number of payload bytes received.
    uint8_t                    checksum;       ///< The calculated checksum of the incoming frame.
} sup_rx_frame_state_t;

// ============================================================================
// Public API Functions
// ============================================================================

/**
 * @brief Initializes the SUP protocol state machine.
 * @param user_rx_frame_state Pointer to a state structure allocated by the user's application.
 * This structure will be managed by the SUP library.
 */
void sup_init(sup_rx_frame_state_t* user_rx_frame_state);

/**
 * @brief Gets a pointer to the internal SUP frame reception state.
 * @return Pointer to the reception state, or NULL if not initialized.
 */
sup_rx_frame_state_t* sup_get_rx_state();

/**
 * @brief Processes one incoming byte through the SUP state machine.
 * @details This function should be called for every byte received from the UART.
 * @param rx_byte The byte received from the communication peripheral.
 */
void sup_handle_rx_byte(const uint8_t rx_byte);

/**
 * @brief Constructs and sends a complete SUP frame.
 * @param id The frame identifier (from sup_id_t).
 * @param payload Pointer to the payload data buffer. Can be NULL if payload_size is 0.
 * @param payload_size The number of bytes in the payload.
 */
void sup_send_frame(const uint8_t id, const uint8_t* payload, const uint8_t payload_size);

/**
 * @brief Sends a single byte over the hardware interface (e.g., UART).
 * @details **This function must be implemented by the user.** The SUP library
 * calls this function to transmit each byte of a frame.
 * @param tx_byte The byte to send.
 */
void sup_send_byte(const uint8_t tx_byte);

/**
 * @brief A helper function to send an acknowledgment (ACK) frame.
 * @param ack_for_id The ID of the frame being acknowledged.
 * @param optional_byte A pointer to an optional second payload byte for additional status info.
 * Pass NULL if not needed.
 */
void sup_send_ack(const uint8_t ack_for_id, const uint8_t* optional_byte);

/**
 * @brief A helper function to send a negative acknowledgment (NACK) frame.
 * @param nack_for_id The ID of the frame being negatively acknowledged.
 * @param optional_byte A pointer to an optional second payload byte for an error code.
 * Pass NULL if not needed.
 */
void sup_send_nack(const uint8_t nack_for_id, const uint8_t* optional_byte);

/**
 * @brief A helper function to send a data (DATA) frame.
 * @param payload Pointer to the data payload.
 * @param payload_size The size of the data payload in bytes.
 */
void sup_send_data(const uint8_t* payload, const uint8_t payload_size);

#endif // SUP_H
