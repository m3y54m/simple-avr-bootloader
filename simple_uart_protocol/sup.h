
/**
 * @file sup.h
 * @brief Simple UART Protocol (SUP) - Protocol definitions and API
 *
 * @warning This code is provided for educational purposes only and is not
 * intended for production use. Use at your own risk. No warranty is provided.
 *
 */

#ifndef SUP_H
#define SUP_H

#include <stdbool.h>
#include <stdint.h>

/// SUP Protocol start-of-frame byte
#define SUP_SOF 0xA1
/// SUP Protocol end-of-frame byte
#define SUP_EOF 0xE9
/// Maximum payload size (reasonable limit for embedded systems)
#define SUP_MAX_PAYLOAD_SIZE 128

/*
 * SUP protocol frame format (ASCII diagram)
 *
 *  +-----+----+------+-------------+----------+-----+
 *  | SOF | ID | SIZE |   PAYLOAD   | CHECKSUM | EOF |
 *  +-----+----+------+-------------+----------+-----+
 *  | 1B  | 1B |  1B  | 0..N bytes  |    1B    | 1B  |
 *  +-----+----+------+-------------+----------+-----+
 *
 * Bytes order when transmitted: [SOF][ID][SIZE][PAYLOAD...][CHECKSUM][EOF]
 *
 * Notes:
 * - SOF: Start-Of-Frame marker
 * - ID: Frame identifier (see SUP_ID_* values)
 * - SIZE: Number of payload bytes (0..SUP_MAX_PAYLOAD_SIZE)
 * - PAYLOAD: Variable-length data (SIZE bytes)
 * - CHECKSUM: Single-byte checksum covering SIZE, ID and PAYLOAD
 *             (implementation-specific: commonly sum modulo 256)
 * - EOF: End-Of-Frame marker
 *
 * Assumption: The checksum covers the ID, SIZE and PAYLOAD bytes and is
 * a simple 8-bit sum (mod 256).
 */

/**
 * SUP Frame IDs
 *
 * ACK:
 * +------+------+-----------------------------------+
 * | ID   | SIZE |                PAYLOAD            |
 * +------+------+------------+----------------------+
 * | 0x01 | 1~2  |  ACK'ed ID |  Optional ACK Info   |
 * +------+------+------------+----------------------+
 *
 * NACK:
 * +------+------+-----------------------------------+
 * | ID   | SIZE |                PAYLOAD            |
 * +------+------+------------+----------------------+
 * | 0x02 | 1~2  | NACK'ed ID |  Optional NACK Info  |
 * +------+------+------------+----------------------+
 *
 * DATA:
 * +------+------+-------------------+
 * |  ID  | SIZE |      PAYLOAD      |
 * +------+------+-------------------+
 * | 0x03 |  N   |  N bytes of data  |
 * +------+------+-------------------+
 *
 * CMD_FW_UPDATE:
 * +------+------+-------------------+
 * |  ID  | SIZE |      PAYLOAD      |
 * +------+------+-------------------+
 * | 0x11 |  0   |    No payload     |
 * +------+------+-------------------+
 *
 */

enum
{
    //---- Basic IDs ----
    SUP_ID_ACK  = 0x01, ///< Acknowledgment for received frame
    SUP_ID_NACK = 0x02, ///< Negative Acknowledgment for received frame

    /**
     * @brief Binary data chunk transfer
     *
     * @details This frame ID is used to send a chunk of binary data
     * from the host to the MCU during a firmware update process.
     * The payload contains a portion of the binary data to be written
     * to the MCU's Flash memory.
     * Each DATA frame will be ACKed by the MCU upon successful reception,
     * or NACKed if there was an error.
     */
    SUP_ID_DATA = 0x03, ///< Binary data chunk transfer

    //---- Commands ----

    /**
     * @brief Command to initiate firmware update
     *
     * @details
     * This command is issued by the host to tell the MCU that a
     * firmware update sequence will begin. The exchange is a
     * simple request/acknowledge/data flow optimized for small embedded
     * systems (e.g. AVR).
     *
     * Transfer overview (host -> MCU):
     *  1) Host -> MCU : CMD_FW_UPDATE
     *  2) MCU  -> Host : ACK for CMD_FW_UPDATE
     *  3) Host -> MCU : DATA frame containing total firmware size
     *                   (2 bytes, uint16_t, little-endian)
     *  4) MCU  -> Host : ACK for firmware size
     *  5) Host -> MCU : DATA frames containing firmware payload chunks
     *                   (each DATA frame is ACKed by the MCU)
     *  6) Repeat step 5 until full firmware is transferred
     *
     * Note:
     *  - The firmware size is transmitted as a 16-bit unsigned integer in
     *    little-endian format. This is sufficient for typical AVR flash
     *    sizes and keeps the protocol minimal.
     */
    SUP_ID_CMD_FW_UPDATE = 0x11,
};

/**
 * @enum sup_frame_parsing_state_t
 * @brief SUP protocol frame parsing states
 */
typedef enum
{
    SUP_STATE_WAIT_SOF = 0,      ///< Waiting for start-of-frame byte
    SUP_STATE_WAIT_ID,           ///< Waiting for frame ID byte
    SUP_STATE_WAIT_PAYLOAD_SIZE, ///< Waiting for payload size byte
    SUP_STATE_GET_PAYLOAD,       ///< Receiving payload bytes
    SUP_STATE_WAIT_CHECKSUM,     ///< Waiting for checksum byte
    SUP_STATE_WAIT_EOF           ///< Waiting for end-of-frame byte
} sup_frame_parsing_state_t;

/**
 * @enum sup_frame_parsing_result_t
 * @brief SUP frame parsing result
 */
typedef enum
{
    SUP_RESULT_PENDING = 0,    ///< Frame is still being received
    SUP_RESULT_SUCCESS,        ///< Frame is received and verified
    SUP_RESULT_ERROR_SIZE,     ///< Invalid payload size
    SUP_RESULT_ERROR_CHECKSUM, ///< Invalid checksum
    SUP_RESULT_ERROR_EOF,      ///< Invalid EOF byte
} sup_frame_parsing_result_t;

/**
 * @struct sup_frame_t
 * @brief SUP protocol frame structure
 */
typedef struct
{
    uint8_t id;                            ///< Frame identifier
    uint8_t payload_size;                  ///< Size of payload in bytes
    uint8_t payload[SUP_MAX_PAYLOAD_SIZE]; ///< Payload data
} sup_frame_t;

/**
 * @struct sup_rx_frame_state_t
 * @brief SUP protocol state machine structure (publicly accessible for user's
 * application)
 */
typedef struct
{
    sup_frame_parsing_state_t  parsing_state;  ///< State of frame parsing
    uint8_t                    payload_index;  ///< Index of frame payload
    uint8_t                    checksum;       ///< Calculated frame checksum
    sup_frame_parsing_result_t parsing_result; ///< Result of frame parsing
    sup_frame_t                frame;          ///< Parsed frame

} sup_rx_frame_state_t;

/**
 * @brief Initialize the SUP protocol
 * @param user_rx_frame_state Pointer to frame reception state structure in
 * user's app
 */
void sup_init(sup_rx_frame_state_t* user_rx_frame_state);

/**
 * @brief Get the current SUP frame reception state
 * @return Pointer to the frame reception state
 */
sup_rx_frame_state_t* sup_get_rx_state();

/**
 * @brief Process a received byte for SUP protocol.
 * @param rx_byte Received byte
 */
void sup_handle_rx_byte(const uint8_t rx_byte);

/**
 * @brief Send a SUP protocol frame.
 * @param id Frame identifier
 * @param payload Pointer to payload data
 * @param payload_size Size of payload in bytes
 */
void sup_send_frame(const uint8_t id, const uint8_t* payload, const uint8_t payload_size);

/**
 * @brief Send a byte(platform-specific, must be implemented by the user).
 * @param tx_byte Byte to send
 */
void sup_send_byte(const uint8_t tx_byte);

/**
 * @brief Send acknowledgment frame (ACK) for a received frame ID
 * @param ack_for_id Frame ID being acknowledged
 * @param optional_byte Optional additional byte providing meta-information
 *                      (e.g., status code). Can be NULL if not needed.
 */
void sup_send_ack(const uint8_t ack_for_id, const uint8_t* optional_byte);

/**
 * @brief Send negative acknowledgment frame (NACK) for a received frame ID
 * @param nack_for_id Frame ID being negatively acknowledged
 * @param optional_byte Optional additional byte providing meta-information
 *                      (e.g., error code). Can be NULL if not needed.
 */
void sup_send_nack(const uint8_t nack_for_id, const uint8_t* optional_byte);

/**
 * @brief Send data frame
 * @param payload Pointer to data payload
 * @param payload_size Size of data payload in bytes
 */
void sup_send_data(const uint8_t* payload, const uint8_t payload_size);

#endif // SUP_H
