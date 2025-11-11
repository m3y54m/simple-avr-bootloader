/**
 * @file sup.c
 * @brief Simple UART Protocol (SUP) - Core Implementation
 *
 * @details
 * This file contains the implementation of the SUP state machine for parsing
 * incoming byte streams and functions for constructing and sending SUP frames.
 *
 * @warning This code is provided for educational purposes only and is not
 * intended for production use. Use at your own risk. No warranty is provided.
 *
 * @author m3y54m
 * @version 1.0.0
 * @date 2025
 * @license MIT License
 */

#include "sup.h"
#include <stddef.h> // For NULL
#include <string.h> // For memset

// ============================================================================
// Private Module Variables
// ============================================================================

/**
 * @brief Module-level pointer to the user-provided reception state structure.
 */
static sup_rx_frame_state_t* rx_frame_state = NULL;

// ============================================================================
// Private Helper Functions
// ============================================================================

/**
 * @brief Resets the frame reception state to its initial values.
 * @details Called when a new SOF is detected or on initialization to prepare
 * for a new incoming frame.
 */
static void reset_rx_frame_state(void)
{
    if (rx_frame_state == NULL)
    {
        return;
    }

    rx_frame_state->parsing_state  = SUP_STATE_WAIT_SOF;
    rx_frame_state->parsing_result = SUP_RESULT_PENDING;
    rx_frame_state->payload_index  = 0;
    rx_frame_state->checksum       = 0;
    // No need to clear the frame buffer, as its contents are only valid
    // after a successful parse, and payload_size will be set correctly.
}

/**
 * @brief Finalizes the parsing process for the current frame.
 * @param final_result The concluding status of the frame parsing.
 * @details Sets the final result and resets the parsing state to wait for the
 * next frame's SOF, making the result available to the application.
 */
static void finish_frame_parsing(sup_frame_parsing_result_t final_result)
{
    if (rx_frame_state == NULL)
    {
        return;
    }
    rx_frame_state->parsing_result = final_result;
    rx_frame_state->parsing_state  = SUP_STATE_WAIT_SOF;
}

/**
 * @brief Calculates the 8-bit checksum for a given set of frame data.
 * @param id Frame ID.
 * @param payload_size Size of the payload.
 * @param payload Pointer to the payload data.
 * @return The calculated 8-bit checksum.
 */
static uint8_t calculate_checksum(uint8_t id, uint8_t payload_size, const uint8_t* payload)
{
    uint8_t checksum = 0;
    checksum += id;
    checksum += payload_size;
    for (uint8_t i = 0; i < payload_size; i++)
    {
        checksum += payload[i];
    }
    return checksum;
}

// ============================================================================
// Public API Functions
// ============================================================================

/**
 * @brief Initializes the SUP protocol state machine.
 */
void sup_init(sup_rx_frame_state_t* user_rx_frame_state)
{
    rx_frame_state = user_rx_frame_state;
    reset_rx_frame_state();
}

/**
 * @brief Gets a pointer to the internal SUP frame reception state.
 */
sup_rx_frame_state_t* sup_get_rx_state(void)
{
    return rx_frame_state;
}

/**
 * @brief Processes one incoming byte through the SUP state machine.
 */
void sup_handle_rx_byte(const uint8_t byte)
{
    // Do nothing if the protocol has not been initialized
    if (rx_frame_state == NULL)
    {
        return;
    }

    switch (rx_frame_state->parsing_state)
    {
        case SUP_STATE_WAIT_SOF:
            // Listen for a Start-Of-Frame byte. Any byte other than SOF is ignored.
            if (byte == SUP_SOF)
            {
                reset_rx_frame_state();
                rx_frame_state->parsing_state = SUP_STATE_WAIT_ID;
            }
            break;

        case SUP_STATE_WAIT_ID:
            rx_frame_state->frame.id = byte;
            rx_frame_state->checksum = byte; // Start checksum calculation with ID
            rx_frame_state->parsing_state = SUP_STATE_WAIT_PAYLOAD_SIZE;
            break;

        case SUP_STATE_WAIT_PAYLOAD_SIZE:
            // Validate payload size against the buffer limit
            if (byte > SUP_MAX_PAYLOAD_SIZE)
            {
                finish_frame_parsing(SUP_RESULT_ERROR_SIZE);
                break;
            }
            rx_frame_state->frame.payload_size = byte;
            rx_frame_state->checksum += byte; // Add SIZE to checksum
            // If there is no payload, skip directly to waiting for the checksum
            rx_frame_state->parsing_state = (byte == 0) ? SUP_STATE_WAIT_CHECKSUM : SUP_STATE_GET_PAYLOAD;
            break;

        case SUP_STATE_GET_PAYLOAD:
            rx_frame_state->frame.payload[rx_frame_state->payload_index] = byte;
            rx_frame_state->checksum += byte; // Add payload byte to checksum
            rx_frame_state->payload_index++;
            // Check if all payload bytes have been received
            if (rx_frame_state->payload_index >= rx_frame_state->frame.payload_size)
            {
                rx_frame_state->parsing_state = SUP_STATE_WAIT_CHECKSUM;
            }
            break;

        case SUP_STATE_WAIT_CHECKSUM:
            if (byte == rx_frame_state->checksum)
            {
                // Checksum matches, proceed to wait for EOF
                rx_frame_state->parsing_state = SUP_STATE_WAIT_EOF;
            }
            else
            {
                // Checksum mismatch, abort frame
                finish_frame_parsing(SUP_RESULT_ERROR_CHECKSUM);
            }
            break;

        case SUP_STATE_WAIT_EOF:
            if (byte == SUP_EOF)
            {
                // EOF received, frame is valid and complete
                finish_frame_parsing(SUP_RESULT_SUCCESS);
            }
            else
            {
                // Incorrect byte at EOF position, abort frame
                finish_frame_parsing(SUP_RESULT_ERROR_EOF);
            }
            break;
    }
}

/**
 * @brief Constructs and sends a complete SUP frame.
 */
void sup_send_frame(const uint8_t id, const uint8_t* payload, const uint8_t payload_size)
{
    // Sanity check: ensure payload pointer is valid if size is non-zero
    if ((payload_size > 0 && payload == NULL) || payload_size > SUP_MAX_PAYLOAD_SIZE)
    {
        return;
    }

    uint8_t checksum = calculate_checksum(id, payload_size, payload);

    sup_send_byte(SUP_SOF);
    sup_send_byte(id);
    sup_send_byte(payload_size);
    for (uint8_t i = 0; i < payload_size; i++)
    {
        sup_send_byte(payload[i]);
    }
    sup_send_byte(checksum);
    sup_send_byte(SUP_EOF);
}

/**
 * @brief A helper function to send an acknowledgment (ACK) frame.
 */
void sup_send_ack(const uint8_t received_id, const uint8_t* optional_byte)
{
    // Safely handle the optional byte to prevent null pointer dereferencing
    if (optional_byte != NULL)
    {
        uint8_t payload[2] = {received_id, *optional_byte};
        sup_send_frame(SUP_ID_ACK, payload, 2);
    }
    else
    {
        uint8_t payload[1] = {received_id};
        sup_send_frame(SUP_ID_ACK, payload, 1);
    }
}

/**
 * @brief A helper function to send a negative acknowledgment (NACK) frame.
 */
void sup_send_nack(const uint8_t received_id, const uint8_t* optional_byte)
{
    // Safely handle the optional byte to prevent null pointer dereferencing
    if (optional_byte != NULL)
    {
        uint8_t payload[2] = {received_id, *optional_byte};
        sup_send_frame(SUP_ID_NACK, payload, 2);
    }
    else
    {
        uint8_t payload[1] = {received_id};
        sup_send_frame(SUP_ID_NACK, payload, 1);
    }
}

/**
 * @brief A helper function to send a data (DATA) frame.
 */
void sup_send_data(const uint8_t* payload, const uint8_t payload_size)
{
    sup_send_frame(SUP_ID_DATA, payload, payload_size);
}
