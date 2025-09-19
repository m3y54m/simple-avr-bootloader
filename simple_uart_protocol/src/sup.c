
/**
 * @file sup.c
 * @brief Simple UART Protocol (SUP) implementation
 *
 * @warning This code is provided for educational purposes only and is not
 * intended for production use. Use at your own risk. No warranty is provided.
 */

#include "sup.h"
#include <string.h>

// =====================
// Private Variables
// =====================

static sup_rx_frame_state_t* rx_frame_state = NULL;

// =====================
// Private Functions
// =====================

/**
 * @brief Reset the reception state
 */
static void reset_rx_frame_state()
{
    if (rx_frame_state == NULL)
        return;

    rx_frame_state->parsing_state  = SUP_STATE_WAIT_SOF;
    rx_frame_state->payload_index  = 0;
    rx_frame_state->checksum       = 0;
    rx_frame_state->parsing_result = SUP_RESULT_PENDING;
    memset(&rx_frame_state->frame, 0, sizeof(sup_frame_t));
}

/**
 * @brief Finish frame parsing with a final result
 * @param final_result Final parsing result
 */
static void finish_frame_parsing(sup_frame_parsing_result_t final_result)
{
    rx_frame_state->parsing_result = final_result;
    rx_frame_state->parsing_state  = SUP_STATE_WAIT_SOF;
}

/**
 * @brief Calculate checksum for a SUP frame.
 * @param id Frame ID
 * @param payload_size Size of payload
 * @param payload Pointer to payload data
 * @return uint8_t Calculated checksum
 */
static uint8_t calculate_checksum(uint8_t id, uint8_t payload_size, const uint8_t* payload)
{
    uint8_t checksum = 0;
    checksum += payload_size;
    checksum += id;
    for (uint8_t i = 0; i < payload_size; i++)
    {
        checksum += payload[i];
    }
    return checksum;
}

// =====================
// Public Functions
// =====================

void sup_init(sup_rx_frame_state_t* user_rx_frame_state)
{
    rx_frame_state = user_rx_frame_state;
    reset_rx_frame_state();
}

sup_rx_frame_state_t* sup_get_rx_state()
{
    return rx_frame_state;
}

void sup_handle_rx_byte(const uint8_t byte)
{
    if (rx_frame_state == NULL)
        return;

    switch (rx_frame_state->parsing_state)
    {
        case SUP_STATE_WAIT_SOF:
            if (byte == SUP_SOF)
            {
                reset_rx_frame_state();
                rx_frame_state->parsing_state = SUP_STATE_WAIT_ID;
            }
            break;
        case SUP_STATE_WAIT_ID:
            rx_frame_state->frame.id      = byte;
            rx_frame_state->checksum      = byte;
            rx_frame_state->parsing_state = SUP_STATE_WAIT_PAYLOAD_SIZE;
            break;
        case SUP_STATE_WAIT_PAYLOAD_SIZE:
            if (byte > SUP_MAX_PAYLOAD_SIZE)
            {
                finish_frame_parsing(SUP_RESULT_ERROR_SIZE);
                return;
            }
            rx_frame_state->frame.payload_size = byte;
            rx_frame_state->checksum += byte;
            if (rx_frame_state->frame.payload_size == 0)
            {
                rx_frame_state->parsing_state = SUP_STATE_WAIT_CHECKSUM;
            }
            else
            {
                rx_frame_state->parsing_state = SUP_STATE_GET_PAYLOAD;
            }
            break;
        case SUP_STATE_GET_PAYLOAD:
            rx_frame_state->frame.payload[rx_frame_state->payload_index++] = byte;
            rx_frame_state->checksum += byte;
            if (rx_frame_state->payload_index >= rx_frame_state->frame.payload_size)
            {
                rx_frame_state->parsing_state = SUP_STATE_WAIT_CHECKSUM;
            }
            break;
        case SUP_STATE_WAIT_CHECKSUM:
            if (byte == rx_frame_state->checksum)
            {
                rx_frame_state->parsing_state = SUP_STATE_WAIT_EOF;
            }
            else
            {
                finish_frame_parsing(SUP_RESULT_ERROR_CHECKSUM);
                return;
            }
            break;
        case SUP_STATE_WAIT_EOF:
            if (byte == SUP_EOF)
            {
                finish_frame_parsing(SUP_RESULT_SUCCESS);
                return;
            }
            else
            {
                finish_frame_parsing(SUP_RESULT_ERROR_EOF);
                return;
            }
    }
}

void sup_send_frame(const uint8_t id, const uint8_t* payload, const uint8_t payload_size)
{
    if (payload_size > SUP_MAX_PAYLOAD_SIZE)
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
