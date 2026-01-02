#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include "config.h"

/**
 * @brief Calculate XOR checksum for packet
 * @param data Pointer to data buffer
 * @param len Length of data (excluding checksum byte)
 * @return Calculated checksum
 */
uint8_t calculate_checksum(const uint8_t *data, size_t len);

/**
 * @brief Parse and execute received command
 * @param cmd Command packet to process
 */
void parse_command(const CommandPacket &cmd);

/**
 * @brief Command parser task (FreeRTOS)
 * Reads serial data and processes command packets
 */
void task_command_parser(void *pvParameters);

#endif // SERIAL_PROTOCOL_H
