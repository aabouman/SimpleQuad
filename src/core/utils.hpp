#pragma once

#include <inttypes.h>

namespace rexlab {

/**
 * @brief Make sure the message starts with the byte `msg_id`.
 * 
 * If the message does not start with a byte matching `msg_id`, find the byte with
 * `msg_id`, shift the data, and report how many extra bytes need to be read to 
 * complete the message.
 * 
 * TODO: Handle the case when the message ID is not found at all.
 * 
 * @param buf Data buffer
 * @param len Length of data buffer
 * @param msg_id Expected first byte
 * @return int Number of extra bytes to read. If 0, the message is valid.
 */
int VerifyRead(char* buf, int len, uint8_t msg_id);

}