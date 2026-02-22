#ifndef EXTERNAL_NAVIGATION_PROTOCOL_H
#define EXTERNAL_NAVIGATION_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * External navigation protocol definitions shared with the Android app.
 *
 * Wire format for every message (BIG_ENDIAN):
 *   - message type: 4 bytes
 *   - total length: 4 bytes (type + length + message-specific header + attributes)
 *   - message-specific header: 8 bytes for currently defined messages
 *   - attributes: 0..N TLV attributes
 *
 * Attribute TLV format (BIG_ENDIAN):
 *   - attribute type: 2 bytes
 *   - attribute length: 2 bytes (type + length + payload)
 *   - payload: variable length
 */

/** Multi-byte integer fields in the wire protocol are big-endian. */
#define ENP_PROTOCOL_BIG_ENDIAN 1u

/** Common fixed field widths in bytes. */
#define ENP_MESSAGE_TYPE_SIZE_BYTES 4u
#define ENP_MESSAGE_LENGTH_SIZE_BYTES 4u
#define ENP_ATTRIBUTE_TYPE_SIZE_BYTES 2u
#define ENP_ATTRIBUTE_LENGTH_SIZE_BYTES 2u

/** Current fixed header size used by all defined message types. */
#define ENP_FIXED_HEADER_SIZE_BYTES 8u

/** Message type IDs. */
typedef enum enp_message_type {
    ENP_MESSAGE_TYPE_INVALID = 0,
    ENP_MESSAGE_TYPE_HANDSHAKE = 1,
    ENP_MESSAGE_TYPE_DESTINATION = 2,
    ENP_MESSAGE_TYPE_MOVEMENT = 3,
    ENP_MESSAGE_TYPE_DESTINATION_REQUEST = 4,
    ENP_MESSAGE_TYPE_CAP_DIRECTION = 5,
    ENP_MESSAGE_TYPE_CAP_DIRECTION_REQUEST_START = 6,
    ENP_MESSAGE_TYPE_CAP_DIRECTION_REQUEST_STOP = 7,
    ENP_MESSAGE_TYPE_CAP_STATE = 8,
    ENP_MESSAGE_TYPE_DEBUG_LOG = 9,
} enp_message_type_t;

struct msg_header {
	uint32_t type;
	uint32_t msg_len;
};

#define MSG_PAYLOAD(hdr) (((uint8_t *)(hdr)) + sizeof(msg_header))
/** CAP operational state enum used in CAP_STATE messages. */
typedef enum enp_cap_state {
    ENP_CAP_STATE_UNKNOWN = 0,
    ENP_CAP_STATE_CALIBRATING = 1,
    ENP_CAP_STATE_NAVIGATING = 2,
    ENP_CAP_STATE_ERROR = 3,
} enp_cap_state_t;

/** Common attribute type IDs. */
typedef enum enp_attribute_type {
    ENP_ATTRIBUTE_TYPE_TEXT_UTF8 = 1,
} enp_attribute_type_t;

/** Optional TLV attribute descriptor (host representation). */
typedef struct enp_attribute {
    uint16_t type;
    const uint8_t* payload;
    uint16_t payload_size;
} enp_attribute_t;

/** HANDSHAKE message-specific header (host representation). */
typedef struct enp_handshake_header {
    uint32_t protocol_version;
    uint32_t capabilities_flags;
} enp_handshake_header_t;

/** DESTINATION message-specific header (host representation). */
typedef struct enp_destination_header {
    uint32_t direction;
    uint32_t distance_meters;
} enp_destination_header_t;

/** MOVEMENT message-specific header (host representation). */
typedef struct enp_movement_header {
    uint32_t direction;
    uint32_t speed_centimeters_per_second;
} enp_movement_header_t;

/** CAP_DIRECTION message-specific header (host representation). */
typedef struct enp_cap_direction_header {
    uint32_t direction;
    uint32_t reserved;
} enp_cap_direction_header_t;

/** DESTINATION_REQUEST message-specific header (host representation). */
typedef struct enp_destination_request_header {
    uint32_t reserved0;
    uint32_t reserved1;
} enp_destination_request_header_t;

/** CAP_DIRECTION_REQUEST message-specific header (host representation). */
typedef struct enp_cap_direction_request_header {
    uint32_t reserved0;
    uint32_t reserved1;
} enp_cap_direction_request_header_t;

/** CAP_STATE message-specific header (host representation). */
typedef struct enp_cap_state_header {
    uint32_t state;
    uint32_t reserved;
} enp_cap_state_header_t;

/** DEBUG_LOG message-specific header (host representation). */
typedef struct enp_debug_log_header {
    uint32_t severity;
    uint32_t reserved;
} enp_debug_log_header_t;

/** Returns encoded TLV size (type + length + payload) for one attribute. */
static inline size_t enp_attribute_encoded_size(uint16_t payload_size) {
    return (size_t)ENP_ATTRIBUTE_TYPE_SIZE_BYTES +
           (size_t)ENP_ATTRIBUTE_LENGTH_SIZE_BYTES +
           (size_t)payload_size;
}

/** Returns total encoded message size for 8-byte message-specific headers. */
static inline size_t enp_message_encoded_size(size_t attributes_total_size) {
    return (size_t)ENP_MESSAGE_TYPE_SIZE_BYTES +
           (size_t)ENP_MESSAGE_LENGTH_SIZE_BYTES +
           (size_t)ENP_FIXED_HEADER_SIZE_BYTES +
           attributes_total_size;
}

#ifdef __cplusplus
}
#endif

#endif /* EXTERNAL_NAVIGATION_PROTOCOL_H */
