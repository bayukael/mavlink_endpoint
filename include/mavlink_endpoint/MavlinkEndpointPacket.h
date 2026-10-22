#pragma once

#include <mavlink/common/mavlink.h>

namespace pendarlab::lib::comm{
  /**
   * @brief A decoded MAVLink message and its framing status.
   *
   * Passed to listener callbacks for every message successfully parsed from the
   * connected transport.
   */
  struct MavlinkEndpointPacket{
    mavlink_message_t msg;    ///< The parsed MAVLink message.
    mavlink_status_t status;  ///< Framing/parsing status associated with the message.
  };
} // namespace pendarlab::lib::comm