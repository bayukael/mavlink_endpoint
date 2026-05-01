#pragma once

#include <mavlink/common/mavlink.h>

namespace pendarlab::lib::comm{
  struct MavlinkEndpointPacket{
    mavlink_message_t msg;
    mavlink_status_t status;
  };
} // namespace pendarlab::lib::comm