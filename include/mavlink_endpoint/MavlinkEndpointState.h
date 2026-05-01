#pragma once

namespace pendarlab::lib::comm{
  
    enum class MavlinkEndpointState{
      DISCONNECTED,
      CONNECTING,
      CONNECTED,
      DISCONNECTING
    };
} // namespace pendarlab::lib::comm