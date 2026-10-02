#pragma once

namespace pendarlab::lib::comm{
  /**
   * @brief Lifecycle state of a MavlinkEndpoint.
   *
   * An endpoint is created DISCONNECTED, moves through CONNECTING to CONNECTED
   * when connect() succeeds, and returns to DISCONNECTED via disconnect().
   */
    enum class MavlinkEndpointState{
      DISCONNECTED, ///< No transport is attached.
      CONNECTING,   ///< connect() is in progress; the endpoint rejects new connections.
      CONNECTED,    ///< A transport is attached and messages can be exchanged.
      DISCONNECTING ///< disconnect() has been requested; not yet fully torn down.
    };
} // namespace pendarlab::lib::comm