#pragma once
#include "mavlink_endpoint/MavlinkEndpointPacket.h"
#include "mavlink_endpoint/MavlinkEndpointState.h"
#include "mavlink_endpoint/MavlinkEndpointToken.h"

#include <byte_transport/RegistryUserAccess.h>
#include <functional>
#include <mavlink/common/mavlink.h>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace pendarlab::lib::comm
{
  /**
   * @brief A MAVLink endpoint that talks to the outside world over a byte transport.
   *
   * The endpoint decodes incoming bytes into MAVLink messages on a background
   * listening thread and forwards each parsed message to every registered listener
   * callback. Outgoing messages are serialized and written through the underlying
   * byte transport.
   *
   * An endpoint is created via create() and is always owned by a shared_ptr.
   * It starts DISCONNECTED with no listeners. Use connect() to attach a transport
   * and createListener() to receive parsed messages.
   *
   * @note Methods are thread-safe; listeners are invoked on the internal listening
   *       thread, not on the caller's thread.
   */
  class MavlinkEndpoint : public std::enable_shared_from_this<MavlinkEndpoint>
  {
  public:
    /// Result of a configuration validation: overall success flag and human-readable messages.
    struct ValidationResult {
      bool ok;
      std::vector<std::string> msg;
    };

    /**
     * @brief Create a new endpoint.
     *
     * Returns an endpoint in the DISCONNECTED state with no listeners. Always
     * instantiate through this factory so the endpoint is owned by a shared_ptr
     * (required by its internal listener/state machinery).
     *
     * @param reg Registry giving read-only access to the registered transport
     *            definitions that connect() may use.
     * @return The newly created endpoint.
     */
    static std::shared_ptr<MavlinkEndpoint> create(const byte_transport::RegistryUserAccess& reg); // To always create MavlinkEndpoint with make_shared

    MavlinkEndpoint(MavlinkEndpoint&&) noexcept;            // Declare move constructor which will be defined as default
    MavlinkEndpoint& operator=(MavlinkEndpoint&&) noexcept; // Declare move assignment which will be defined as default
    ~MavlinkEndpoint();

    /**
     * @brief Register a listener callback for incoming MAVLink messages.
     *
     * The callback is invoked on the internal listening thread for every MAVLink
     * message successfully parsed from the connected transport. It remains
     * registered until the returned token is destroyed or its release() is called.
     * The endpoint holds a shared reference to the listener via the returned token.
     *
     * @param listener_cb Callback invoked with each parsed packet. It must remain
     *                    valid for as long as the returned token is alive.
     * @return A token identifying the registration; destroying it (or calling its
     *         release()) unregisters the callback.
     */
    std::unique_ptr<MavlinkEndpointToken> createListener(std::function<void(const MavlinkEndpointPacket&)> listener_cb);
    /**
     * @brief Serialize and write a MAVLink message through the connected transport.
     *
     * @param msg The message to send.
     * @return The number of bytes written on success, or -1 if the endpoint is not
     *         connected (no transport is attached).
     */
    int writeMessage(const mavlink_message_t& msg);
    /**
     * @brief Establish a connection using a registered transport type.
     *
     * Looks up @p type in the registry, parses @p cfg, creates the transport, and
     * transitions the endpoint to CONNECTED. The connection attempt only proceeds if
     * the endpoint is currently DISCONNECTED.
     *
     * @param type Registration key of the transport definition to use.
     * @param cfg  Transport-specific configuration, validated by the transport's
     *             parseConfig().
     * @return true if the connection was established, false if the endpoint was not
     *         DISCONNECTED, the type is unregistered, or configuration/creation failed
     *         (the endpoint is left DISCONNECTED in those cases).
     */
    bool connect(const std::string& type, const std::unordered_map<std::string, std::string>& cfg);
    /**
     * @brief Tear down the current connection.
     *
     * Detaches the transport and returns the endpoint to DISCONNECTED. No-op
     * (returns true) when already DISCONNECTED or DISCONNECTING.
     *
     * @return true on success, or false if the endpoint is currently CONNECTING.
     */
    bool disconnect();
    /// IDs of all currently registered listeners.
    std::vector<unsigned int> getListenersID();
    /// Number of currently registered listeners.
    size_t getNumOfListener();
    /// Current state of the endpoint.
    MavlinkEndpointState getState();

  private:
    MavlinkEndpoint(const byte_transport::RegistryUserAccess&); // Hide this to ensure the instantiation of MavlinkEndpoint is done through MavlinkEndpoint::create()
    friend class MavlinkEndpointToken;
    bool removeListener(const int& token_id);
    struct MavlinkEndpointImpl;
    std::unique_ptr<MavlinkEndpointImpl> d;
  };
} // namespace pendarlab::lib::comm