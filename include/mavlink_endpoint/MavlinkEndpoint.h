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
  class MavlinkEndpoint : public std::enable_shared_from_this<MavlinkEndpoint>
  {
  public:
    struct ValidationResult {
      bool ok;
      std::vector<std::string> msg;
    };

    static std::shared_ptr<MavlinkEndpoint> create(const byte_transport::RegistryUserAccess&); // To always create MavlinkEndpoint with make_shared

    MavlinkEndpoint(MavlinkEndpoint&&) noexcept;            // Declare move constructor which will be defined as default
    MavlinkEndpoint& operator=(MavlinkEndpoint&&) noexcept; // Declare move assignment which will be defined as default
    ~MavlinkEndpoint();

    std::unique_ptr<MavlinkEndpointToken> createListener(std::function<void(const MavlinkEndpointPacket&)> listener_cb);
    int writeMessage(const mavlink_message_t& msg);
    bool connect(const std::string& type, const std::unordered_map<std::string, std::string>& cfg);
    bool disconnect();
    std::vector<unsigned int> getListenersID();
    size_t getNumOfListener();
    MavlinkEndpointState getState();

  private:
    MavlinkEndpoint(const byte_transport::RegistryUserAccess&); // Hide this to ensure the instantiation of MavlinkEndpoint is done through MavlinkEndpoint::create()
    friend class MavlinkEndpointToken;
    bool removeListener(const int& token_id);
    struct MavlinkEndpointImpl;
    std::unique_ptr<MavlinkEndpointImpl> d;
  };
} // namespace pendarlab::lib::comm