#pragma once

#include <functional>
#include <mavlink/common/mavlink.h>
#include <mavlink_endpoint/MavlinkEndpointPacket.h>
#include <mavlink_endpoint/MavlinkEndpointState.h>
#include <mavlink_endpoint/MavlinkEndpointToken.h>
#include <memory>
#include <unordered_map>
#include <vector>

namespace pendarlab::lib::comm
{
  class MavlinkEndpoint : public std::enable_shared_from_this<MavlinkEndpoint>
  {
  public:
    static std::shared_ptr<MavlinkEndpoint> create();

    MavlinkEndpoint(MavlinkEndpoint&&) noexcept;            // Declare move constructor which will be defined as default
    MavlinkEndpoint& operator=(MavlinkEndpoint&&) noexcept; // Declare move assignment which will be defined as default
    ~MavlinkEndpoint();
    
    std::unique_ptr<MavlinkEndpointToken> createListener(std::function<void(const MavlinkEndpointPacket&)> listener_cb);
    bool removeListener(const MavlinkEndpointToken& token);
    int writeMessage(const mavlink_message_t& msg);
    bool connect(const std::string& type, const std::unordered_map<std::string, std::string>& cfg);
    bool disconnect();
    std::vector<unsigned int> getListenersID();
    size_t getNumOfListener();
    MavlinkEndpointState getState();

  private:
    MavlinkEndpoint();
    struct MavlinkEndpointImpl;
    std::unique_ptr<MavlinkEndpointImpl> p_impl_;
  };
} // namespace pendarlab::lib::comm