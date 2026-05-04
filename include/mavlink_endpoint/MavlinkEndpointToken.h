#pragma once

#include <memory>

namespace pendarlab::lib::comm
{
  class MavlinkEndpoint;

  class MavlinkEndpointToken
  {
  public:
    static std::unique_ptr<MavlinkEndpointToken> create(const std::weak_ptr<MavlinkEndpoint>& p_mavlink_endpoint);
    MavlinkEndpointToken(MavlinkEndpointToken&&) noexcept;            // Declare move constructor which will be defined as default
    MavlinkEndpointToken& operator=(MavlinkEndpointToken&&) noexcept; // Declare move assignment which will be defined as default
    ~MavlinkEndpointToken();
    void release();
    unsigned int getID() const;

  private:
    MavlinkEndpointToken(const std::weak_ptr<MavlinkEndpoint>& p_mavlink_endpoint);
    struct MavlinkEndpointTokenImpl;
    std::unique_ptr<MavlinkEndpointTokenImpl> p_impl_;
  };
} // namespace pendarlab::lib::comm
