#include "mavlink_endpoint/MavlinkEndpointToken.h"

#include <mavlink_endpoint/MavlinkEndpoint.h>

namespace pendarlab::lib::comm
{
  struct MavlinkEndpointToken::MavlinkEndpointTokenImpl {
    MavlinkEndpointTokenImpl(const std::weak_ptr<MavlinkEndpoint>& p);
    MavlinkEndpointTokenImpl(MavlinkEndpointTokenImpl&&) = default;
    MavlinkEndpointTokenImpl& operator=(MavlinkEndpointTokenImpl&&) = default;
    ~MavlinkEndpointTokenImpl();

    void unregister();

    static unsigned int next_id;
    const unsigned int id;
    std::weak_ptr<MavlinkEndpoint> mavlink_endpoint;
  };

  MavlinkEndpointToken::MavlinkEndpointTokenImpl::MavlinkEndpointTokenImpl(const std::weak_ptr<MavlinkEndpoint>& p) :
      id(next_id++), mavlink_endpoint(p)
  {
  }

  MavlinkEndpointToken::MavlinkEndpointTokenImpl::~MavlinkEndpointTokenImpl()
  {
    unregister();
  }

  void MavlinkEndpointToken::MavlinkEndpointTokenImpl::unregister()
  {
    auto endpoint = mavlink_endpoint.lock();
    if (endpoint) {
      endpoint->removeListener(id);
    }
    mavlink_endpoint.reset();
  }

  unsigned MavlinkEndpointToken::MavlinkEndpointTokenImpl::next_id(0);

  MavlinkEndpointToken::MavlinkEndpointToken(const std::weak_ptr<MavlinkEndpoint>& p) :
      d(std::make_unique<MavlinkEndpointTokenImpl>(p))
  {
  }

  MavlinkEndpointToken::~MavlinkEndpointToken()
  {
  }

  MavlinkEndpointToken::MavlinkEndpointToken(MavlinkEndpointToken&&) noexcept = default;
  MavlinkEndpointToken& MavlinkEndpointToken::operator=(MavlinkEndpointToken&&) noexcept = default;

  std::unique_ptr<MavlinkEndpointToken> MavlinkEndpointToken::create(const std::weak_ptr<MavlinkEndpoint>& p_mavlink_endpoint)
  {
    return std::make_unique<MavlinkEndpointToken>(std::move(MavlinkEndpointToken(p_mavlink_endpoint)));
  }

  unsigned int MavlinkEndpointToken::getID() const
  {
    return d->id;
  }

  void MavlinkEndpointToken::release()
  {
    d->unregister();
  }
} // namespace pendarlab::lib::comm