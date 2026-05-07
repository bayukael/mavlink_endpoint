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

    static unsigned int next_id_;
    const unsigned int id_;
    std::weak_ptr<MavlinkEndpoint> mavlink_endpoint_;
  };

  MavlinkEndpointToken::MavlinkEndpointTokenImpl::MavlinkEndpointTokenImpl(const std::weak_ptr<MavlinkEndpoint>& p) :
      id_(next_id_++), mavlink_endpoint_(p)
  {
  }

  MavlinkEndpointToken::MavlinkEndpointTokenImpl::~MavlinkEndpointTokenImpl()
  {
    unregister();
  }

  void MavlinkEndpointToken::MavlinkEndpointTokenImpl::unregister()
  {
    auto endpoint = mavlink_endpoint_.lock();
    if (endpoint) {
      endpoint->removeListener(id_);
    }
    mavlink_endpoint_.reset();
  }

  unsigned MavlinkEndpointToken::MavlinkEndpointTokenImpl::next_id_(0);

  MavlinkEndpointToken::MavlinkEndpointToken(const std::weak_ptr<MavlinkEndpoint>& p) :
      p_impl_(std::make_unique<MavlinkEndpointTokenImpl>(p))
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
    return p_impl_->id_;
  }

  void MavlinkEndpointToken::release()
  {
    p_impl_->unregister();
  }
} // namespace pendarlab::lib::comm