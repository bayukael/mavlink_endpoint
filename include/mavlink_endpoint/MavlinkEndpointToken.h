#pragma once

#include <memory>

namespace pendarlab::lib::comm
{
  class MavlinkEndpoint;

  /**
   * @brief Handle identifying a registered listener on a MavlinkEndpoint.
   *
   * Returned by MavlinkEndpoint::createListener(). The associated listener callback
   * stays registered while this token is alive; destroying the token or calling
   * release() unregisters it. A token does not keep its endpoint alive.
   */
  class MavlinkEndpointToken
  {
  public:
    /**
     * @brief Create a listener token bound to an endpoint.
     *
     * @param p_mavlink_endpoint The endpoint the token refers to.
     * @return The new token.
     */
    static std::unique_ptr<MavlinkEndpointToken> create(const std::weak_ptr<MavlinkEndpoint>& p_mavlink_endpoint);
    MavlinkEndpointToken(MavlinkEndpointToken&&) noexcept;            // Declare move constructor which will be defined as default
    MavlinkEndpointToken& operator=(MavlinkEndpointToken&&) noexcept; // Declare move assignment which will be defined as default
    ~MavlinkEndpointToken();
    /**
     * @brief Unregister the associated listener callback immediately.
     *
     * After this call the token no longer refers to any registration. The callback
     * is unregistered even if the owning endpoint is still alive; calling release()
     * again is safe.
     */
    void release();
    /// Unique identifier of this token within its endpoint's listener registry.
    unsigned int getID() const;

  private:
    MavlinkEndpointToken(const std::weak_ptr<MavlinkEndpoint>& p_mavlink_endpoint);
    struct MavlinkEndpointTokenImpl;
    std::unique_ptr<MavlinkEndpointTokenImpl> d;
  };
} // namespace pendarlab::lib::comm
