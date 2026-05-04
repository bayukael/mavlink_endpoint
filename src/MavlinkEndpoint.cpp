#include "mavlink_endpoint/MavlinkEndpoint.h"

#include <atomic>
#include <byte_transport/ByteTransportFactory.h>
#include <byte_transport/IByteTransport.h>
#include <condition_variable>
#include <mavlink_endpoint/MavlinkEndpointPacket.h>
#include <mavlink_endpoint/MavlinkEndpointState.h>
#include <mavlink_endpoint/MavlinkEndpointToken.h>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <unordered_map>

namespace pendarlab::lib::comm
{
  struct MavlinkEndpoint::MavlinkEndpointImpl {
    MavlinkEndpointImpl();
    MavlinkEndpointImpl(MavlinkEndpointImpl&&) = default;
    MavlinkEndpointImpl& operator=(MavlinkEndpointImpl&&) = default;
    ~MavlinkEndpointImpl();

    void waitForConnectionAndListener();
    void listeningRoutine();
    std::optional<MavlinkEndpointPacket> processMavlinkMsgByte(uint8_t c);
    void setState(const MavlinkEndpointState& s);

    std::atomic<bool> keep_running_;
    std::condition_variable_any listening_thread_cv_;
    std::mutex registry_mtx_;
    std::unordered_map<int, std::function<void(const MavlinkEndpointPacket&)>> listener_cb_registry_;
    std::mutex connection_mtx_;
    std::shared_ptr<IByteTransport> byte_transport_;
    mavlink_message_t msg_buffer_;
    mavlink_status_t stat_buffer_;
    std::mutex state_mtx_;
    MavlinkEndpointState state_;
    std::thread listening_thread_;
  };

  MavlinkEndpoint::MavlinkEndpointImpl::MavlinkEndpointImpl() :
      keep_running_(true), listening_thread_(&MavlinkEndpointImpl::listeningRoutine, this), state_(MavlinkEndpointState::DISCONNECTED)
  {
  }

  MavlinkEndpoint::MavlinkEndpointImpl::~MavlinkEndpointImpl(){
    keep_running_ = false;
    if(listening_thread_.joinable()){
      listening_thread_.join();
    }
  }

  void MavlinkEndpoint::MavlinkEndpointImpl::waitForConnectionAndListener()
  {
    std::scoped_lock lock(registry_mtx_, connection_mtx_);
    listening_thread_cv_.wait(lock, [&]{return (!keep_running_ || ( !listener_cb_registry_.empty() && byte_transport_!=nullptr));});
  }

  void MavlinkEndpoint::MavlinkEndpointImpl::listeningRoutine()
  {
    while (keep_running_) {
      waitForConnectionAndListener();

      while (true) {
        std::shared_ptr<IByteTransport> transport;
        std::unordered_map<int, std::function<void(const MavlinkEndpointPacket&)>> registry;
        {
          std::lock_guard lock(connection_mtx_);
          transport = byte_transport_;
        }
        {
          std::lock_guard lock(registry_mtx_);
          registry = listener_cb_registry_;
        }
        if (!keep_running_ || !transport || registry.empty()) { // If transport does not exist or registry is empty
          break;
        }

        const size_t buf_size = 300; // A Mavlink message might contain maximum 280 bytes. We put 300 because this is SPARTAAA.
        uint8_t buf[buf_size];
        int bytes_read = transport->read(buf, buf_size);

        for (size_t i = 0; i < bytes_read; i++) {
          auto process_result = processMavlinkMsgByte(buf[i]);
          if (process_result.has_value()) {
            for (auto [id, cb] : registry) { // Call all callbacks
              cb(process_result.value());
            }
          }
        }
      }
    }
  }

  std::optional<MavlinkEndpointPacket> MavlinkEndpoint::MavlinkEndpointImpl::processMavlinkMsgByte(uint8_t c)
  {
    mavlink_message_t msg;
    mavlink_status_t status;

    uint8_t mavlink_message_received = mavlink_frame_char_buffer(&msg_buffer_, &stat_buffer_, c, &msg, &status);
    if (mavlink_message_received == MAVLINK_FRAMING_BAD_CRC || mavlink_message_received == MAVLINK_FRAMING_BAD_SIGNATURE) {
      _mav_parse_error(&stat_buffer_);
      stat_buffer_.msg_received = MAVLINK_FRAMING_INCOMPLETE;
      stat_buffer_.parse_state = MAVLINK_PARSE_STATE_IDLE;
      if (c == MAVLINK_STX) {
        stat_buffer_.parse_state = MAVLINK_PARSE_STATE_GOT_STX;
        msg_buffer_.len = 0;
        mavlink_start_checksum(&msg_buffer_);
      }
      mavlink_message_received = 0;
    }

    if (mavlink_message_received == 1) {
      MavlinkEndpointPacket result;
      result.msg = msg;
      result.status = status;
      return result;
    }
    return std::nullopt;
  }

  void MavlinkEndpoint::MavlinkEndpointImpl::setState(const MavlinkEndpointState& s)
  {
    std::lock_guard lock(state_mtx_);
    state_ = s;
  }

  std::shared_ptr<MavlinkEndpoint> MavlinkEndpoint::create()
  {
    return std::make_shared<MavlinkEndpoint>(MavlinkEndpoint());
  }

  MavlinkEndpoint::MavlinkEndpoint(MavlinkEndpoint&&) noexcept = default;
  MavlinkEndpoint& MavlinkEndpoint::operator=(MavlinkEndpoint&&) noexcept = default;
  MavlinkEndpoint::~MavlinkEndpoint() = default;

  MavlinkEndpoint::MavlinkEndpoint() : p_impl_(std::make_unique<MavlinkEndpointImpl>())
  {
  }

  std::unique_ptr<MavlinkEndpointToken> MavlinkEndpoint::createListener(std::function<void(const MavlinkEndpointPacket&)> listener_cb)
  {
    auto token = MavlinkEndpointToken::create(shared_from_this());
    {
      std::lock_guard lock(p_impl_->registry_mtx_);
      p_impl_->listener_cb_registry_[token->getID()] = listener_cb;
    }
    p_impl_->listening_thread_cv_.notify_one();
    return token;
  }

  int MavlinkEndpoint::writeMessage(const mavlink_message_t& msg)
  {
    uint8_t write_buffer[300]; // A Mavlink message might contain maximum 280 bytes. We put 300 because this is SPARTAAA.
    unsigned int len = mavlink_msg_to_send_buffer(write_buffer, &msg);
    std::shared_ptr<IByteTransport> transport;
    {
      std::lock_guard lock(p_impl_->connection_mtx_);
      transport = p_impl_->byte_transport_;
    }
    if(!transport){
      return -1;
    }
    int bytes_written = transport->write(write_buffer, len);
    return bytes_written;
  }

  bool MavlinkEndpoint::connect(const std::string& type, const std::unordered_map<std::string, std::string>& cfg)
  {
    if (getState() != MavlinkEndpointState::DISCONNECTED) { // Connection process happens only if it is disconnected
      return false;
    }
    p_impl_->setState(MavlinkEndpointState::CONNECTING);

    auto validation_result = ByteTransportFactory::validateConfig(type, cfg);
    if (!validation_result.ok) {
      p_impl_->setState(MavlinkEndpointState::DISCONNECTED);
      return false;
    }

    auto transport = ByteTransportFactory::create(type, cfg);
    if (!transport) {
      p_impl_->setState(MavlinkEndpointState::DISCONNECTED);
      return false;
    }
    {
      std::lock_guard lock(p_impl_->connection_mtx_);
      p_impl_->byte_transport_ = std::move(transport);
    }
    p_impl_->setState(MavlinkEndpointState::CONNECTED);
    p_impl_->listening_thread_cv_.notify_one();
    return true;
  }

  bool MavlinkEndpoint::disconnect()
  {
    auto state = getState();
    switch (state) {
      case MavlinkEndpointState::CONNECTING: return false;

      case MavlinkEndpointState::CONNECTED:
        p_impl_->setState(MavlinkEndpointState::DISCONNECTING);
        {
          std::lock_guard lock(p_impl_->connection_mtx_);
          p_impl_->byte_transport_ = nullptr;
        }
        p_impl_->setState(MavlinkEndpointState::DISCONNECTED);

      default: break;
    }
    return true;
  }

  std::vector<unsigned int> MavlinkEndpoint::getListenersID()
  {
    std::vector<unsigned int> listenerID_list;
    for (auto [id, cb] : p_impl_->listener_cb_registry_) {
      listenerID_list.push_back(id);
    }
    return listenerID_list;
  }

  size_t MavlinkEndpoint::getNumOfListener()
  {
    size_t num_of_listener;
    {
      std::lock_guard lock(p_impl_->registry_mtx_);
      num_of_listener = p_impl_->listener_cb_registry_.size();
    }
    return num_of_listener;
  }

  MavlinkEndpointState MavlinkEndpoint::getState()
  {
    std::lock_guard lock(p_impl_->state_mtx_);
    return p_impl_->state_;
  }

  bool MavlinkEndpoint::removeListener(const int& token_id)
  {
    int result;
    {
      std::lock_guard lock(p_impl_->registry_mtx_);
      result = p_impl_->listener_cb_registry_.erase(token_id);
    }
    return result > 0 ? true : false;
  }

} // namespace pendarlab::lib::comm