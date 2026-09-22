#include "mavlink_endpoint/MavlinkEndpoint.h"

#include <atomic>
#include <byte_transport/Transport.h>
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
  using ByteTransport = byte_transport::Transport;
  using RegistryUserAccess = byte_transport::RegistryUserAccess;

  struct TripleLock {
    std::unique_lock<std::mutex> lk1_;
    std::unique_lock<std::mutex> lk2_;
    std::unique_lock<std::mutex> lk3_;

    TripleLock(std::mutex& m1, std::mutex& m2, std::mutex& m3);
    void lock();
    void unlock();
  };

  TripleLock::TripleLock(std::mutex& m1, std::mutex& m2, std::mutex& m3) :
      lk1_(m1, std::defer_lock), lk2_(m2, std::defer_lock), lk3_(m3, std::defer_lock)
  {
    std::lock(lk1_, lk2_, lk3_);
  }

  void TripleLock::lock()
  {
    std::lock(lk1_, lk2_, lk3_);
  }

  void TripleLock::unlock()
  {
    lk1_.unlock();
    lk2_.unlock();
    lk3_.unlock();
  }

  struct MavlinkEndpoint::MavlinkEndpointImpl {
    MavlinkEndpointImpl(const RegistryUserAccess& reg);
    MavlinkEndpointImpl(MavlinkEndpointImpl&&) = default;
    MavlinkEndpointImpl& operator=(MavlinkEndpointImpl&&) = default;
    ~MavlinkEndpointImpl();

    void waitForConnectionAndListener();
    void listeningRoutine();
    std::optional<MavlinkEndpointPacket> processMavlinkMsgByte(uint8_t c);
    void setState(const MavlinkEndpointState& s);
    bool keepRunning();

    const RegistryUserAccess& registry;
    std::mutex running_mtx;
    bool keep_running;
    std::condition_variable_any listening_thread_cv;
    std::mutex registry_mtx;
    std::unordered_map<int, std::function<void(const MavlinkEndpointPacket&)>> listener_cb_registry;
    std::mutex connection_mtx;
    std::shared_ptr<ByteTransport> byte_transport;
    mavlink_message_t msg_buffer;
    mavlink_status_t stat_buffer;
    std::mutex state_mtx;
    MavlinkEndpointState state;
    std::thread listening_thread;
  };

  MavlinkEndpoint::MavlinkEndpointImpl::MavlinkEndpointImpl(const RegistryUserAccess& reg) :
      registry(reg),
      keep_running(true),
      listening_thread(&MavlinkEndpointImpl::listeningRoutine, this),
      state(MavlinkEndpointState::DISCONNECTED),
      msg_buffer{ 0 },
      stat_buffer{ 0 }
  {
  }

  MavlinkEndpoint::MavlinkEndpointImpl::~MavlinkEndpointImpl()
  {
    {
      std::unique_lock lock(running_mtx);
      keep_running = false;
    }
    listening_thread_cv.notify_one();
    if (listening_thread.joinable()) {
      listening_thread.join();
    }
  }

  void MavlinkEndpoint::MavlinkEndpointImpl::waitForConnectionAndListener()
  {
    TripleLock lock(registry_mtx, connection_mtx, running_mtx);
    listening_thread_cv.wait(lock, [&] { return (!keep_running || (!listener_cb_registry.empty() && byte_transport != nullptr)); });
  }

  void MavlinkEndpoint::MavlinkEndpointImpl::listeningRoutine()
  {
    while (keepRunning()) {
      waitForConnectionAndListener();

      while (true) {
        std::shared_ptr<ByteTransport> transport;
        std::unordered_map<int, std::function<void(const MavlinkEndpointPacket&)>> registry;
        {
          std::lock_guard lock(connection_mtx);
          transport = byte_transport;
        }
        {
          std::lock_guard lock(registry_mtx);
          registry = listener_cb_registry;
        }
        if (!keepRunning() || !transport || registry.empty()) { // If transport does not exist or registry is empty
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

    uint8_t mavlink_message_received = mavlink_frame_char_buffer(&msg_buffer, &stat_buffer, c, &msg, &status);
    if (mavlink_message_received == MAVLINK_FRAMING_BAD_CRC || mavlink_message_received == MAVLINK_FRAMING_BAD_SIGNATURE) {
      _mav_parse_error(&stat_buffer);
      stat_buffer.msg_received = MAVLINK_FRAMING_INCOMPLETE;
      stat_buffer.parse_state = MAVLINK_PARSE_STATE_IDLE;
      if (c == MAVLINK_STX) {
        stat_buffer.parse_state = MAVLINK_PARSE_STATE_GOT_STX;
        msg_buffer.len = 0;
        mavlink_start_checksum(&msg_buffer);
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
    std::lock_guard lock(state_mtx);
    state = s;
  }

  bool MavlinkEndpoint::MavlinkEndpointImpl::keepRunning()
  {
    std::lock_guard lock(running_mtx);
    return keep_running;
  }

  std::shared_ptr<MavlinkEndpoint> MavlinkEndpoint::create(const RegistryUserAccess& reg)
  {
    return std::make_shared<MavlinkEndpoint>(MavlinkEndpoint(reg));
  }

  MavlinkEndpoint::MavlinkEndpoint(MavlinkEndpoint&&) noexcept = default;
  MavlinkEndpoint& MavlinkEndpoint::operator=(MavlinkEndpoint&&) noexcept = default;
  MavlinkEndpoint::~MavlinkEndpoint() = default;

  MavlinkEndpoint::MavlinkEndpoint(const RegistryUserAccess& reg) : d(std::make_unique<MavlinkEndpointImpl>(reg))
  {
  }

  std::unique_ptr<MavlinkEndpointToken> MavlinkEndpoint::createListener(std::function<void(const MavlinkEndpointPacket&)> listener_cb)
  {
    auto token = MavlinkEndpointToken::create(shared_from_this());
    {
      std::lock_guard lock(d->registry_mtx);
      d->listener_cb_registry[token->getID()] = listener_cb;
    }
    d->listening_thread_cv.notify_one();
    return token;
  }

  int MavlinkEndpoint::writeMessage(const mavlink_message_t& msg)
  {
    uint8_t write_buffer[300]; // A Mavlink message might contain maximum 280 bytes. We put 300 because this is SPARTAAA.
    unsigned int len = mavlink_msg_to_send_buffer(write_buffer, &msg);
    std::shared_ptr<ByteTransport> transport;
    {
      std::lock_guard lock(d->connection_mtx);
      transport = d->byte_transport;
    }
    if (!transport) {
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
    d->setState(MavlinkEndpointState::CONNECTING);

    auto transport_def = d->registry[type];
    if(!transport_def){
      d->setState(MavlinkEndpointState::DISCONNECTED);
      return false;
    }

    auto parse_result = transport_def->parseConfig(cfg);
    if(!parse_result.ok()){
      d->setState(MavlinkEndpointState::DISCONNECTED);
      return false;
    }

    auto transport = transport_def->create(parse_result.config.value());
    if(!transport){
      d->setState(MavlinkEndpointState::DISCONNECTED);
      return false;
    }

    {
      std::lock_guard lock(d->connection_mtx);
      d->byte_transport = std::move(transport);
    }
    d->setState(MavlinkEndpointState::CONNECTED);
    d->listening_thread_cv.notify_one();

    return true;
  }

  bool MavlinkEndpoint::disconnect()
  {
    auto state = getState();
    switch (state) {
      case MavlinkEndpointState::CONNECTING: return false;

      case MavlinkEndpointState::CONNECTED:
        d->setState(MavlinkEndpointState::DISCONNECTING);
        {
          std::lock_guard lock(d->connection_mtx);
          d->byte_transport = nullptr;
        }
        d->setState(MavlinkEndpointState::DISCONNECTED);

      default: break;
    }
    return true;
  }

  std::vector<unsigned int> MavlinkEndpoint::getListenersID()
  {
    std::vector<unsigned int> listenerID_list;
    for (auto [id, cb] : d->listener_cb_registry) {
      listenerID_list.push_back(id);
    }
    return listenerID_list;
  }

  size_t MavlinkEndpoint::getNumOfListener()
  {
    size_t num_of_listener;
    {
      std::lock_guard lock(d->registry_mtx);
      num_of_listener = d->listener_cb_registry.size();
    }
    return num_of_listener;
  }

  MavlinkEndpointState MavlinkEndpoint::getState()
  {
    std::lock_guard lock(d->state_mtx);
    return d->state;
  }

  bool MavlinkEndpoint::removeListener(const int& token_id)
  {
    int result;
    {
      std::lock_guard lock(d->registry_mtx);
      result = d->listener_cb_registry.erase(token_id);
    }
    return result > 0 ? true : false;
  }

} // namespace pendarlab::lib::comm